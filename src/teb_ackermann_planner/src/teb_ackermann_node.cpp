/*
 * @Function: TEB -> 阿克曼小车最小闭环 ROS2 节点
 * @Date:2026-09-22
 *
 * 数据流：
 *   /scan            (sensor_msgs/LaserScan)     -> 障碍物点（裁剪正前方扇区 + 量程过滤 + 抽稀，按激光外参转到局部坐标）
 *   /reference_path  (nav_msgs/Path)             -> 全局参考路径（局部坐标 x,y；yaw 取四元数）
 *   /vehicle_pose    (geometry_msgs/PoseStamped) -> 当前车辆位姿（局部坐标）
 *        -> TEB 局部窗口优化 (plannerManager)
 *        -> AckermannController 由 TEB 结果换算后轮线速度 + 前轮绝对转角
 *        -> AckermannControlInterface::sendCommand()（由使用方实现）
 *
 * 所有参数均通过 ROS 参数声明，可用 launch / yaml 覆盖。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "teb_ackermann_planner/ackermann_control_interface.h"
#include "teb_ackermann_planner/ackermann_controller.h"
#include "teb_ackermann_planner/ackermann_params.h"
#include "teb_ackermann_planner/planner_manager.h"
#include "teb_ackermann_planner/tools.h"

namespace teb_local_planner
{
class TebAckermannNode : public rclcpp::Node
{
public:
    TebAckermannNode()
    : rclcpp::Node("teb_ackermann_node"),
      params_(loadAckermannParams()),
      cfg_(loadTebConfig()),
      controller_(params_),
      planner_(std::make_shared<plannerManager>(cfg_))
    {
        // 话题名做成参数，便于 remap / 配置
        scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
        path_topic_ = this->declare_parameter<std::string>("reference_path_topic", "/reference_path");
        pose_topic_ = this->declare_parameter<std::string>("vehicle_pose_topic", "/vehicle_pose");

        // 雷达处理参数（角度单位度，内部转弧度）
        scan_angle_limit_ = deg2rad(this->declare_parameter<double>("scan_angle_limit_deg", 90.0));
        scan_range_min_   = this->declare_parameter<double>("scan_range_min", 0.5);
        scan_range_max_   = this->declare_parameter<double>("scan_range_max", 5.0);
        scan_max_points_  = this->declare_parameter<int>("scan_max_points", 100);

        // 激光雷达相对车体（后轴中心）的外参，静态参数，不做 tf
        laser_x_   = this->declare_parameter<double>("laser_x", 0.0);
        laser_y_   = this->declare_parameter<double>("laser_y", 0.0);
        laser_yaw_ = deg2rad(this->declare_parameter<double>("laser_yaw_deg", 0.0));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&TebAckermannNode::scanCallback, this, std::placeholders::_1));
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&TebAckermannNode::pathCallback, this, std::placeholders::_1));
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10,
            std::bind(&TebAckermannNode::poseCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(params_.control_period),
            std::bind(&TebAckermannNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
                    "TEB 阿克曼节点启动：scan=%s, path=%s, pose=%s, 控制周期=%.3f s",
                    scan_topic_.c_str(), path_topic_.c_str(), pose_topic_.c_str(),
                    params_.control_period);
        RCLCPP_INFO(this->get_logger(),
                    "车身参数：轴距=%.2f m, 最大速度=%.2f m/s, "
                    "最大转角=%.1f deg, 回转速率=%.1f deg/s, 最小转弯半径=%.2f m",
                    params_.wheelbase, params_.max_speed,
                    rad2deg(params_.max_steering_angle), rad2deg(params_.steering_rate),
                    cfg_.min_turning_radius);
    }

    // 注入控制下发实现（未注入时只计算、不下发）
    void setControlInterface(AckermannControlInterface* api) { control_api_ = api; }

private:
    // ---------------------------------------------------------------------
    // 参数
    // ---------------------------------------------------------------------
    static constexpr double kPi = 3.14159265358979323846;
    static double deg2rad(double deg) { return deg * kPi / 180.0; }
    static double rad2deg(double rad) { return rad * 180.0 / kPi; }

    // 车身参数
    AckermannParams loadAckermannParams()
    {
        AckermannParams p;
        p.wheelbase         = this->declare_parameter<double>("wheelbase", p.wheelbase);
        p.max_speed         = this->declare_parameter<double>("max_speed", p.max_speed);
        p.control_period    = this->declare_parameter<double>("control_period", p.control_period);
        // 前轮转角按位置控制，参数以“度”配置
        p.max_steering_angle = deg2rad(this->declare_parameter<double>("max_steering_angle_deg", 18.0));
        p.steering_rate      = deg2rad(this->declare_parameter<double>("steering_rate_deg", 8.0));
        p.steering_offset    = deg2rad(this->declare_parameter<double>("steering_offset_deg", 0.0));
        return p;
    }

    // TEB 优化参数（轴距 / 速度上限与车身参数保持一致，避免两处配置冲突）
    TebConfig loadTebConfig()
    {
        TebConfig cfg;

        // 避障
        cfg.min_obstacle_dist = this->declare_parameter<double>("min_obstacle_dist", cfg.min_obstacle_dist);
        cfg.penalty_epsilon   = this->declare_parameter<double>("penalty_epsilon", cfg.penalty_epsilon);
        cfg.obstacle_weight   = this->declare_parameter<double>("obstacle_weight", cfg.obstacle_weight);
        cfg.weight_viapoint   = this->declare_parameter<double>("weight_viapoint", cfg.weight_viapoint);

        // 阿克曼运动学：最小转弯半径由轴距与前轮最大转角决定
        cfg.wheelbase          = params_.wheelbase;
        cfg.min_turning_radius = params_.wheelbase / std::tan(params_.max_steering_angle);
        cfg.weight_kinematics  = this->declare_parameter<double>("weight_kinematics", cfg.weight_kinematics);

        // 速度限幅
        cfg.max_vel_x           = params_.max_speed;
        cfg.max_vel_theta       = this->declare_parameter<double>(
            "max_vel_theta", params_.max_speed / cfg.min_turning_radius);
        cfg.weight_velocity     = this->declare_parameter<double>("weight_velocity", cfg.weight_velocity);

        // 加速度限幅
        cfg.acc_lim_x           = this->declare_parameter<double>("acc_lim_x", cfg.acc_lim_x);
        cfg.acc_lim_theta       = this->declare_parameter<double>("acc_lim_theta", cfg.acc_lim_theta);
        cfg.weight_acceleration = this->declare_parameter<double>("weight_acceleration", cfg.weight_acceleration);

        // 时间最优与时间分辨率
        cfg.weight_timeoptimal = this->declare_parameter<double>("weight_timeoptimal", cfg.weight_timeoptimal);
        cfg.dt_ref             = this->declare_parameter<double>("dt_ref", cfg.dt_ref);
        cfg.dt_min             = this->declare_parameter<double>("dt_min", cfg.dt_min);
        cfg.dt_max             = this->declare_parameter<double>("dt_max", cfg.dt_max);

        // 局部窗口
        cfg.local_window_length    = this->declare_parameter<double>("local_window_length", cfg.local_window_length);
        cfg.path_point_spacing     = this->declare_parameter<double>("path_point_spacing", cfg.path_point_spacing);
        cfg.local_window_max_poses = this->declare_parameter<int>("local_window_max_poses", cfg.local_window_max_poses);
        cfg.slow_down_distance     = this->declare_parameter<double>("slow_down_distance", cfg.slow_down_distance);
        cfg.viapoint_start_offset  = this->declare_parameter<int>("viapoint_start_offset", cfg.viapoint_start_offset);

        // 优化器
        cfg.optimization_verbose = this->declare_parameter<bool>("optimization_verbose", cfg.optimization_verbose);
        cfg.no_inner_iterations  = this->declare_parameter<int>("no_inner_iterations", cfg.no_inner_iterations);
        cfg.no_outer_iterations  = this->declare_parameter<int>("no_outer_iterations", cfg.no_outer_iterations);

        return cfg;
    }


    // ---------------------------------------------------------------------
    // 回调
    // ---------------------------------------------------------------------
    // /scan 回调：裁剪正前方扇区 + 量程过滤 + 等间隔抽稀，再按激光外参转到局部坐标系
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacles_.clear();

        const int n = static_cast<int>(msg->ranges.size());
        if (n <= 0 || !std::isfinite(msg->angle_increment) || std::fabs(msg->angle_increment) < 1e-9)
        {
            return;
        }
        if (!has_pose_)
        {
            // 没有车辆位姿就无法把雷达点转到局部坐标系
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "尚未收到车辆位姿，暂时忽略 /scan 障碍点");
            return;
        }

        // 1) 角度裁剪：只保留车头正前方 ±scan_angle_limit_（按激光外参偏置换算到雷达坐标系）
        const double a_start = std::max(static_cast<double>(msg->angle_min), -scan_angle_limit_ - laser_yaw_);
        const double a_end   = std::min(static_cast<double>(msg->angle_max), scan_angle_limit_ - laser_yaw_);
        int i_start = static_cast<int>(std::ceil((a_start - static_cast<double>(msg->angle_min)) /
                                                 static_cast<double>(msg->angle_increment)));
        int i_end   = static_cast<int>(std::floor((a_end - static_cast<double>(msg->angle_min)) /
                                                  static_cast<double>(msg->angle_increment)));
        i_start = std::max(0, std::min(i_start, n - 1));
        i_end   = std::max(0, std::min(i_end, n - 1));
        if (i_end < i_start)
        {
            return;
        }

        // 2) 抽稀：光束数超过 scan_max_points_ 时等间隔取样
        const int step = std::max(1, (i_end - i_start + 1) / std::max(1, scan_max_points_));

        // 3) 有效量程 = 参数限幅 与 雷达自身量程 的交集
        double range_min = scan_range_min_;
        double range_max = scan_range_max_;
        if (std::isfinite(msg->range_min) && msg->range_min > range_min)
        {
            range_min = static_cast<double>(msg->range_min);
        }
        if (std::isfinite(msg->range_max) && msg->range_max < range_max)
        {
            range_max = static_cast<double>(msg->range_max);
        }
        if (range_max <= range_min)
        {
            return;
        }

        const double cos_l = std::cos(laser_yaw_);
        const double sin_l = std::sin(laser_yaw_);
        const double cos_v = std::cos(static_cast<double>(current_pose_.theta));
        const double sin_v = std::sin(static_cast<double>(current_pose_.theta));

        obstacles_.reserve(static_cast<size_t>((i_end - i_start) / step + 1));
        for (int i = i_start; i <= i_end; i += step)
        {
            const double r = static_cast<double>(msg->ranges[i]);
            if (!std::isfinite(r) || r < range_min || r > range_max)
            {
                continue;
            }

            const double angle = static_cast<double>(msg->angle_min) +
                                 static_cast<double>(i) * static_cast<double>(msg->angle_increment);
            // 雷达坐标系 -> 车体坐标系
            const double lx = r * std::cos(angle);
            const double ly = r * std::sin(angle);
            const double vx = cos_l * lx - sin_l * ly + laser_x_;
            const double vy = sin_l * lx + cos_l * ly + laser_y_;
            // 车体坐标系 -> 局部坐标系（用当前车辆位姿）
            tools::obstacleInfo obs;
            obs.x     = static_cast<float>(static_cast<double>(current_pose_.x) + cos_v * vx - sin_v * vy);
            obs.y     = static_cast<float>(static_cast<double>(current_pose_.y) + sin_v * vx + cos_v * vy);
            obs.theta = 0.0f;
            obstacles_.push_back(obs);
        }

        RCLCPP_DEBUG(this->get_logger(), "收到 /scan，障碍物点 %zu 个", obstacles_.size());
    }


    // 参考路径回调：nav_msgs/Path（局部坐标）-> tools::pathInfo
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::vector<tools::pathInfo> path;
        path.reserve(msg->poses.size());
        for (const auto& pose_stamped : msg->poses)
        {
            tools::pathInfo p;
            p.x     = static_cast<float>(pose_stamped.pose.position.x);
            p.y     = static_cast<float>(pose_stamped.pose.position.y);
            p.theta = static_cast<float>(quaternionToYaw(pose_stamped.pose.orientation));
            p.dt    = 0.0f;
            path.push_back(p);
        }

        reference_path_ = path;
        controller_.reset();   // 新参考路径：清掉上一段的转角与最近点状态
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "收到参考路径 %zu 个点", reference_path_.size());
    }

    // 当前车辆位姿回调（局部坐标，由使用方自己完成 GPS -> 局部坐标的换算）
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_.x     = static_cast<float>(msg->pose.position.x);
        current_pose_.y     = static_cast<float>(msg->pose.position.y);
        current_pose_.theta = static_cast<float>(quaternionToYaw(msg->pose.orientation));
        current_pose_.dt    = 0.0f;
        has_pose_           = true;
    }

    // 定时器回调：TEB 局部优化 -> 阿克曼指令 -> 下发接口
    void controlLoop()
    {
        if (reference_path_.size() < 2 || !has_pose_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "等待参考路径(%zu 点)与当前位姿(收到=%d)...",
                                 reference_path_.size(), static_cast<int>(has_pose_));
            return;
        }

        // 1) TEB 优化：参考路径 + 最新一帧障碍点 + 当前车辆位姿（局部窗口起点）
        planner_->setpathInfo(reference_path_);
        planner_->setObstacleInfo(obstacles_);
        planner_->setStartPose(current_pose_);
        planner_->runOptimization();

        std::vector<tools::pathInfo> optimized_traj;
        planner_->getPlannerResults(optimized_traj);
        if (optimized_traj.size() < 2)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "优化结果无效");
            return;
        }

        // 2) 由 TEB 结果换算阿克曼指令（后轮线速度 + 前轮绝对转角）
        const AckermannCommand cmd = controller_.computeCommand(current_pose_, optimized_traj);

        // 3) 下发（实现由使用方提供）
        if (control_api_)
        {
            control_api_->sendCommand(cmd);
        }
        else
        {
            RCLCPP_WARN_ONCE(this->get_logger(),
                             "未注入 AckermannControlInterface，指令只计算不下发"
                             "（见 main() 中的注释示例）");
        }
    }

    // 四元数 -> yaw
    static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(2.0 * (static_cast<double>(q.w) * static_cast<double>(q.z) +
                                 static_cast<double>(q.x) * static_cast<double>(q.y)),
                          1.0 - 2.0 * (static_cast<double>(q.y) * static_cast<double>(q.y) +
                                       static_cast<double>(q.z) * static_cast<double>(q.z)));
    }

private:
    AckermannParams params_;                            // 车身参数
    TebConfig cfg_;                                     // TEB 配置（planner_ 持有其引用，需先于 planner_ 构造）
    AckermannController controller_;                    // 阿克曼控制器
    std::shared_ptr<plannerManager> planner_;           // TEB 优化器
    AckermannControlInterface* control_api_ = nullptr;  // 控制下发接口（使用方注入）

    std::string scan_topic_;
    std::string path_topic_;
    std::string pose_topic_;

    // 雷达处理参数 / 外参
    double scan_angle_limit_ = 1.5707963267948966;   // 正前方裁剪半角 rad（默认 ±90 deg）
    double scan_range_min_   = 0.5;                  // 有效最近距离 m
    double scan_range_max_   = 5.0;                  // 有效最远距离 m
    int    scan_max_points_  = 100;                  // 抽稀后最大障碍点数
    double laser_x_          = 0.0;                  // 雷达在车体坐标系下的位置 m
    double laser_y_          = 0.0;
    double laser_yaw_        = 0.0;                  // 雷达安装偏航角 rad

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<tools::pathInfo> reference_path_;       // 参考路径（局部坐标）
    std::vector<tools::obstacleInfo> obstacles_;        // 最新一帧障碍物点（局部坐标）
    tools::pathInfo current_pose_{};                    // 当前车辆位姿
    bool has_pose_ = false;
};
}  // namespace teb_local_planner


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<teb_local_planner::TebAckermannNode>();

    // -------------------------------------------------------------------------
    // 控制下发由使用方实现：继承 AckermannControlInterface 后注入，例如
    //
    // class MyControlApi : public teb_local_planner::AckermannControlInterface
    // {
    // public:
    //     bool sendCommand(const teb_local_planner::AckermannCommand& cmd) override
    //     {
    //         if (!cmd.valid)
    //         {
    //             // 没有可用轨迹 / 位姿：按你的策略处理（例如停车）
    //             return true;
    //         }
    //         // cmd.rear_wheel_speed : 后轮线速度 m/s（≥0，0 表示停车）
    //         // cmd.steering_angle   : 前轮目标绝对转角 rad（位置控制）
    //         //                        执行器以固定 8 deg/s = 0.13962634015954636 rad/s 转到该角度
    //         // TODO: 在这里下发（CAN / 串口 / 你自己的控制话题 ...）
    //         return true;
    //     }
    // };
    // MyControlApi my_control_api;
    // node->setControlInterface(&my_control_api);
    // -------------------------------------------------------------------------

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

