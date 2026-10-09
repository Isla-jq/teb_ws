#include "teb_ackermann_planner/ackermann_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace teb_local_planner
{
    void AckermannController::reset()
    {
        last_steering_angle_ = 0.0;
        last_closest_idx_    = 0;
    }

    int AckermannController::findClosestIndex(const tools::pathInfo& pose,
                                              const std::vector<tools::pathInfo>& traj) const
    {
        const int n = static_cast<int>(traj.size());
        if (n <= 0)
            return -1;

        // 从上次的索引开始向后搜索（车往前走，最近点不会大幅回退）
        const int start = std::max(0, std::min(last_closest_idx_, n - 1));

        double min_dist_sq = std::numeric_limits<double>::max();
        int    min_idx     = start;
        for (int i = start; i < n; i++)
        {
            const double dx = static_cast<double>(traj[i].x) - static_cast<double>(pose.x);
            const double dy = static_cast<double>(traj[i].y) - static_cast<double>(pose.y);
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                min_idx     = i;
            }
        }
        return min_idx;
    }

    AckermannCommand AckermannController::computeCommand(const tools::pathInfo& current_pose,
                                                         const std::vector<tools::pathInfo>& optimized_traj)
    {
        AckermannCommand cmd;
        cmd.steering_angle   = last_steering_angle_;   // 无有效轨迹时保持上一次转角
        cmd.rear_wheel_speed = 0.0;
        cmd.valid            = false;

        if (optimized_traj.size() < 2)
        {
            // 没有可用轨迹：指令无效，是否停车由使用方的下发实现决定
            return cmd;
        }

        const int closest_idx = findClosestIndex(current_pose, optimized_traj);
        if (closest_idx < 0)
            return cmd;
        last_closest_idx_ = closest_idx;

        // 取最近点所在段：i -> i+1（最近点为末点时退回上一段）
        const int i = std::min(closest_idx, static_cast<int>(optimized_traj.size()) - 2);

        const double dt = static_cast<double>(optimized_traj[i].dt);   // 该段 TEB 时间间隔 s
        const double dx = static_cast<double>(optimized_traj[i + 1].x) - static_cast<double>(optimized_traj[i].x);
        const double dy = static_cast<double>(optimized_traj[i + 1].y) - static_cast<double>(optimized_traj[i].y);
        const double ds = std::hypot(dx, dy);                          // 该段弧长 m
        const double dtheta = tools::normalizeAngle(
            static_cast<double>(optimized_traj[i + 1].theta) - static_cast<double>(optimized_traj[i].theta));

        // 退化段（点重复或时间无效）：保持上次转角，速度为 0
        if (ds < 1e-3 || dt < 1e-6)
            return cmd;

        // ---------------- 后轮线速度 ----------------
        // 段速度 = 弧长 / ΔT，恒为非负（车辆只能前进，不倒车）
        const double v = std::min(params_.max_speed, ds / dt);

        // ---------------- 前轮转角 ----------------
        // 段转弯半径 R = ds / dtheta，阿克曼几何 delta = atan(wheelbase / R)
        double wheel_angle = std::atan(params_.wheelbase * dtheta / ds);
        // 1) 前轮最大转角限幅（机械极限）
        wheel_angle = std::max(-params_.max_steering_angle,
                               std::min(params_.max_steering_angle, wheel_angle));
        // 2) 叠加转角零位偏置，得到下发给执行器的目标角
        double steering = wheel_angle + params_.steering_offset;

        // 3) 指令斜率限制：一个控制周期内转角变化量不超过 steering_rate * control_period
        //    （执行器按位置控制以固定角速度回转，保证指令不超出该回转能力）
        const double max_step = params_.steering_rate * params_.control_period;
        const double delta    = steering - last_steering_angle_;
        if (delta > max_step)
            steering = last_steering_angle_ + max_step;
        if (delta < -max_step)
            steering = last_steering_angle_ - max_step;

        last_steering_angle_ = steering;

        cmd.steering_angle   = steering;
        cmd.rear_wheel_speed = v;
        cmd.valid            = true;
        return cmd;
    }
}  // namespace teb_local_planner
