/*
 * @Function: Trajectory Optimize Method Manager (TEB)
 */

#include "teb_ackermann_planner/planner_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

namespace teb_local_planner
{
namespace
{
// 顶点/边的 id 分段，保证同一张 g2o 图内 id 不重复
constexpr int kTimediffIdOffset         = 100000;
constexpr int kEdgeIdOffsetKinematics   = 1000000;
constexpr int kEdgeIdOffsetVelocity     = 2000000;
constexpr int kEdgeIdOffsetAcceleration = 3000000;
constexpr int kEdgeIdOffsetTimeOptimal  = 4000000;
constexpr int kEdgeIdOffsetObstacle     = 5000000;
constexpr int kEdgeIdOffsetViaPoint     = 6000000;
}  // namespace

// ---------------------------------------------------------------------------
// 优化器初始化
// ---------------------------------------------------------------------------
std::shared_ptr<g2o::SparseOptimizer> plannerManager::initOptimizer()
{
    // 线程安全注册自定义类型（C++11 起使用标准库 std::call_once）
    static std::once_flag flag;
    std::call_once(flag, &registerG2OTypes);

    // 顶点维度不统一（位姿 3 维 / ΔT 1 维），使用动态块求解器 BlockSolverX
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    auto optimizer = std::make_shared<g2o::SparseOptimizer>();
    optimizer->setAlgorithm(solver);
    optimizer->setVerbose(cfg_.optimization_verbose);  // 从配置读取verbose

    return optimizer;
}

// 注册g2o类型（顶点+边）
void plannerManager::registerG2OTypes()
{
    g2o::Factory* factory = g2o::Factory::instance();
    factory->registerType("VERTEX_POSE_SE2", std::make_shared<g2o::HyperGraphElementCreator<VertexPoseSE2>>());
    factory->registerType("VERTEX_TIME_DIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
    factory->registerType("EDGE_obstacle_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacleConstraint>>());
    factory->registerType("EDGE_via_point_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPointConstraint>>());
    factory->registerType("EDGE_KINEMATICS_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarLike>>());
    factory->registerType("EDGE_VELOCITY_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
    factory->registerType("EDGE_ACCELERATION_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
    factory->registerType("EDGE_TIME_OPTIMAL_CONSTRAINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
}

// ---------------------------------------------------------------------------
// 输入 / 输出接口
// ---------------------------------------------------------------------------
void plannerManager::setpathInfo(const std::vector<tools::pathInfo>& path)
{
    pathPointArr_ = path;
}

void plannerManager::setObstacleInfo(const std::vector<tools::obstacleInfo>& obs)
{
    obstaclePointInfo_ = obs;
}

void plannerManager::setStartPose(const tools::pathInfo& pose)
{
    start_pose_ = pose;
    has_start_pose_ = true;
}

void plannerManager::getPlannerResults(std::vector<tools::pathInfo>& path)
{
    // 优化结果已在 runOptimization() 中拷贝到 optimized_traj_
    path = optimized_traj_;
}


// ---------------------------------------------------------------------------
// 局部窗口截取
// ---------------------------------------------------------------------------
bool plannerManager::buildLocalWindow()
{
    local_window_.clear();
    segment_speed_scale_.clear();

    if (!has_start_pose_ || pathPointArr_.size() < 2)
    {
        return false;
    }

    // 1) 在全局参考路径上找离车辆最近的路径点
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pathPointArr_.size(); ++i)
    {
        const double dx = pathPointArr_[i].x - start_pose_.x;
        const double dy = pathPointArr_[i].y - start_pose_.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // 2) 最近点 -> 全局路径末端的总弧长（用于末端线性减速）
    double tail_length = 0.0;
    for (size_t i = closest_idx; i + 1 < pathPointArr_.size(); ++i)
    {
        tail_length += std::hypot(pathPointArr_[i + 1].x - pathPointArr_[i].x,
                                  pathPointArr_[i + 1].y - pathPointArr_[i].y);
    }

    // 3) 从最近点起沿参考路径按弧长重采样局部窗口点
    //    点距默认 path_point_spacing；若点数会超过上限则自动放大点距
    const int max_poses = std::max(3, cfg_.local_window_max_poses);
    double spacing = std::max(1.0e-3, cfg_.path_point_spacing);
    spacing = std::max(spacing, cfg_.local_window_length / static_cast<double>(max_poses - 2));

    std::vector<tools::pathInfo> ref_points;   // 窗口参考点（不含车辆位姿）
    std::vector<double> ref_arclen;            // 各参考点相对最近点的弧长

    double passed_len = 0.0;      // 已走过的参考路径弧长
    double next_len = spacing;    // 下一个采样点所在的弧长
    for (size_t i = closest_idx;
         i + 1 < pathPointArr_.size() && passed_len < cfg_.local_window_length; ++i)
    {
        const double x0 = pathPointArr_[i].x;
        const double y0 = pathPointArr_[i].y;
        const double x1 = pathPointArr_[i + 1].x;
        const double y1 = pathPointArr_[i + 1].y;
        const double seg_len = std::hypot(x1 - x0, y1 - y0);
        if (seg_len < 1.0e-6)
        {
            continue;
        }

        while (next_len <= passed_len + seg_len && next_len <= cfg_.local_window_length)
        {
            const double ratio = (next_len - passed_len) / seg_len;
            tools::pathInfo ref;
            ref.x = static_cast<float>(x0 + ratio * (x1 - x0));
            ref.y = static_cast<float>(y0 + ratio * (y1 - y0));
            // 朝向按角度最短方向线性插值
            ref.theta = static_cast<float>(
                pathPointArr_[i].theta +
                ratio * tools::normalizeAngle(pathPointArr_[i + 1].theta - pathPointArr_[i].theta));
            ref.dt = 0.0f;
            ref_points.push_back(ref);
            ref_arclen.push_back(next_len);
            next_len += spacing;
        }
        passed_len += seg_len;
    }

    if (ref_points.empty())
    {
        return false;
    }

    // 4) 组装局部窗口：第 0 个点是车辆当前位姿，其余是重采样参考点
    tools::pathInfo start = start_pose_;
    start.dt = 0.0f;
    local_window_.push_back(start);

    std::vector<double> window_arclen;
    window_arclen.push_back(0.0);
    for (size_t i = 0; i < ref_points.size(); ++i)
    {
        local_window_.push_back(ref_points[i]);
        window_arclen.push_back(ref_arclen[i]);
    }

    // 5) 末端减速系数：距全局路径末端 slow_down_distance 内线性减速到 0
    const double slow_down = std::max(1.0e-3, cfg_.slow_down_distance);
    for (size_t i = 0; i + 1 < local_window_.size(); ++i)
    {
        // 用该段末端点距路径末端的剩余距离
        const double remain = std::max(0.0, tail_length - window_arclen[i + 1]);
        segment_speed_scale_.push_back(std::min(1.0, remain / slow_down));
    }

    return true;
}

// ---------------------------------------------------------------------------
// 建图：顶点
// ---------------------------------------------------------------------------
void plannerManager::AddVertices()
{
    const size_t n = local_window_.size();

    // 位姿顶点：起点 = 车辆当前位姿，末端 = 局部窗口末端参考点，均固定不参与优化
    for (size_t i = 0; i < n; ++i)
    {
        VertexPoseSE2* vertex = new VertexPoseSE2();
        vertex->setId(static_cast<int>(i));
        vertex->setEstimate(Eigen::Vector3d(local_window_[i].x, local_window_[i].y, local_window_[i].theta));
        if (i == 0 || i + 1 == n)
        {
            vertex->setFixed(true);
        }
        optimizer_->addVertex(vertex);
        pose_vertices_.push_back(vertex);
    }

    // 时间间隔顶点：段数 = 位姿数 - 1
    for (size_t i = 0; i + 1 < n; ++i)
    {
        VertexTimeDiff* timediff = new VertexTimeDiff();
        timediff->setId(kTimediffIdOffset + static_cast<int>(i));
        timediff->setDtBounds(cfg_.dt_min, cfg_.dt_max);
        timediff->setEstimate(cfg_.dt_ref);
        optimizer_->addVertex(timediff);
        timediff_vertices_.push_back(timediff);
    }
}

// ---------------------------------------------------------------------------
// 建图：边
// ---------------------------------------------------------------------------
// 车体运动学约束（相邻位姿必须满足阿克曼运动学）
void plannerManager::AddEdgesKinematics()
{
    const int n = static_cast<int>(pose_vertices_.size());
    int edge_id = kEdgeIdOffsetKinematics;
    for (int i = 0; i + 1 < n; ++i)
    {
        EdgeKinematicsCarLike* edge = new EdgeKinematicsCarLike;
        edge->setId(edge_id++);
        edge->setVertex(0, pose_vertices_[i]);
        edge->setVertex(1, pose_vertices_[i + 1]);
        edge->setMeasurement(0.0);
        edge->setInformation(Eigen::Matrix2d::Identity() * cfg_.weight_kinematics);
        edge->setTebConfig(cfg_);
        optimizer_->addEdge(edge);
    }
}

// 速度约束（相邻位姿间距 / ΔT 需满足速度上下限）
void plannerManager::AddEdgesVelocity()
{
    const int n = static_cast<int>(pose_vertices_.size());
    int edge_id = kEdgeIdOffsetVelocity;
    for (int i = 0; i + 1 < n; ++i)
    {
        EdgeVelocity* edge = new EdgeVelocity;
        edge->setId(edge_id++);
        edge->setVertex(0, pose_vertices_[i]);
        edge->setVertex(1, pose_vertices_[i + 1]);
        edge->setVertex(2, timediff_vertices_[i]);
        edge->setMeasurement(0.0);
        // 靠近全局路径末端时按系数线性减速到 0
        const double scale = (i < static_cast<int>(segment_speed_scale_.size()))
                                 ? segment_speed_scale_[i]
                                 : 1.0;
        edge->setMaxVelScale(scale);
        edge->setInformation(Eigen::Matrix2d::Identity() * cfg_.weight_velocity);
        edge->setTebConfig(cfg_);
        optimizer_->addEdge(edge);
    }
}

// 加速度约束（相邻两段速度变化率需满足加速度限制）
void plannerManager::AddEdgesAcceleration()
{
    const int n = static_cast<int>(pose_vertices_.size());
    int edge_id = kEdgeIdOffsetAcceleration;
    for (int i = 0; i + 2 < n; ++i)
    {
        EdgeAcceleration* edge = new EdgeAcceleration;
        edge->setId(edge_id++);
        edge->setVertex(0, pose_vertices_[i]);
        edge->setVertex(1, pose_vertices_[i + 1]);
        edge->setVertex(2, pose_vertices_[i + 2]);
        edge->setVertex(3, timediff_vertices_[i]);
        edge->setVertex(4, timediff_vertices_[i + 1]);
        edge->setMeasurement(0.0);
        edge->setInformation(Eigen::Matrix2d::Identity() * cfg_.weight_acceleration);
        edge->setTebConfig(cfg_);
        optimizer_->addEdge(edge);
    }
}

// 时间最优约束（误差即 ΔT：权重越大越倾向于缩短每段时间，ΔT 下限由顶点夹紧在 dt_min）
void plannerManager::AddEdgesTimeOptimal()
{
    const int n = static_cast<int>(timediff_vertices_.size());
    int edge_id = kEdgeIdOffsetTimeOptimal;
    for (int i = 0; i < n; ++i)
    {
        EdgeTimeOptimal* edge = new EdgeTimeOptimal;
        edge->setId(edge_id++);
        edge->setVertex(0, timediff_vertices_[i]);
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = cfg_.weight_timeoptimal;
        edge->setInformation(information);
        edge->setTebConfig(cfg_);
        optimizer_->addEdge(edge);
    }
}


// 障碍物约束（一元边：每个位姿对应每个障碍点）
void plannerManager::AddObstacleEdges()
{
    const int n = static_cast<int>(pose_vertices_.size());
    int edge_id = kEdgeIdOffsetObstacle;
    for (int i = 0; i < n; ++i)
    {
        // 固定顶点（起点、末端）不需要避障
        if (pose_vertices_[i]->fixed())
        {
            continue;
        }
        for (size_t k = 0; k < obstaclePointInfo_.size(); ++k)
        {
            EdgeObstacleConstraint* edge = new EdgeObstacleConstraint;
            edge->setId(edge_id++);
            edge->setVertex(0, pose_vertices_[i]);
            edge->setMeasurement(obstaclePointInfo_[k]);
            Eigen::Matrix<double, 1, 1> information;
            information(0, 0) = cfg_.obstacle_weight;
            edge->setInformation(information);
            edge->setTebConfig(cfg_);
            optimizer_->addEdge(edge);
        }
    }
}

// 参考路径跟随约束（一元边：位姿贴近局部窗口参考点）
void plannerManager::AddViaPointEdges()
{
    const int n = static_cast<int>(pose_vertices_.size());
    const int offset = std::max(0, cfg_.viapoint_start_offset);
    int edge_id = kEdgeIdOffsetViaPoint;
    for (int i = offset; i < n; ++i)
    {
        // 固定顶点（起点、末端）不需要跟随约束
        if (pose_vertices_[i]->fixed())
        {
            continue;
        }
        EdgeViaPointConstraint* edge = new EdgeViaPointConstraint;
        edge->setId(edge_id++);
        edge->setVertex(0, pose_vertices_[i]);
        edge->setMeasurement(local_window_[i]);
        edge->setInformation(Eigen::Matrix2d::Identity() * cfg_.weight_viapoint);
        edge->setTebConfig(cfg_);
        optimizer_->addEdge(edge);
    }
}

// ---------------------------------------------------------------------------
// 优化
// ---------------------------------------------------------------------------
bool plannerManager::optimizeGraph()
{
    if (!optimizer_ || pose_vertices_.empty())
    {
        return false;
    }

    optimizer_->initializeOptimization();  // 初始化优化，设置各顶点与边
    const int iterations = optimizer_->optimize(cfg_.no_inner_iterations);
    return iterations > 0;
}

// 释放上一周期的图，g2o 会 delete 全部顶点与边
void plannerManager::clearGraph()
{
    if (optimizer_)
    {
        optimizer_->clear();
    }
    pose_vertices_.clear();
    timediff_vertices_.clear();
}

// ---------------------------------------------------------------------------
// 主流程：重建图 -> 优化 -> 取结果
// ---------------------------------------------------------------------------
void plannerManager::runOptimization()
{
    // 1) 清空上一周期的图
    clearGraph();

    // 2) 按车辆当前位置截取局部窗口
    if (!buildLocalWindow())
    {
        optimized_traj_.clear();
        return;
    }

    // 3) 每周期全量重建图
    AddVertices();
    AddEdgesKinematics();
    AddEdgesVelocity();
    AddEdgesAcceleration();
    AddEdgesTimeOptimal();
    AddObstacleEdges();
    AddViaPointEdges();

    // 4) 多次外层迭代优化
    for (int i = 0; i < cfg_.no_outer_iterations; ++i)
    {
        if (!optimizeGraph())
        {
            break;
        }
    }

    // 5) 取优化结果（必须在下一轮 clearGraph() 之前取出）
    optimized_traj_.clear();
    optimized_traj_.reserve(pose_vertices_.size());
    for (size_t i = 0; i < pose_vertices_.size(); ++i)
    {
        tools::pathInfo point;
        point.x = static_cast<float>(pose_vertices_[i]->x());
        point.y = static_cast<float>(pose_vertices_[i]->y());
        point.theta = static_cast<float>(pose_vertices_[i]->theta());
        point.dt = (i < timediff_vertices_.size())
                       ? static_cast<float>(timediff_vertices_[i]->dt())
                       : 0.0f;
        optimized_traj_.push_back(point);
    }
}

}