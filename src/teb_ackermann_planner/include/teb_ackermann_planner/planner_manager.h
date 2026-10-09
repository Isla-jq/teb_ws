
/*
 * @Function:Trajectory Optimize Method Manager
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */
#pragma once 
#include <memory>
#include <vector>

#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "edge_obstacle_edge.h"
#include "edge_via_point.h"
#include "edge_kinematics.h"
#include "edge_velocity.h"
#include "edge_acceleration.h"
#include "edge_time_optimal.h"
#include "tools.h"

// -----------------------------------------------------------------------------
// 优化器管理类
// -----------------------------------------------------------------------------
namespace teb_local_planner
{


class plannerManager
{
public:
    // 构造函数：初始化配置、优化器、顶点容器
    plannerManager(const TebConfig& cfg)
        : cfg_(cfg)
    {
        optimizer_ = initOptimizer();
    }

    // 析构函数：图内顶点/边全部由 g2o 释放（G2O_DELETE_IMPLICITLY_OWNED_OBJECTS=1），无需手工 delete
    ~plannerManager() = default;

    // 输入：全局参考路径（局部窗口从中截取）
    void setpathInfo(const std::vector<tools::pathInfo>& path);

    // 输入：障碍点（与参考路径同坐标系）
    void setObstacleInfo(const std::vector<tools::obstacleInfo>& obs);

    // 输入：当前车辆位姿（局部窗口第一个顶点，固定不优化）
    void setStartPose(const tools::pathInfo& pose);

    // 输出：局部优化轨迹（x, y, theta, dt；dt 为该点到下一点的期望时间间隔）
    void getPlannerResults(std::vector<tools::pathInfo>& path);

    // 核心：运行优化（每周期重建整张图）
    void runOptimization();

private:
    // 初始化优化器（注册类型、配置求解器）
    std::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // 注册g2o类型（顶点+边）
    static void registerG2OTypes();

    // 清空图（顶点与边由 g2o 统一释放）
    void clearGraph();

    // 截取局部窗口：以车辆位姿为起点，沿参考路径按弧长截取 local_window_length
    bool buildLocalWindow();

    // 添加顶点（TEB风格：位姿顶点 + 时间间隔顶点）
    void AddVertices();

    // 添加边（TEB风格：new边+setTebConfig+关联顶点）
    void AddEdgesKinematics();

    void AddEdgesVelocity();

    void AddEdgesAcceleration();

    void AddEdgesTimeOptimal();

    void AddObstacleEdges();

    void AddViaPointEdges();

    // 执行优化
    bool optimizeGraph();

private:
    const TebConfig& cfg_;                               // 全局配置（只读）
    std::shared_ptr<g2o::SparseOptimizer> optimizer_;    // 优化器
    std::vector<VertexPoseSE2*> pose_vertices_;          // 位姿顶点（被 optimizer_ 持有）
    std::vector<VertexTimeDiff*> timediff_vertices_;     // 时间间隔顶点（被 optimizer_ 持有）

    std::vector<tools::pathInfo> pathPointArr_;          // 全局参考路径
    std::vector<tools::obstacleInfo> obstaclePointInfo_; // 障碍点

    tools::pathInfo start_pose_;                         // 当前车辆位姿
    bool has_start_pose_ = false;

    std::vector<tools::pathInfo> local_window_;          // 局部窗口参考轨迹（第 0 个点 = 车辆位姿）
    std::vector<double> segment_speed_scale_;            // 每段末端减速系数（1.0 = 不减速）

    std::vector<tools::pathInfo> optimized_traj_;        // 最近一次优化结果
};
}  // namespace teb_local_planner
