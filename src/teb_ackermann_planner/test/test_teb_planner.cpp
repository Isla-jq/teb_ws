/*
 * @Function: TEB 核心单元测试 —— plannerManager 建图 / 优化行为验证
 * @Date:2026-10-08
 *
 * 说明：全部用例在进程内直接构造合成参考路径与障碍点，直接调用 plannerManager 接口，
 *       不依赖任何 ROS 话题、外参或外部数据；只使用 gtest + g2o/Eigen（无 rclcpp）。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "teb_ackermann_planner/planner_manager.h"

namespace teb_local_planner
{
namespace
{
constexpr double kPi     = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

// ---------------------------------------------------------------------------
// 合成输入
// ---------------------------------------------------------------------------
// 直线参考路径：(0,0) 沿 +x 到 (length,0)，点距 step，朝向恒为 0
std::vector<tools::pathInfo> makeStraightPath(const double length, const double step)
{
    std::vector<tools::pathInfo> path;
    const int count = static_cast<int>(length / step) + 1;
    path.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        tools::pathInfo p;
        p.x     = static_cast<float>(i * step);
        p.y     = 0.0f;
        p.theta = 0.0f;
        p.dt    = 0.0f;
        path.push_back(p);
    }
    return path;
}

// 直角参考路径：先沿 +x 到 (leg,0)（朝向 0），拐角后沿 +y 到 (leg,leg)（朝向 pi/2）
std::vector<tools::pathInfo> makeRightAnglePath(const double leg, const double step)
{
    std::vector<tools::pathInfo> path;
    auto push = [&path](const double x, const double y, const double theta) {
        tools::pathInfo p;
        p.x     = static_cast<float>(x);
        p.y     = static_cast<float>(y);
        p.theta = static_cast<float>(theta);
        p.dt    = 0.0f;
        path.push_back(p);
    };

    const int count = static_cast<int>(leg / step);
    for (int i = 0; i <= count; ++i)
    {
        push(i * step, 0.0, 0.0);   // 第一段（含拐角点，朝向仍为 0）
    }
    for (int i = 1; i <= count; ++i)
    {
        push(leg, i * step, kHalfPi);   // 第二段
    }
    return path;
}

// 单个障碍点
tools::obstacleInfo makeObstacle(const double x, const double y)
{
    tools::obstacleInfo o;
    o.x     = static_cast<float>(x);
    o.y     = static_cast<float>(y);
    o.theta = 0.0f;
    return o;
}

// 障碍墙：x ∈ [x_begin, x_end] 等间隔排点，y 固定（对应参考线一侧的墙面/栅栏）
std::vector<tools::obstacleInfo> makeObstacleWall(const double x_begin, const double x_end,
                                                  const double y, const double step)
{
    std::vector<tools::obstacleInfo> wall;
    const int count = static_cast<int>((x_end - x_begin) / step + 0.5);
    for (int i = 0; i <= count; ++i)
    {
        wall.push_back(makeObstacle(x_begin + i * step, y));
    }
    return wall;
}

tools::pathInfo makeStartPose(const double x, const double y, const double theta)
{
    tools::pathInfo p;
    p.x     = static_cast<float>(x);
    p.y     = static_cast<float>(y);
    p.theta = static_cast<float>(theta);
    p.dt    = 0.0f;
    return p;
}

// ---------------------------------------------------------------------------
// 几何量测
// ---------------------------------------------------------------------------
// 轨迹上所有点到最近障碍点的最小距离
double minDistanceToObstacles(const std::vector<tools::pathInfo>& traj,
                              const std::vector<tools::obstacleInfo>& obs)
{
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& t : traj)
    {
        for (const auto& o : obs)
        {
            min_dist = std::min(min_dist, std::hypot(static_cast<double>(t.x) - o.x,
                                                     static_cast<double>(t.y) - o.y));
        }
    }
    return min_dist;
}

// 段转角（归一化到 [-pi, pi]）
double segmentAngleDiff(const tools::pathInfo& a, const tools::pathInfo& b)
{
    return tools::normalizeAngle(static_cast<double>(b.theta) - a.theta);
}

// 段长
double segmentLength(const tools::pathInfo& a, const tools::pathInfo& b)
{
    return std::hypot(static_cast<double>(b.x) - a.x, static_cast<double>(b.y) - a.y);
}

// 段速度 = 段长 / ΔT
double segmentSpeed(const tools::pathInfo& a, const tools::pathInfo& b)
{
    return (a.dt > 1.0e-6) ? segmentLength(a, b) / a.dt : 0.0;
}

// ---------------------------------------------------------------------------
// 一个完整规划周期：喂入参考路径 / 障碍点 / 车辆位姿，取回局部优化轨迹
// ---------------------------------------------------------------------------
std::vector<tools::pathInfo> runOnce(plannerManager& planner,
                                     const std::vector<tools::pathInfo>& path,
                                     const std::vector<tools::obstacleInfo>& obstacles,
                                     const tools::pathInfo& start)
{
    planner.setpathInfo(path);
    planner.setObstacleInfo(obstacles);
    planner.setStartPose(start);
    planner.runOptimization();

    std::vector<tools::pathInfo> traj;
    planner.getPlannerResults(traj);
    return traj;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1) 直线参考路径（无障碍）：窗口结构、端点固定、ΔT 受速度约束、运动学不越界
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, StraightPathProducesDrivableTrajectory)
{
    TebConfig cfg;                             // 全部使用默认参数
    plannerManager planner(cfg);               // cfg 必须先于 planner 声明（内部保存其引用）

    const auto path = makeStraightPath(20.0, 0.2);
    const auto traj = runOnce(planner, path, {}, makeStartPose(0.0, 0.0, 0.0));

    // 局部窗口 = 8 m 弧长 / 0.2 m 点距 + 起点顶点
    ASSERT_GE(traj.size(), 10u);
    EXPECT_NEAR(traj.size(), 41u, 2u);          // 1 + 40

    // 起点顶点固定 = 车辆当前位姿（第 0 个点不参与优化）
    EXPECT_NEAR(traj.front().x, 0.0, 1.0e-4);
    EXPECT_NEAR(traj.front().y, 0.0, 1.0e-4);
    EXPECT_NEAR(traj.front().theta, 0.0, 1.0e-4);

    // 末端顶点固定 = 窗口末端参考点（重采样按点距推进，受浮点累加影响最后
    // 一个采样点可能落在 7.8 而非 8.0，允许一个点距的误差）
    EXPECT_GE(traj.back().x, cfg.local_window_length - cfg.path_point_spacing - 0.05);
    EXPECT_LE(traj.back().x, cfg.local_window_length + 0.05);
    EXPECT_NEAR(traj.back().y, 0.0, 1.0e-3);
    EXPECT_FLOAT_EQ(traj.back().dt, 0.0f);      // 最后一个点没有 ΔT 顶点

    // 直线参考 + 无障碍：轨迹不得偏离参考线
    double max_abs_y = 0.0;
    for (const auto& t : traj)
    {
        max_abs_y = std::max(max_abs_y, std::fabs(static_cast<double>(t.y)));
    }
    EXPECT_LT(max_abs_y, 0.02);

    // ΔT 必须落在 [dt_min, dt_max]（顶点夹紧），且速度上限约束真的起作用：
    // 点距 0.2 m、max_vel_x = 0.3 m/s -> 段速度被限制在 0.25 m/s 附近
    // => ΔT 应远大于 dt_min（若速度边未生效，ΔT 会塌到 dt_min = 0.05）
    double dt_sum = 0.0;
    int dt_count  = 0;
    for (size_t i = 0; i + 1 < traj.size(); ++i)
    {
        EXPECT_GE(traj[i].dt, cfg.dt_min - 1.0e-6);
        EXPECT_LE(traj[i].dt, cfg.dt_max + 1.0e-6);
        dt_sum += traj[i].dt;
        ++dt_count;
    }
    ASSERT_GT(dt_count, 0);
    const double mean_dt = dt_sum / dt_count;
    EXPECT_GT(mean_dt, 0.4);
    EXPECT_LT(mean_dt, cfg.dt_max);

    // 运动学：每段等效转弯半径不得小于最小转弯半径（软约束，留 20% 余量）
    for (size_t i = 0; i + 1 < traj.size(); ++i)
    {
        const double dtheta = std::fabs(segmentAngleDiff(traj[i], traj[i + 1]));
        if (dtheta < 1.0e-3)
        {
            continue;
        }
        EXPECT_GE(segmentLength(traj[i], traj[i + 1]) / dtheta, 0.8 * cfg.min_turning_radius);
    }
}

// ---------------------------------------------------------------------------
// 2) 车辆不在参考路径上：起点仍严格固定在车辆位姿，窗口按最近路径点截取
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, StartPoseIsPinnedToVehiclePose)
{
    TebConfig cfg;
    plannerManager planner(cfg);

    const auto path = makeStraightPath(20.0, 0.2);
    // 车辆位姿 (0.35, 0.25, 0.15)：最近路径点为 (0.4, 0)
    const auto traj = runOnce(planner, path, {}, makeStartPose(0.35, 0.25, 0.15));

    ASSERT_GE(traj.size(), 10u);
    EXPECT_NEAR(traj.front().x, 0.35, 1.0e-4);
    EXPECT_NEAR(traj.front().y, 0.25, 1.0e-4);
    EXPECT_NEAR(traj.front().theta, 0.15, 1.0e-4);

    // 窗口末端 = 最近路径点 (0.4,0) 起约 8 m 弧长处的参考点 -> x ≈ 8.4（允许一个点距误差）
    EXPECT_GE(traj.back().x, 8.4 - cfg.path_point_spacing - 0.05);
    EXPECT_LE(traj.back().x, 8.4 + 0.05);
    EXPECT_NEAR(traj.back().y, 0.0, 1.0e-3);
}

// ---------------------------------------------------------------------------
// 3) 参考线一侧的障碍墙：轨迹必须横向让开（验证避障边与权重生效）
//    墙面在参考线右侧 y = +0.35 m，初值到障碍的最小距离只有 0.35 m，
//    低于 min_obstacle_dist(0.5 m)；墙沿路径方向有宽度，纵向挪点无法脱困，
//    只能向 -y 一侧绕行
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, ObstacleWallOnPathSideIsCircumvented)
{
    TebConfig cfg;
    // 用接近上游 TEB 的权重配比（避障 50 / 跟随 1）使避障响应可观测：
    // 默认 weight_viapoint = 10 与 obstacle_weight = 10 相当时，横向让位位移会小很多
    cfg.obstacle_weight = 50.0;
    cfg.weight_viapoint = 1.0;
    plannerManager planner(cfg);

    const auto path      = makeStraightPath(20.0, 0.2);
    const auto obstacles = makeObstacleWall(2.6, 3.4, 0.35, 0.1);
    const auto traj      = runOnce(planner, path, obstacles, makeStartPose(0.0, 0.0, 0.0));

    ASSERT_GE(traj.size(), 10u);

    // 初值到障碍墙的最小距离 = 0.35 m；优化后必须满足 min_obstacle_dist
    const double min_dist = minDistanceToObstacles(traj, obstacles);
    EXPECT_GT(min_dist, 0.45);

    // 让位方向必须是“远离障碍墙”：障碍在 +y 一侧 -> 轨迹向 -y 偏
    double min_y = 0.0;
    for (size_t i = 1; i + 1 < traj.size(); ++i)
    {
        min_y = std::min(min_y, static_cast<double>(traj[i].y));
    }
    EXPECT_LT(min_y, -0.05);

    // 绕行后仍须回到窗口末端参考点（末端固定）
    EXPECT_NEAR(traj.back().y, 0.0, 1.0e-3);
    EXPECT_GE(traj.back().x, cfg.local_window_length - cfg.path_point_spacing - 0.05);
    EXPECT_LE(traj.back().x, cfg.local_window_length + 0.05);
}

// ---------------------------------------------------------------------------
// 4) 90 度直角参考路径：拐角处必须按最小转弯半径“切弯”，不得原地打死
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, SharpCornerRespectsMinTurningRadius)
{
    TebConfig cfg;
    cfg.no_inner_iterations = 50;   // 临时实验：迭代预算对拐角平滑的影响
    cfg.no_outer_iterations = 10;
    plannerManager planner(cfg);

    const auto path = makeRightAnglePath(5.0, 0.2);          // (0,0)->(5,0)->(5,5)
    const auto traj = runOnce(planner, path, {}, makeStartPose(0.0, 0.0, 0.0));

    ASSERT_GE(traj.size(), 10u);

    // 窗口末端 ≈ (0,0) 起弧长 8 m 处：第一段 5 m + 第二段约 3 m -> (5, 3)，朝向 pi/2
    EXPECT_NEAR(traj.back().x, 5.0, 1.0e-3);
    EXPECT_GE(traj.back().y, 3.0 - cfg.path_point_spacing - 0.05);
    EXPECT_LE(traj.back().y, 3.0 + 0.05);
    EXPECT_NEAR(std::fabs(traj.back().theta), kHalfPi, 0.05);

    // 直角参考的等效半径接近 0，优化结果必须满足最小转弯半径（软约束，留 20% 余量）
    bool has_turn_segment = false;
    for (size_t i = 0; i + 1 < traj.size(); ++i)
    {
        const double dtheta = std::fabs(segmentAngleDiff(traj[i], traj[i + 1]));
        if (dtheta < 0.02)
        {
            continue;   // 近似直线段
        }
        has_turn_segment = true;
        EXPECT_GE(segmentLength(traj[i], traj[i + 1]) / dtheta, 0.8 * cfg.min_turning_radius)
            << "段 " << i << " 的等效转弯半径过小";
    }
    EXPECT_TRUE(has_turn_segment);      // 轨迹确实在拐弯
}

// ---------------------------------------------------------------------------
// 5) 参考路径比局部窗口短：窗口覆盖到路径末端，且末端进入减速区（速度下降）
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, ShortPathSlowsDownTowardsPathEnd)
{
    TebConfig cfg;
    plannerManager planner(cfg);

    const auto path = makeStraightPath(4.0, 0.2);            // 4 m < local_window_length
    const auto traj = runOnce(planner, path, {}, makeStartPose(0.0, 0.0, 0.0));

    ASSERT_GE(traj.size(), 10u);

    // 窗口终点落在全局路径终点附近（允许一个点距误差）
    EXPECT_GE(traj.back().x, 4.0 - cfg.path_point_spacing - 0.05);
    EXPECT_LE(traj.back().x, 4.0 + 0.05);
    EXPECT_NEAR(traj.back().y, 0.0, 1.0e-3);

    // 远端的段速度应明显低于起点附近的段速度
    const size_t n    = traj.size();
    const double v_mid  = segmentSpeed(traj[0], traj[1]);
    const double v_last = segmentSpeed(traj[n - 2], traj[n - 1]);
    EXPECT_GT(v_mid, 0.1);
    EXPECT_LT(v_last, v_mid);

    // ΔT 随减速区增大（上限 dt_max）
    EXPECT_GE(traj[n - 2].dt, traj[0].dt);
    EXPECT_LE(traj[n - 2].dt, cfg.dt_max + 1.0e-6);
}

// ---------------------------------------------------------------------------
// 6) 输入不完整（无路径 / 无位姿 / 只有一个路径点）：输出空轨迹且不崩溃
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, IncompleteInputsProduceEmptyTrajectory)
{
    TebConfig cfg;
    plannerManager planner(cfg);

    std::vector<tools::pathInfo> traj;

    // 6.1 什么都没给
    planner.runOptimization();
    planner.getPlannerResults(traj);
    EXPECT_TRUE(traj.empty());

    // 6.2 只给单个路径点（不足两个点无法计算弧长）
    planner.setpathInfo(makeStraightPath(0.0, 0.2));
    planner.setObstacleInfo({});
    planner.setStartPose(makeStartPose(0.0, 0.0, 0.0));
    planner.runOptimization();
    planner.getPlannerResults(traj);
    EXPECT_TRUE(traj.empty());
}

// ---------------------------------------------------------------------------
// 7) 模拟控制循环：连续多个周期重建全图（顶点/边反复 new + clear），结果始终可用
// ---------------------------------------------------------------------------
TEST(TebPlannerTest, RepeatedOptimizationCyclesStayConsistent)
{
    TebConfig cfg;
    plannerManager planner(cfg);

    const auto path = makeStraightPath(20.0, 0.2);
    const std::vector<tools::obstacleInfo> obstacles{makeObstacle(6.0, 0.6)};   // 路径侧前方障碍

    for (int cycle = 0; cycle < 20; ++cycle)
    {
        const double x = 0.8 * cycle;                        // 车辆每周期前进 0.8 m
        const auto traj = runOnce(planner, path, obstacles, makeStartPose(x, 0.0, 0.0));

        ASSERT_GE(traj.size(), 5u) << "第 " << cycle << " 个周期未产出轨迹";
        EXPECT_NEAR(traj.front().x, x, 1.0e-4) << "第 " << cycle << " 个周期起点未固定在车辆位姿";
        EXPECT_NEAR(traj.front().y, 0.0, 1.0e-4);

        // 直线上不应明显跑偏（障碍在 y=+0.6 一侧，靠得较远）
        double max_abs_y = 0.0;
        for (const auto& t : traj)
        {
            max_abs_y = std::max(max_abs_y, std::fabs(static_cast<double>(t.y)));
        }
        EXPECT_LT(max_abs_y, 0.5);
    }
}
// ---------------------------------------------------------------------------
// 临时实验（测量用，跑完删除）：迭代预算对“拐角最小等效转弯半径”的影响
// ---------------------------------------------------------------------------
TEST(TebPlannerTmp, IterationBudgetEffect)
{
    const auto path = makeRightAnglePath(5.0, 0.2);
    for (const int inner : {5, 10, 20, 50})
    {
        TebConfig cfg;
        cfg.no_inner_iterations = inner;
        cfg.no_outer_iterations = 4;
        plannerManager planner(cfg);
        const auto traj = runOnce(planner, path, {}, makeStartPose(0.0, 0.0, 0.0));

        double min_radius = std::numeric_limits<double>::max();
        double min_dtheta = 0.0;
        for (size_t i = 0; i + 1 < traj.size(); ++i)
        {
            const double dtheta = std::fabs(segmentAngleDiff(traj[i], traj[i + 1]));
            if (dtheta < 0.02)
            {
                continue;
            }
            const double radius = segmentLength(traj[i], traj[i + 1]) / dtheta;
            if (radius < min_radius)
            {
                min_radius = radius;
                min_dtheta = dtheta;
            }
        }
        std::cout << "[budget] inner=" << inner << " outer=4"
                  << " 最小等效转弯半径=" << min_radius
                  << " (该段 Δθ=" << min_dtheta << ")" << std::endl;
    }
}

// 临时实验：参考路径为半径有限的圆弧（R_ref = 1.0 m < R_min = 1.85 m）时的收敛情况
TEST(TebPlannerTmp, FiniteCurvatureReference)
{
    const double ref_radius = 1.0;
    const double step       = 0.2;
    std::vector<tools::pathInfo> path;
    // 90 度圆弧：圆心 (0, R)，起点 (0,0) 朝向 0，终点 (R,R) 朝向 pi/2
    const int arc_count = static_cast<int>((kHalfPi * ref_radius) / step + 0.5);
    for (int i = 0; i <= arc_count; ++i)
    {
        const double phi = -kHalfPi + i * step / ref_radius;
        tools::pathInfo p;
        p.x     = static_cast<float>(ref_radius * (1.0 + std::sin(phi)));
        p.y     = static_cast<float>(ref_radius * (1.0 - std::cos(phi)));
        p.theta = static_cast<float>(phi + kHalfPi);
        p.dt    = 0.0f;
        path.push_back(p);
    }
    // 圆弧后接直线段
    for (int i = 1; i <= 25; ++i)
    {
        tools::pathInfo p;
        p.x     = static_cast<float>(ref_radius);
        p.y     = static_cast<float>(ref_radius + i * step);
        p.theta = static_cast<float>(kHalfPi);
        p.dt    = 0.0f;
        path.push_back(p);
    }

    for (const int inner : {5, 20})
    {
        TebConfig cfg;
        cfg.no_inner_iterations = inner;
        cfg.no_outer_iterations = 4;
        plannerManager planner(cfg);
        const auto traj = runOnce(planner, path, {}, makeStartPose(0.0, 0.0, 0.0));

        double min_radius = std::numeric_limits<double>::max();
        for (size_t i = 0; i + 1 < traj.size(); ++i)
        {
            const double dtheta = std::fabs(segmentAngleDiff(traj[i], traj[i + 1]));
            if (dtheta < 0.02)
            {
                continue;
            }
            min_radius = std::min(min_radius, segmentLength(traj[i], traj[i + 1]) / dtheta);
        }
        std::cout << "[arc R=1.0] inner=" << inner
                  << " 最小等效转弯半径=" << min_radius
                  << " 末端=" << traj.back().x << "," << traj.back().y
                  << " theta=" << traj.back().theta << std::endl;
    }
}
}  // namespace teb_local_planner
