/*
 * @Function: Ackermann vehicle parameters and control command
 * @Date:2026-09-22
 */
#pragma once

// -----------------------------------------------------------------------------
// 阿克曼小车车身参数（参考值，按实际车辆修改）
// 单位约定：距离 m，线速度 m/s，角度 rad，角速度 rad/s
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
struct AckermannParams
{
    double wheelbase          = 0.60;                 // 轴距：后轴中心到前轴中心的距离 m
    double max_speed          = 0.30;                 // 后轮最大线速度 m/s（车辆只前进，不倒车）

    // 前轮转角为位置控制：下发目标角，执行器按固定角速度转到该角度
    double max_steering_angle = 0.3141592653589793;   // 前轮最大转角 18 deg -> rad
    double steering_rate      = 0.13962634015954636;  // 前轮转角回转速率 8 deg/s -> rad/s
    double steering_offset    = 0.0;                  // 前轮转角零位偏置 rad

    double control_period     = 0.10;                 // 控制周期 s（节点定时器周期）
};

// -----------------------------------------------------------------------------
// 下发给执行器的阿克曼控制指令
// -----------------------------------------------------------------------------
struct AckermannCommand
{
    double rear_wheel_speed = 0.0;    // 后轮线速度指令 m/s（≥0，0 表示停车）
    double steering_angle   = 0.0;    // 前轮目标转角指令 rad（位置控制，正为左转）
    bool   valid            = false;  // 指令是否有效：false 表示当前没有可用轨迹/位姿
};
}  // namespace teb_local_planner
