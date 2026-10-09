/*
 * @Function: Ackermann Controller: optimized trajectory -> (rear wheel speed, steering angle)
 * @Date:2026-09-22
 */
#pragma once

#include <vector>

#include "ackermann_params.h"
#include "tools.h"

namespace teb_local_planner
{
/**
 * 阿克曼控制器（直接取 TEB 结果换算指令）：
 *  1. 在优化轨迹上找离车辆最近的点（记录索引，只向后搜索，避免轨迹自交时跳点）
 *  2. 取该点所在段 i -> i+1 的 TEB 结果：
 *       后轮线速度 v = 段长 / ΔT_i
 *       前轮转角   delta = atan(wheelbase * Δtheta / 段长)   （等价于 atan(wheelbase / R)，R 为段转弯半径）
 *  3. delta 限幅到 ±max_steering_angle，再叠加零位偏置 steering_offset
 *  4. 指令斜率限制：本次指令相对上一次指令的变化量 <= steering_rate * control_period
 *     （执行器按位置控制以固定角速度转动，这里保证指令本身不会超出该转速）
 *
 * 注意：本类只计算指令，不做下发；下发由 AckermannControlInterface 的实现完成。
 */
class AckermannController
{
public:
    explicit AckermannController(const AckermannParams& params)
        : params_(params)
    {
    }

    void setParams(const AckermannParams& params) { params_ = params; }
    const AckermannParams& getParams() const { return params_; }

    // 清空内部状态（换参考路径 / 重新开始跟踪时调用）
    void reset();

    // 计算控制指令
    AckermannCommand computeCommand(const tools::pathInfo& current_pose,
                                    const std::vector<tools::pathInfo>& optimized_traj);

    double getLastSteeringAngle() const { return last_steering_angle_; }

private:
    // 轨迹上距离 pose 最近的点索引
    int findClosestIndex(const tools::pathInfo& pose,
                         const std::vector<tools::pathInfo>& traj) const;

private:
    AckermannParams params_;               // 车身参数
    double last_steering_angle_ = 0.0;     // 上一次下发的转角指令 rad
    int    last_closest_idx_    = 0;       // 上一次的最近点索引
};
}  // namespace teb_local_planner
