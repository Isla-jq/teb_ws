/*
 * @Function: Ackermann control command output interface (implemented by user)
 * @Date:2026-09-22
 */
#pragma once

#include "ackermann_params.h"

namespace teb_local_planner
{
/**
 * 控制下发接口：规划器只负责计算指令，不做任何下发。
 * 使用方继承本类，在 sendCommand() 里把指令通过 CAN / 串口 / 自定义控制话题等下发，
 * 然后调用 TebAckermannNode::setControlInterface() 注入。
 */
class AckermannControlInterface
{
public:
    virtual ~AckermannControlInterface() = default;

    // 下发一条阿克曼指令；返回 true 表示下发成功
    virtual bool sendCommand(const AckermannCommand& cmd) = 0;
};
}  // namespace teb_local_planner
