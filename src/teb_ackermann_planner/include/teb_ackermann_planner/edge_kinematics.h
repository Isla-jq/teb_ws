/*
 * @Function: Ackermann Kinematics Edge Class
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 阿克曼运动学约束边：相邻两个位姿必须满足汽车的运动学约束
namespace teb_local_planner
{
// public BaseTebBinaryEdge<2, double, VertexPoseSE2, VertexPoseSE2>
// 第一个参数 2：误差向量维度（1 非完整约束偏差 + 1 最小转弯半径偏差）
// 第二个参数 double：测量值类型（该边无测量值）
// 后两个参数：边连接的两个位姿顶点
class EdgeKinematicsCarLike : public BaseTebBinaryEdge<2, double, VertexPoseSE2, VertexPoseSE2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算
    virtual void computeError() override
    {
        const VertexPoseSE2* conf1 = static_cast<const VertexPoseSE2*>(_vertices[0]);
        const VertexPoseSE2* conf2 = static_cast<const VertexPoseSE2*>(_vertices[1]);

        const Eigen::Vector2d deltaS = conf2->position() - conf1->position();

        // 1) 非完整约束：相邻位姿连线必须沿车头方向（前后轮都在车体纵轴上，不能横向平移）
        //    与 TEB 原版保持一致的近似形式：用两端朝向的 cos/sin 之和投影横向位移
        _error[0] = std::fabs((std::cos(conf1->theta()) + std::cos(conf2->theta())) * deltaS[1] -
                              (std::sin(conf1->theta()) + std::sin(conf2->theta())) * deltaS[0]);

        // 2) 最小转弯半径约束：弧长 / 转角 = 转弯半径，不得小于机械极限
        const double angle_diff = tools::normalizeAngle(conf2->theta() - conf1->theta());
        if (std::fabs(angle_diff) < 1.0e-6)
        {
            // 直线段无半径约束（同时避免除零）
            _error[1] = 0.0;
        }
        else
        {
            _error[1] = tools::penaltyBoundFromBelow(deltaS.norm() / std::fabs(angle_diff),
                                                     cfg_->min_turning_radius, 0.0);
        }
    }
};
}  // namespace teb_local_planner
