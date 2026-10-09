/*
 * @Function: Velocity Edge Class
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 速度约束边（软约束）：限制相邻两个位姿之间由 ΔT 决定的速度与角速度
namespace teb_local_planner
{
// public BaseTebMultiEdge<2, double>
// 第一个参数 2：误差向量维度（线速度 1 维 + 角速度 1 维）
// 第二个参数 double：测量值类型（该边无测量值）
// 顶点顺序：[位姿 i, 位姿 i+1, ΔT i]
class EdgeVelocity : public BaseTebMultiEdge<2, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeVelocity()
    {
        this->resize(3);          // 3 个顶点：两个位姿 + 一个 ΔT
        this->setMeasurement(0.);
    }

    // 末端减速系数：1.0 = 不减速；0.0 = 该段必须停住（到达全局路径末端时线性减速）
    void setMaxVelScale(const double scale) { max_vel_scale_ = scale; }

    virtual void computeError() override
    {
        const VertexPoseSE2* conf1 = static_cast<const VertexPoseSE2*>(_vertices[0]);
        const VertexPoseSE2* conf2 = static_cast<const VertexPoseSE2*>(_vertices[1]);
        const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

        const Eigen::Vector2d deltaS = conf2->position() - conf1->position();

        // 线速度：位移 / ΔT（恒为非负，不区分前进/倒车）
        const double vel = deltaS.norm() / deltaT->dt();

        // 角速度：朝向变化 / ΔT
        const double omega = tools::normalizeAngle(conf2->theta() - conf1->theta()) / deltaT->dt();

        _error[0] = tools::penaltyBoundToInterval(vel, 0.0,
                                                  cfg_->max_vel_x * max_vel_scale_, cfg_->penalty_epsilon);
        _error[1] = tools::penaltyBoundToInterval(omega, cfg_->max_vel_theta, cfg_->penalty_epsilon);
    }

private:
    double max_vel_scale_ = 1.0;
};
}  // namespace teb_local_planner
