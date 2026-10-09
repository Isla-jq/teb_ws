/*
 * @Function: Acceleration Edge Class
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 加速度约束边（软约束）：限制相邻两段之间的线加速度与角加速度
namespace teb_local_planner
{
// public BaseTebMultiEdge<2, double>
// 第一个参数 2：误差向量维度（线加速度 1 维 + 角加速度 1 维）
// 第二个参数 double：测量值类型（该边无测量值）
// 顶点顺序：[位姿 i, 位姿 i+1, 位姿 i+2, ΔT i, ΔT i+1]
class EdgeAcceleration : public BaseTebMultiEdge<2, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAcceleration()
    {
        this->resize(5);          // 3 个位姿 + 2 个 ΔT
        this->setMeasurement(0.);
    }

    virtual void computeError() override
    {
        const VertexPoseSE2* pose1 = static_cast<const VertexPoseSE2*>(_vertices[0]);
        const VertexPoseSE2* pose2 = static_cast<const VertexPoseSE2*>(_vertices[1]);
        const VertexPoseSE2* pose3 = static_cast<const VertexPoseSE2*>(_vertices[2]);
        const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
        const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

        // ---------------- 线加速度 ----------------
        const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
        const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

        // 线速度：位移 / ΔT（恒为非负，不区分前进/倒车）
        const double vel1 = diff1.norm() / dt1->dt();
        const double vel2 = diff2.norm() / dt2->dt();

        const double acc_lin = (vel2 - vel1) * 2.0 / (dt1->dt() + dt2->dt());
        _error[0] = tools::penaltyBoundToInterval(acc_lin, cfg_->acc_lim_x, cfg_->penalty_epsilon);

        // ---------------- 角加速度 ----------------
        const double angle_diff1 = tools::normalizeAngle(pose2->theta() - pose1->theta());
        const double angle_diff2 = tools::normalizeAngle(pose3->theta() - pose2->theta());
        const double omega1 = angle_diff1 / dt1->dt();
        const double omega2 = angle_diff2 / dt2->dt();

        const double acc_rot = (omega2 - omega1) * 2.0 / (dt1->dt() + dt2->dt());
        _error[1] = tools::penaltyBoundToInterval(acc_rot, cfg_->acc_lim_theta, cfg_->penalty_epsilon);
    }
};
}  // namespace teb_local_planner
