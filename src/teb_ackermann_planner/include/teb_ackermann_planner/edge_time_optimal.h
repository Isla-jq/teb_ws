/*
 * @Function: Time Optimal Edge Class
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 时间最优边：最小化 ΔT 之和，使轨迹尽量快地到达目标
namespace teb_local_planner
{
// public BaseTebUnaryEdge<1, double, VertexTimeDiff>
// 第一个参数 1：误差向量维度（ΔT 本身）
// 第二个参数 double：测量值类型（该边无测量值）
// 第三个参数 VertexTimeDiff：该边连接一个时间间隔顶点
class EdgeTimeOptimal : public BaseTebUnaryEdge<1, double, VertexTimeDiff>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeTimeOptimal() { this->setMeasurement(0.); }

    // 误差即 ΔT：配合权重最小化总时间（ΔT 的下限由顶点自身夹紧在 dt_min）
    virtual void computeError() override
    {
        const VertexTimeDiff* timediff = static_cast<const VertexTimeDiff*>(_vertices[0]);
        _error[0] = timediff->dt();
    }
};
}  // namespace teb_local_planner
