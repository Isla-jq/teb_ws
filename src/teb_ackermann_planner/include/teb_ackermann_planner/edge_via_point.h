
/*
 * @Function: Follw Path Contraint Edge Class
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */

#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"
#include "tools.h"

// viapoint 约束边（软约束）：把位姿顶点拉向参考路径点
namespace teb_local_planner
{
// public BaseTebUnaryEdge<2, tools::pathInfo, VertexPoseSE2>
// 第一个参数 2：误差向量维度（x, y 两个方向）
// 第二个参数 tools::pathInfo：边的测量值类型（参考路径点）
// 第三个参数 VertexPoseSE2：该边连接一个位姿顶点
class EdgeViaPointConstraint : public BaseTebUnaryEdge<2, tools::pathInfo, VertexPoseSE2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算（使用基类的cfg_获取配置）
    virtual void computeError() override
    {
        const VertexPoseSE2* v1 = static_cast<const VertexPoseSE2*>(_vertices[0]);
        _error[0] = _measurement.x - v1->x();
        _error[1] = _measurement.y - v1->y();
    }
};
}  // namespace teb_local_planner
