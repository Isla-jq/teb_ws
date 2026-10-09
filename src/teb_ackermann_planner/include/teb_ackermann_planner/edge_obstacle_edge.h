
/*
 * @Function: Obstacle Contraint Edge Class
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 距离约束边（软约束）：把位姿顶点推离障碍物，保持 >= min_obstacle_dist
namespace teb_local_planner
{
// public BaseTebUnaryEdge<1, tools::obstacleInfo, VertexPoseSE2>
// 第一个参数 1：误差向量维度（障碍距离误差为 1 维）
// 第二个参数 tools::obstacleInfo：边的测量值类型（障碍点）
// 第三个参数 VertexPoseSE2：该边连接一个位姿顶点
class EdgeObstacleConstraint : public BaseTebUnaryEdge<1, tools::obstacleInfo, VertexPoseSE2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算（顶点->障碍物距离小于阈值时为正值）
    virtual void computeError() override
    {
        const VertexPoseSE2* v1 = static_cast<const VertexPoseSE2*>(_vertices[0]);
        const double dist = std::hypot(_measurement.x - v1->x(), _measurement.y - v1->y());
        _error[0] = tools::penaltyBoundFromBelow(dist, cfg_->min_obstacle_dist, cfg_->penalty_epsilon);
    }
};
}  // namespace teb_local_planner
