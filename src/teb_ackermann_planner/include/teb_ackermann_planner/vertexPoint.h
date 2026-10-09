/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include "base_teb_edges.h"
#include "tools.h"
// -----------------------------------------------------------------------------
// 顶点定义：TEB 轨迹由两类顶点构成
//   1) VertexPoseSE2  : 轨迹点位姿 (x, y, theta)，维度 3
//   2) VertexTimeDiff : 相邻两点的时间间隔 ΔT，维度 1（每个间隔一个顶点）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
/**
 * 位姿顶点：g2o::BaseVertex<3, Eigen::Vector3d>
 * 第一个模板参数 3：待优化变量的维度（x, y, theta）
 * 第二个模板参数 Eigen::Vector3d：估计值类型
 */
class VertexPoseSE2 : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置顶点为原点
    virtual void setToOriginImpl() override { _estimate.setZero(); }

    // 顶点更新：x,y 直接叠加增量，theta 叠加后归一化到 [-pi, pi]
    virtual void oplusImpl(const double* update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] = tools::normalizeAngle(_estimate[2] + update[2]);
    }

    inline double x() const { return _estimate[0]; }
    inline double y() const { return _estimate[1]; }
    inline double theta() const { return _estimate[2]; }
    inline Eigen::Vector2d position() const { return Eigen::Vector2d(_estimate[0], _estimate[1]); }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
};

/**
 * 时间间隔顶点：g2o::BaseVertex<1, double>，估计值即 ΔT（秒）
 * 优化方式：ΔT 直接叠加增量，并限制在 [dt_min, dt_max] 内
 */
class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexTimeDiff() { setToOriginImpl(); }

    virtual void setToOriginImpl() override { _estimate = 0.3; }

    virtual void oplusImpl(const double* update) override
    {
        const double dt = _estimate + update[0];
        if (dt < dt_min_)
        {
            _estimate = dt_min_;
        }
        else if (dt > dt_max_)
        {
            _estimate = dt_max_;
        }
        else
        {
            _estimate = dt;
        }
    }

    void setDtBounds(const double dt_min, const double dt_max)
    {
        dt_min_ = dt_min;
        dt_max_ = dt_max;
    }

    inline double dt() const { return _estimate; }

    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }

private:
    double dt_min_ = 0.05;
    double dt_max_ = 0.8;
};
}  // namespace teb_local_planner
