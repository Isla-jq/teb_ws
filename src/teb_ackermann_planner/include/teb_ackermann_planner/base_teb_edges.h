/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>


namespace teb_local_planner
{
// -----------------------------------------------------------------------------
// 存储全局配置：权重、目标位置等
// -----------------------------------------------------------------------------

struct TebConfig
{
    // ---------------- 避障（软约束） ----------------
    double min_obstacle_dist = 0.5;   // 期望的障碍最小距离 m
    double penalty_epsilon   = 0.05;  // 软约束边界余量
    double obstacle_weight   = 10.0;  // 避障权重
    double weight_viapoint   = 10.0;  // 跟随参考路径权重

    // ---------------- 阿克曼运动学 ----------------
    double wheelbase          = 0.6;    // 轴距 m
    double min_turning_radius = 1.85;   // 最小转弯半径 m（= 轴距 / tan(前轮最大转角)）
    double weight_kinematics  = 1000.0; // 运动学约束权重

    // ---------------- 速度限幅 ----------------
    double max_vel_x           = 0.3;   // 速度上限 m/s（车辆只前进，不倒车）
    double max_vel_theta       = 1.0;   // 最大角速度 rad/s
    double weight_velocity     = 100.0; // 速度约束权重

    // ---------------- 加速度限幅 ----------------
    double acc_lim_x           = 0.5;   // 线加速度 m/s^2
    double acc_lim_theta       = 1.0;   // 角加速度 rad/s^2
    double weight_acceleration = 10.0;  // 加速度约束权重

    // ---------------- 时间最优 ----------------
    double weight_timeoptimal = 1.0;   // 时间最优权重

    // ---------------- 轨迹时间分辨率 ----------------
    double dt_ref = 0.3;   // ΔT 初始值 / 参考值 s
    double dt_min = 0.05;  // ΔT 下限 s
    double dt_max = 0.8;   // ΔT 上限 s

    // ------------- viapoint 约束起点偏移 -------------
    // 起点附近的顶点与运动学/速度约束相互拉扯，跳过前 k 个顶点不加 viapoint 约束
    int viapoint_start_offset = 3;

    // ---------------- 局部窗口截取 ----------------
    double local_window_length    = 8.0;   // 局部子路径长度 m
    double path_point_spacing     = 0.2;   // 局部窗口重采样点距 m
    int    local_window_max_poses = 60;    // 局部窗口最大顶点数（点距不足时自动放大）
    double slow_down_distance     = 1.5;   // 距全局路径末端多远开始线性减速到 0 m

    // ---------------- 优化器 ----------------
    bool optimization_verbose = false;
    int no_inner_iterations = 5;
    int no_outer_iterations = 4;
};

// 一元边基类（连接1个顶点）
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebUnaryEdge() { this->_vertices[0] = nullptr; }

    // 不手写析构函数：g2o 在 removeEdge()/clear() 时统一解绑并释放边，
    // 若在此处再访问 _vertices[i]（顶点可能已先被释放）会造成 use-after-free

    // 统一误差获取接口：计算误差后返回
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现（满足g2o接口要求，子类可重写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口：设置TebConfig
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针（所有子类边可直接访问）
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐
};

// 2.2 二元边基类（连接2个顶点）
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebBinaryEdge()
    {
        this->_vertices[0] = nullptr;
        this->_vertices[1] = nullptr;
    }

    // 不手写析构函数（理由同 BaseTebUnaryEdge）

    // 统一误差获取接口
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 多元边基类（连接 >= 3 个顶点，顶点数在构造时由 resize() 指定）
// 用于同时关联 [位姿, 位姿, ΔT] 或 [位姿, 位姿, 位姿, ΔT, ΔT] 的约束边
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:
    using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
    using g2o::BaseMultiEdge<D, E>::computeError;

    BaseTebMultiEdge() {}

    // 统一配置传递接口
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

    // 序列化默认实现（本工程不做图文件读写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

protected:
    // 共享配置指针
    const TebConfig* cfg_ = nullptr;
    // 继承 g2o 的误差和顶点成员
    using g2o::BaseMultiEdge<D, E>::_error;
    using g2o::BaseMultiEdge<D, E>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace teb_local_planner
