#pragma once 
#include <cmath>

namespace tools
{
    /* 角度归一化到 [-pi, pi] */
    inline double normalizeAngle(const double& angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
    /* 最小距离+安全边界-当前距离（离得远为0，超过安全范围为负数）*/
    {
        if (var >= a+epsilon)
        {
            return 0.;
        }
        else
        {
            return (-var + (a+epsilon));
        }
    }

    /* 把 var 限制在 [-a+eps, a-eps] 内，越界部分线性增长（TEB 原版 penaltyBoundToInterval） */
    inline double penaltyBoundToInterval(const double& var, const double& a, const double& epsilon)
    {
        if (var < -a + epsilon)
        {
            return (-var - (a - epsilon));
        }
        if (var <= a - epsilon)
        {
            return 0.;
        }
        else
        {
            return (var - (a - epsilon));
        }
    }

    /* 把 var 限制在 [a+eps, b-eps] 内，越界部分线性增长（TEB 原版 penaltyBoundToInterval） */
    inline double penaltyBoundToInterval(const double& var, const double& a, const double& b, const double& epsilon)
    {
        if (var < a + epsilon)
        {
            return (-var + (a + epsilon));
        }
        if (var <= b - epsilon)
        {
            return 0.;
        }
        else
        {
            return (var - (b - epsilon));
        }
    }

    struct pathInfo
    {
        float x;
        float y;
        float theta;
        float dt;   // 该点到下一点的期望时间间隔 s（优化输出用；参考路径输入填 0）
    };

    struct obstacleInfo
    {
        float x;
        float y;
        float theta;
    };
}