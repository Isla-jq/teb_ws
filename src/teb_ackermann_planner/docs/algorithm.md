# teb_ackermann_planner：算法与公式

> 本文只讲**本包代码里的算法与公式**：优化问题怎么定义、每类约束的误差怎么算、
> 局部窗口与控制器怎么推导、公式对应哪个函数。ROS 用法不在此展开。
>
> **符号约定**（全篇统一）：局部窗口有 $n$ 个轨迹点，
> 第 $i$ 个位姿 $\mathbf{s}_i=(x_i,y_i,\theta_i)$（后轴中心，$\theta$ 为车头朝向），
> 第 $i$ 段位移 $\Delta\mathbf{s}_i=\mathbf{s}_{i+1}-\mathbf{s}_i$，段长 $\|\Delta\mathbf{s}_i\|$，
> 第 $i$ 段时长 $\Delta T_i$，朝向差
> $\Delta\theta_i=\operatorname{atan2}\!\big(\sin(\theta_{i+1}-\theta_i),\ \cos(\theta_{i+1}-\theta_i)\big)\in(-\pi,\pi]$。
> 车辆参数：轴距 $L$、前轮最大转角 $\delta_{\max}$、最小转弯半径 $R_{\min}=L/\tan\delta_{\max}$。

## 目录

1. 优化问题的定义（变量、目标函数、罚函数、求解器）
2. 局部窗口构造（`buildLocalWindow`）
3. 顶点与更新规则
4. 各类约束边的误差公式（6 类边）
5. 优化循环与图的生命周期
6. 控制器公式（轨迹 → 执行器指令）
7. 节点层：ROS 消息如何变成公式的输入
8. 参数表与调参方向
9. 数值行为与已知限制（含实测）
10. 公式出处与代码索引

---

## 1. 优化问题的定义

### 1.1 优化变量（g2o 顶点）

$$\Gamma=\{\,\mathbf{s}_1,\dots,\mathbf{s}_n,\ \Delta T_1,\dots,\Delta T_{n-1}\,\}$$

$$\mathbf{s}_i=(x_i,y_i,\theta_i),\qquad
\Delta T_i\in[\Delta T_{\min},\Delta T_{\max}]=[0.05,\ 0.8]\ \mathrm{s}$$

总维度 $3n+(n-1)$。其中：

- $\mathbf{s}_1$ 由车辆当前位姿给出（`setStartPose`），$\mathbf{s}_n$ 取局部窗口末端参考点，
  两者 `setFixed(true)`，不参与优化；
- 其余位姿顶点与全部 $\Delta T_i$ 顶点自由优化（`AddVertices()`）。

| 顶点 | 维度 | 初值 | 边界 | 代码 |
| --- | --- | --- | --- | --- |
| 位姿 $\mathbf{s}_i$ | 3 | 局部窗口参考点 | $\theta$ 每次更新后归一化 | `VertexPoseSE2` |
| 时间 $\Delta T_i$ | 1 | `dt_ref` = 0.3 s | $[0.05,0.8]$ | `VertexTimeDiff` |

### 1.2 目标函数（g2o 边）

$$\min_{\Gamma}\ \sum_{k} w_k\big\|\mathbf{e}_k(\Gamma)\big\|^2$$

代码里权重以**信息矩阵**形式给出：$\Omega_k=w_k I_{d_k}$，即 `setInformation(Identity() * w)`。

| 边（约束） | 维度 $d_k$ | 权重 $w_k$ | 默认值 | 文件 |
| --- | --- | --- | --- | --- |
| 运动学 `EdgeKinematicsCarLike` | 2 | `weight_kinematics` | 1000 | `edge_kinematics.h` |
| 速度 `EdgeVelocity` | 2 | `weight_velocity` | 100 | `edge_velocity.h` |
| 加速度 `EdgeAcceleration` | 2 | `weight_acceleration` | 10 | `edge_acceleration.h` |
| 时间最优 `EdgeTimeOptimal` | 1 | `weight_timeoptimal` | 1 | `edge_time_optimal.h` |
| 避障 `EdgeObstacleConstraint` | 1 | `obstacle_weight` | 10 | `edge_obstacle_edge.h` |
| 路径跟随 `EdgeViaPointConstraint` | 2 | `weight_viapoint` | 10 | `edge_via_point.h` |

权重之比就是"谁让步"的优先级：$1000:100:10:10:10:1$ 意味着
**运动学是准硬约束**（阿克曼车必须满足），其次是速度，再次是加速度/避障/跟随，最后是时间最优。

### 1.3 不等式约束怎么变成残差（罚函数）

车辆约束基本都是不等式（速度不超过 $v_{\max}$、离障碍至少 $d_{\min}$、转弯半径不小于 $R_{\min}$）。
这里不引入乘子、也不设显式硬约束，而是"**越界才收费**"的线性罚残差（`tools.h`），
g2o 内部再对残差取平方，等价于二次罚：

$$P_{\downarrow}(x,a,\epsilon)=
\begin{cases}0, & x\ge a+\epsilon\\ a+\epsilon-x, & x<a+\epsilon\end{cases}$$

$$P_{\updownarrow}(x,a,\epsilon)=
\begin{cases}-x-(a-\epsilon), & x<-a+\epsilon\\ 0, & -a+\epsilon\le x\le a-\epsilon\\ x-(a-\epsilon), & x>a-\epsilon\end{cases}$$

$$P_{\updownarrow}(x,a,b,\epsilon)=
\begin{cases}a+\epsilon-x, & x<a+\epsilon\\ 0, & a+\epsilon\le x\le b-\epsilon\\ x-(b-\epsilon), & x>b-\epsilon\end{cases}$$

分别对应 `tools::penaltyBoundFromBelow(var,a,epsilon)` 与两个 `tools::penaltyBoundToInterval` 重载，
其中 $\epsilon=$ `penalty_epsilon`（默认 0.05）。

三点设计说明：

- $\epsilon$ 是**软边界余量**：$x$ 恰好落在边界上时罚值为 0（合法的初值不会被推走），
  但一旦进入 $[a-\epsilon,\ a]$ 这段"预警带"就开始产生轻微代价，
  优化器于是平滑地减速/让位，而不是等到越过硬边界才突变。
- 罚值是**线性**的，进目标函数被平方 ⇒ $\big[\max(0,\cdot)\big]^2$ 型二次罚：
  梯度随越界量线性增长，比"越界就给大常数"数值上稳定得多。
- 该式连续且几乎处处可导，g2o 基类默认的数值微分雅可比可以直接用（本包没有手写 `linearizeOplus`）。

角度归一化工具函数贯穿几乎所有边：

$$\operatorname{normAng}(\theta)=\operatorname{atan2}\big(\sin\theta,\ \cos\theta\big)\in(-\pi,\pi]$$

### 1.4 求解器与迭代结构

| 项 | 取值 | 代码位置 |
| --- | --- | --- |
| 后端 | `g2o::SparseOptimizer` | `plannerManager::initOptimizer()` |
| 优化算法 | `OptimizationAlgorithmLevenberg`（LM） | 同上 |
| 分块求解器 | `BlockSolverX`（顶点维度不齐 3 / 1，无法用固定维度 Traits） | 同上 |
| 线性求解器 | `LinearSolverDense`（窗口小，直接稠密分解） | 同上 |
| 内部迭代 | `optimize(no_inner_iterations)` = 5 次 LM 迭代 | `optimizeGraph()` |
| 外层重复 | `for (i < no_outer_iterations)` = 4 次；`optimizeGraph()` 返回 false 即提前退出 | `runOptimization()` |

即每个控制周期最多 $5\times 4=20$ 次 LM 迭代（`no_inner_iterations` × `no_outer_iterations`）。
每轮外层都重新 `initializeOptimization()`；因为整张图是**每周期重建**的（见第 5 节），
所以上一周期的位移不是被"接着优化"，而是每一轮都从当轮初值重新出发。

## 2. 局部窗口构造（`plannerManager::buildLocalWindow()`）

**输入**：全局参考路径 $\{\mathbf{p}_j\}_{j=0}^{M-1}$（每个点含 $x,y,\theta$）、
车辆当前位姿 $\mathbf{s}_{veh}$、配置 $L_{win}=$ `local_window_length`（8.0 m）、
$d_p=$ `path_point_spacing`（0.2 m）、$N_{\max}=$ `local_window_max_poses`（60）。

**① 最近点**

$$j^{\star}=\arg\min_{0\le j<M}\ \lVert \mathbf{p}_j-\mathbf{s}_{veh}\rVert^2$$

**② 尾段弧长**（从最近点到全局路径末端，供末端减速用）

$$L_{tail}=\sum_{j=j^{\star}}^{M-2}\lVert\mathbf{p}_{j+1}-\mathbf{p}_j\rVert$$

**③ 自适应点距**

$$\delta=\max\Big(d_p,\ \frac{L_{win}}{N_{\max}-2}\Big)$$

默认参数下 $\delta=\max(0.2,\ 8/58)=0.2\ \mathrm m$，窗口内最多约 $L_{win}/\delta+1\approx 41$ 个点。
这一步保证"把 `path_point_spacing` 调小"不会让顶点数失控：点数一旦要吃满上限，
点距就被自动放大。

**④ 沿弧长等距重采样**

在距最近点弧长 $\delta,2\delta,3\delta,\dots$（不超过 $L_{win}$）处插值出窗口参考点。
设采样弧长 $t$ 落在路径段 $j$ 内，$l_j=\sum_{k<j}\lVert\mathbf{p}_{k+1}-\mathbf{p}_k\rVert$ 为该段起点弧长，
$\lambda=(t-l_j)/\lVert\mathbf{p}_{j+1}-\mathbf{p}_j\rVert$，则

$$x(t)=x_j+\lambda\,(x_{j+1}-x_j),\qquad y(t)=y_j+\lambda\,(y_{j+1}-y_j)$$

$$\theta(t)=\theta_j+\lambda\cdot\operatorname{normAng}(\theta_{j+1}-\theta_j)$$

朝向按**最短角方向**插值；若直接对 $\theta$ 数值插值，跨 $\pm\pi$ 的路径段会得到错误方向。

**⑤ 组装窗口**（$n$ 个位姿顶点）

$$\mathbf{s}_1=\mathbf{s}_{veh},\qquad \mathbf{s}_i=(x(t_{i-1}),\ y(t_{i-1}),\ \theta(t_{i-1})),\quad i=2,\dots,n$$

即窗口第 1 个顶点永远是车当前位姿，其余是重采样参考点。
窗口末端超过 $L_{win}$ 就停止采样；若一个点都没采到（路径太短、或参考点与车辆重合），
本函数返回 false，该周期不优化（`optimized_traj_` 清空）。

**⑥ 末端减速系数**（每段一个标量 $c_i\in[0,1]$）

$$c_i=\min\Big(1,\ \frac{\max\big(0,\ L_{tail}-t_{i+1}\big)}{d_{slow}}\Big),\qquad
d_{slow}=\texttt{slow\_down\_distance}=1.5\ \mathrm m,\quad i=1,\dots,n-1$$

含义：该段末端点距全局路径终点还剩 $L_{tail}-t_{i+1}$，
在最后 1.5 m 内把允许速度线性压到 0，从而实现"到终点速度为零"。
注意减速**不是**加了一条新边，而是**缩放速度边的上下界**（见 4.2）。

---

## 3. 顶点与更新规则

### 3.1 位姿顶点 `VertexPoseSE2`（维度 3）

估计值 $\mathbf{s}=(x,y,\theta)$，更新规则（`oplusImpl`）：

$$\begin{bmatrix}x\\ y\\ \theta\end{bmatrix}
\leftarrow
\begin{bmatrix}x+\delta_x\\ y+\delta_y\\ \operatorname{normAng}\big(\theta+\delta_\theta\big)\end{bmatrix}$$

$\theta$ 每次扰动后折叠回 $(-\pi,\pi]$，避免角度多圈累积导致插值/比较出错。

| 项 | 取值 |
| --- | --- |
| id | $0,\dots,n-1$ |
| 初值 | 局部窗口参考点（`local_window_`） |
| 固定顶点 | $i=1$（车当前位姿）、$i=n$（窗口末端参考点），`setFixed(true)` |

固定两端的作用：起点固定 ⇒ 轨迹必然从车当前位置出发（否则跟随项会把起点拽到参考线上，
造成每周期横向跳变）；末端固定 ⇒ 轨迹必然朝前方参考线延伸，
不会为了缩短时间而把整条轨迹缩成一团。

### 3.2 时间间隔顶点 `VertexTimeDiff`（维度 1）

估计值即 $\Delta T$，初值 `dt_ref` = 0.3 s，更新规则：

$$\Delta T\leftarrow \operatorname{clamp}\big(\Delta T+\delta_T,\ \Delta T_{\min},\ \Delta T_{\max}\big),
\qquad [\Delta T_{\min},\Delta T_{\max}]=[0.05,\ 0.8]\ \mathrm s$$

把夹紧直接写在 `oplusImpl` 里是一种**投影式边界处理**：任何一步更新后 $\Delta T$ 都落在区间内，
于是时间最优边可以放心地把 $\Delta T$ 往小压，不会出现 $\Delta T\to 0$ 让
$v=\lVert\Delta\mathbf{s}\rVert/\Delta T$ 发散的情况。

| 项 | 取值 |
| --- | --- |
| id | $100000+i$（与位姿顶点 id 分段，避免冲突） |
| 数量 | $n-1$（等于段数） |
| 初值 / 边界 | `dt_ref` / `setDtBounds(dt_min, dt_max)` |

## 4. 各类约束边的误差公式

公式里的索引是 1 基（$\mathbf{s}_1\dots\mathbf{s}_n$）；代码里是 0 基，位姿 id 为 $0\dots n-1$，
固定顶点是 id 0 与 id $n-1$。所有边的 $\Omega_k=w_k I_{d_k}$。

| 边 | 顶点 | 条数（默认 $n\approx41,\ m\le100$） |
| --- | --- | --- |
| 运动学 | $\mathbf{s}_i,\mathbf{s}_{i+1}$ | $n-1=40$ |
| 速度 | $\mathbf{s}_i,\mathbf{s}_{i+1},\Delta T_i$ | $n-1=40$ |
| 加速度 | $\mathbf{s}_i,\mathbf{s}_{i+1},\mathbf{s}_{i+2},\Delta T_i,\Delta T_{i+1}$ | $n-2=39$ |
| 时间最优 | $\Delta T_i$ | $n-1=40$ |
| 避障 | $\mathbf{s}_i$ 与每个障碍点 | $(n-2)\cdot m\approx3900$ |
| 路径跟随 | $\mathbf{s}_i$ | $n-1-\text{offset}=37$ |

即默认参数下一张图约 4000 条边，**九成以上是避障一元边** —— 所以 `scan_max_points`
（抽稀后障碍点数上限）是实时性的主要旋钮。

### 4.1 运动学边 `EdgeKinematicsCarLike`（二元，$d=2$）

**(1) 非完整（无侧滑）约束**

$$e_1=\Big\lvert\big(\cos\theta_i+\cos\theta_{i+1}\big)\Delta y_i
-\big(\sin\theta_i+\sin\theta_{i+1}\big)\Delta x_i\Big\rvert$$

推导：设两点朝向的平均为 $\bar\theta$，位移 $\Delta\mathbf{s}_i$ 在车体**横向**上的分量是

$$\Delta_\perp=\Delta x_i\sin\bar\theta-\Delta y_i\cos\bar\theta$$

无侧滑要求 $\Delta_\perp=0$。把 $\sin\theta_i+\sin\theta_{i+1}\approx 2\sin\bar\theta$、
$\cos\theta_i+\cos\theta_{i+1}\approx 2\cos\bar\theta$ 代入即得 $e_1$（与 $\Delta_\perp$ 差常数 2，
被权重吸收）。几何含义：$e_1=0\iff\Delta\mathbf{s}_i$ 平行于车头方向，车不会横向平移。

**(2) 最小转弯半径约束**

$$e_2=
\begin{cases}
0, & \lvert\Delta\theta_i\rvert<10^{-6}\\
P_{\downarrow}\!\left(\dfrac{\lVert\Delta\mathbf{s}_i\rVert}{\lvert\Delta\theta_i\rvert},\ R_{\min},\ 0\right), & \text{其它}
\end{cases}$$

推导：把一段近似成圆弧，则弦长与转角之比就是曲率半径

$$R_i=\frac{\lVert\Delta\mathbf{s}_i\rVert}{\lvert\Delta\theta_i\rvert}$$

阿克曼几何给出 $\tan\delta_i=L/R_i$，前轮机械极限 $\lvert\delta_i\rvert\le\delta_{\max}$ 等价于

$$R_i\ \ge\ R_{\min}=\frac{L}{\tan\delta_{\max}}$$

阈值取 $\epsilon=0$（不给余量），因为 $R_{\min}$ 本身就是硬机械极限；
直线段 $\lvert\Delta\theta_i\rvert\to0$ 时 $R_i\to\infty$ 无约束，代码直接置 0 并避免除零。

> 与 TEB 原版 car-like 的差别：上游把前轮转角及其角速度当作独立约束量来建模；
> 本包不引入转角顶点，而是把机械极限**折算成转弯半径** $R\ge R_{\min}$，
> 顶点更少，且与控制器里 $R=\lVert\Delta s\rVert/\Delta\theta$ 的反解天然一致（见 6.3）。
> 另外 $e_1$ 带绝对值，在 $\Delta_\perp=0$ 处不可导，雅可比由 g2o 基类的数值微分提供
> （本包没有手写 `linearizeOplus`）。

### 4.2 速度边 `EdgeVelocity`（多元 3 顶点，$d=2$）

$$v_i=\frac{\lVert\Delta\mathbf{s}_i\rVert}{\Delta T_i},\qquad
\omega_i=\frac{\Delta\theta_i}{\Delta T_i}$$

$$e_1=P_{\updownarrow}\big(v_i,\ 0,\ c_i\,v_{\max},\ \epsilon\big),\qquad
e_2=P_{\updownarrow}\big(\omega_i,\ \omega_{\max},\ \epsilon\big)$$

- 线速度取位移大小的速率，恒为非负；本车只前进、不倒车，
  所以下界取 $0$（$e_1$ 的可行区间是 $[\epsilon,\ c_i v_{\max}-\epsilon]$）。
- $c_i$ 是第 2 节第 ⑥ 步的末端减速系数：把线速度上界缩放，
  越靠近全局终点允许速度越小，到终点 $c_i\to0$、速度被压到 0。
- $\omega_{\max}=$ `max_vel_theta`，节点里由 $v_{\max}/R_{\min}$ 推出（见 7.5）。
- $c_i$ **只缩放线速度**，没有缩放 $\omega_{\max}$（见第 9 节）。

### 4.3 加速度边 `EdgeAcceleration`（多元 5 顶点，$d=2$）

$$a_i=\frac{2\,\big(v_{i+1}-v_i\big)}{\Delta T_i+\Delta T_{i+1}},\qquad
\alpha_i=\frac{2\,\big(\omega_{i+1}-\omega_i\big)}{\Delta T_i+\Delta T_{i+1}}$$

$$e_1=P_{\updownarrow}\big(a_i,\ a_{\lim},\ \epsilon\big),\qquad
e_2=P_{\updownarrow}\big(\alpha_i,\ \alpha_{\lim},\ \epsilon\big)$$

$v_i,\omega_i$ 的算法与 4.2 完全相同（线速度同样取位移大小的速率，恒为非负）。
系数 $\dfrac{2}{\Delta T_i+\Delta T_{i+1}}$ 来自**中点差分**：速度差发生在两段的中点之间，
时间间隔是两段时长的平均 $\dfrac{\Delta T_i+\Delta T_{i+1}}{2}$，于是 $a_i=\Delta v/\Delta t$ 即上式。

### 4.4 时间最优边 `EdgeTimeOptimal`（一元，$d=1$）

$$e=\Delta T_i,\qquad \Omega=w_t=1.0\quad\Longrightarrow\quad \min\ \sum_i \Delta T_i$$

代价就是总执行时间 $T=\sum_i\Delta T_i$ 的加权和。$\Delta T_i$ 的下界由顶点夹紧在 `dt_min`
提供，所以"压缩时间"最终被 `dt_min` 与速度上限共同限制住。

### 4.5 避障边 `EdgeObstacleConstraint`（一元，$d=1$）

对每个**非固定**位姿顶点 $\mathbf{s}_i$ 与每个障碍点 $\mathbf{o}_k=(o_{kx},o_{ky})$：

$$d_{ik}=\sqrt{(o_{kx}-x_i)^2+(o_{ky}-y_i)^2},\qquad
e=P_{\downarrow}\big(d_{ik},\ d_{\min},\ \epsilon\big)$$

- $d_{\min}=$ `min_obstacle_dist`（0.5 m）、$\epsilon=$ `penalty_epsilon`（0.05 m）；
- 只有 $d_{ik}<d_{\min}+\epsilon$ 时才产生非零罚值与非零梯度，远处障碍不产生任何"拉力"
  （因此单张图上有几千条避障边，但绝大多数当前为零代价）；
- 起点与末端是固定顶点，跳过；
- 障碍点按**质点**处理（无半径、无膨胀）。车体尺寸的全部影响都由 $d_{\min}$ 承担，
  所以 $d_{\min}$ 应 ≥ 车体半宽（含车长方向的保守余量）+ 安全距离。

### 4.6 路径跟随边 `EdgeViaPointConstraint`（一元，$d=2$）

$$e=\begin{bmatrix}x_i^{ref}-x_i\\ y_i^{ref}-y_i\end{bmatrix},\qquad \Omega=w_{via}I_2,\ w_{via}=10$$

用线性弹簧把位姿顶点拉向窗口参考点 $(x_i^{ref},y_i^{ref})$（只约束位置，朝向交给运动学边）。
两条实现细节：

- 跳过固定顶点；
- 跳过前 `viapoint_start_offset`（3）个顶点：起点附近位姿被运动学与速度边强力牵引，
  再叠加跟随项会出现"既被拉回参考线、又被拽向前方"的对抗，表现为起点附近抖动/折角，
  所以前 3 个点不加跟随约束。

---

## 5. 优化循环与图的生命周期

`runOptimization()` 每个控制周期被完整执行一次，顺序固定：

1. `clearGraph()`：`optimizer_->clear()` 释放上一周期全部顶点与边
   （顶点/边由 g2o 持有，`G2O_DELETE_IMPLICITLY_OWNED_OBJECTS=1`，无需手工 delete）；
2. `buildLocalWindow()`：截取窗口（第 2 节）；失败则清空结果直接返回；
3. `AddVertices()`：位姿顶点 $n$ 个 + 时间顶点 $n-1$ 个（含两端固定）；
4. 依次加边：`AddEdgesKinematics` → `AddEdgesVelocity` → `AddEdgesAcceleration` →
   `AddEdgesTimeOptimal` → `AddObstacleEdges` → `AddViaPointEdges`
   （各类 id 分段：位姿顶点 0 起、时间顶点 100000 起、六类边各占 1000000 起的号段，
   保证同一张图内 id 不重复）；
5. 外层重复 `no_outer_iterations` 次 `optimizeGraph()`：
   内部 `initializeOptimization()` + `optimize(no_inner_iterations)`，
   返回值（实际迭代次数）为 0 则跳出；
6. 把顶点值导出到 `optimized_traj_`：
   $(x_i,y_i,\theta_i)$ 与 $dt_i=\Delta T_i$（最后一点 $dt=0$，因为它后面没有段）。

| 阶段 | 复杂度（默认参数） |
| --- | --- |
| 最近点搜索 + 重采样 | $O(M)$（$M$ = 全局路径点数） |
| 建图 | $O(n + n\,m)\approx 4000$ 次 new + `addEdge` |
| 优化 | 20 次 LM 迭代，每次对约 $4n$ 维变量求解稠密线性系统 |

## 6. 控制器公式（`AckermannController::computeCommand()`）

输入：当前位姿 $\mathbf{s}_{veh}$ 与优化轨迹 $\{(\mathbf{s}_i,\Delta T_i)\}_{i=1}^{n}$；
输出：`AckermannCommand{ rear_wheel_speed, steering_angle, valid }`。

### 6.1 最近段定位（利用单调性的搜索）

$$i_{near}=\arg\min_{i\ \ge\ i_{last}}\ \lVert\mathbf{s}_i-\mathbf{s}_{veh}\rVert^2,
\qquad i=\min\big(i_{near},\ n-1\big)$$

搜索从上次结果 `last_closest_idx_` 开始**只向后看**（车向前走，最近点不会大幅回退），
把每周期 $O(n)$ 的搜索摊薄成 $O(n-i_{last})$。取段 $i\to i+1$；若最近点落在末点，则退回上一段。

### 6.2 后轮线速度

$$v=\frac{\lVert\Delta\mathbf{s}_i\rVert}{\Delta T_i},\qquad
v_{cmd}=\operatorname{sat}\big(v,\ \big[0,\ v_{fwd}\big]\big)$$

$v_{fwd}=$ `max_speed`（0.30 m/s）。本车只前进、不倒车，所以线速度指令 $v_{cmd}\ge 0$。
注意这里用的 $\Delta T_i$ 就是优化器求出的**同一个** $\Delta T_i$ ——
优化器负责"算时间"，控制器负责"用时间"，两者共用一套时间变量。

### 6.3 前轮转角（与运动学边互为反解）

由 4.1 的 $R_i=\lVert\Delta\mathbf{s}_i\rVert/\lvert\Delta\theta_i\rvert$ 与阿克曼几何 $\tan\delta=L/R$：

$$\delta_i=\arctan\!\left(\frac{L\,\Delta\theta_i}{\lVert\Delta\mathbf{s}_i\rVert}\right)$$

（$\arctan$ 为奇函数，所以该式自带转向符号；左转 $\Delta\theta_i>0$ 得 $\delta_i>0$。）

这一步是本包"优化—控制一致性"的关键：优化器用 $R\ge R_{\min}$ 约束轨迹，
控制器用同一个 $R$ 反算前轮转角。于是只要 $R\ge R_{\min}$，反算出的 $\lvert\delta_i\rvert\le\delta_{\max}$
就自动成立，**不会出现"优化合格但转角超机械极限"的情况**。

转角的三级处理：

$$\delta_{raw}=\operatorname{sat}\big(\delta_i,\ [-\delta_{\max},\ \delta_{\max}]\big),
\qquad \delta_{target}=\delta_{raw}+\delta_{offset}$$

$$\delta_{cmd}^{(k)}=\delta_{cmd}^{(k-1)}
+\operatorname{sat}\Big(\delta_{target}-\delta_{cmd}^{(k-1)},\
\big[-\dot\delta_{\max}\Delta t_c,\ +\dot\delta_{\max}\Delta t_c\big]\Big)$$

其中 $\delta_{\max}=$ `max_steering_angle`（18°）、$\delta_{offset}=$ `steering_offset`（零位偏置）、
$\dot\delta_{\max}=$ `steering_rate`（8°/s）、$\Delta t_c=$ `control_period`（0.1 s）。
第一级是机械限幅，第二级补零位偏置，第三级是**指令斜率限制**：
执行器的前轮是位置控制、以固定角速度回转，一个周期内最多转
$\dot\delta_{\max}\Delta t_c=0.8^\circ$，所以下发角必须按这个步长渐进逼近目标角。
`last_steering_angle_` 会被 `reset()`（收到新参考路径时）清零。

### 6.4 退化段的处理

$$\lVert\Delta\mathbf{s}_i\rVert<10^{-3}\ \mathrm m\quad\text{或}\quad \Delta T_i<10^{-6}\ \mathrm s
\ \Longrightarrow\ v_{cmd}=0,\ \ \delta_{cmd}=\delta_{cmd}^{(k-1)},\ \ valid=\text{false}$$

即"速度清零、保持上次转角、指令标记为无效"；是否停车由使用方实现的
`AckermannControlInterface::sendCommand()` 决定（本包不做下发实现）。

---

## 7. 节点层：ROS 消息如何变成公式的输入

### 7.1 每个控制周期的数据流

输入话题：`/scan`（`LaserScan`）、`/reference_path`（`Path`）、`/vehicle_pose`（`PoseStamped`），
话题名本身也是参数，可 remap。

`/scan` → 障碍点集合 → `setObstacleInfo()`
`/reference_path` → 参考路径 → `setpathInfo()`
`/vehicle_pose` → 当前位姿 → `setStartPose()`
（以上三者）→ `runOptimization()` → `getPlannerResults()`
→ `controller_.computeCommand()` → `sendCommand()`

### 7.2 障碍点提取（`scanCallback`）

**① 角度裁剪**（只保留车头正前方 $\pm\psi_{lim}$，换算到雷达坐标系要减去安装偏航角 $\psi_l$）

$$a_{start}=\max\big(\alpha_{min},\ -\psi_{lim}-\psi_l\big),\qquad
a_{end}=\min\big(\alpha_{max},\ +\psi_{lim}-\psi_l\big)$$

**② 等间隔抽稀**（保证点数不超过 `scan_max_points`）

$$s=\max\Big(1,\ \frac{i_{end}-i_{start}+1}{N_{max}^{scan}}\Big)$$

**③ 有效量程 = 参数限幅 ∩ 雷达自身量程**

$$r_{min}=\max\big(r_{min}^{param},\ r_{min}^{sensor}\big),\qquad
r_{max}=\min\big(r_{max}^{param},\ r_{max}^{sensor}\big)$$

**④ 坐标变换**（雷达坐标系 → 车体坐标系 → 局部坐标系）

$$\begin{bmatrix}v_x\\ v_y\end{bmatrix}=
\begin{bmatrix}\cos\psi_l & -\sin\psi_l\\ \sin\psi_l & \cos\psi_l\end{bmatrix}
\begin{bmatrix}r\cos\varphi\\ r\sin\varphi\end{bmatrix}
+\begin{bmatrix}l_x\\ l_y\end{bmatrix}$$

$$\begin{bmatrix}o_x\\ o_y\end{bmatrix}=
\begin{bmatrix}x_{veh}\\ y_{veh}\end{bmatrix}+
\begin{bmatrix}\cos\theta_{veh} & -\sin\theta_{veh}\\ \sin\theta_{veh} & \cos\theta_{veh}\end{bmatrix}
\begin{bmatrix}v_x\\ v_y\end{bmatrix}$$

$\varphi$ 为该光束角度、$r$ 为其距离，$(l_x,l_y,\psi_l)$ 是激光外参（**静态参数**，本包不用 tf）。
转换需要当前位姿，因此没有 `has_pose_` 时该帧被丢弃并节流告警。

### 7.3 参考路径与位姿

参考路径：`nav_msgs/Path` 的每个位姿取 $(x,y)$，朝向由四元数解出

$$\theta=\operatorname{atan2}\Big(2(wz+xy),\ 1-2\big(y^2+z^2\big)\Big)$$

收到新路径会调用 `controller_.reset()`（清空上次转角与最近点索引），防止旧状态污染新路径。
位姿：`PoseStamped`（**局部坐标**，GPS→局部坐标的换算由使用方完成，本包不做）。

### 7.4 控制周期

定时器周期 = `control_period`（0.1 s ⇒ 10 Hz）。
每个周期依次：喂输入 → `runOptimization()` → 取轨迹（< 2 点则告警并返回）→ 算指令 → 下发。
`AckermannControlInterface` 未注入时只计算不下发（`RCLCPP_WARN_ONCE` 提示一次）。
所有回调与定时器都在同一个（单线程）执行器线程里，所以包内没有加锁；
若改成多线程执行器，需要自行对 `reference_path_ / obstacles_ / current_pose_` 加保护。

### 7.5 节点里由车身参数派生的量

$$R_{\min}=\frac{L}{\tan\delta_{\max}},\qquad \omega_{\max}=\frac{v_{\max}}{R_{\min}}$$

默认 $L=0.60$ m、$\delta_{\max}=18^\circ$ ⇒ $R_{\min}=1.85$ m；
$v_{\max}=0.30$ m/s ⇒ $\omega_{\max}=0.162$ rad/s。
即节点**始终覆盖** `TebConfig` 里的 `min_turning_radius` 与 `max_vel_theta`
（后者的结构体默认值是 1.0，但从不会被用到），保证车体参数与优化参数只有一个真相来源。
`max_steering_angle_deg`、`steering_rate_deg`、`steering_offset_deg` 三个参数以**度**配置，
节点内部转弧度。

## 8. 参数表与调参方向

### 8.1 优化器参数（`TebConfig`，默认值定义在 `base_teb_edges.h`）

| 参数 | 默认值 | 作用（对应公式） | 调参方向 |
| --- | --- | --- | --- |
| `min_obstacle_dist` | 0.5 m | 避障边下界 $d_{\min}$（4.5） | 调大更保守；但会把轨迹更早推离参考线 |
| `penalty_epsilon` | 0.05 m | 罚函数余量 $\epsilon$（1.3），速度/障碍/角速度共用 | 调大=更早"预警"；过大则约束长期处于轻微越界 |
| `obstacle_weight` | 10 | 避障边权重（4.5） | 调大更远离障碍，但受运动学权重 1000 牵制 |
| `weight_viapoint` | 10 | 跟随参考路径权重（4.6） | 调大更贴参考线，代价是绕障空间变小 |
| `viapoint_start_offset` | 3 | 前 $k$ 个顶点不加跟随（4.6） | 起点有抖动/折角时调大；过大会让起点段完全不受参考线约束 |
| `wheelbase` | 0.6 m | 轴距 $L$（4.1 转弯半径换算） | 与实车一致；节点会用 `AckermannParams` 覆盖 |
| `min_turning_radius` | 1.85 m | $R_{\min}$（4.1） | 节点按 $L/\tan\delta_{\max}$ 覆盖，一般不改 |
| `weight_kinematics` | 1000 | 运动学边权重（4.1） | 默认不动；调大 ⇒ 运动学更硬、跟随更多让步 |
| `max_vel_x` | 0.3 m/s | 速度边速度上限（4.2） | 节点用 `max_speed` 覆盖 |
| `max_vel_theta` | 1.0 rad/s | 角速度上限 $\omega_{\max}$（4.2） | 节点覆盖为 $v_{\max}/R_{\min}=0.162$，结构体默认值实际用不到 |
| `weight_velocity` | 100 | 速度边权重（4.2） | 调大收敛更快，但更容易把跟随项甩开 |
| `acc_lim_x` | 0.5 m/s² | 线加速度上限（4.3） | 按实车加减速能力给 |
| `acc_lim_theta` | 1.0 rad/s² | 角加速度上限（4.3） | 按转向执行器能力给 |
| `weight_acceleration` | 10 | 加速度边权重（4.3） | 轨迹抖动明显时调大 |
| `weight_timeoptimal` | 1.0 | 时间最优边权重（4.4） | 相对量；调大轨迹更快、更贴速度上限 |
| `dt_ref` | 0.3 s | $\Delta T$ 初值（3.2） | 与窗口点距/期望速度同量级 |
| `dt_min` / `dt_max` | 0.05 / 0.8 s | $\Delta T$ 夹紧区间（3.2） | `dt_min` 与速度上限共同决定轨迹能有多快 |
| `local_window_length` | 8.0 m | 窗口长度 $L_{win}$（2） | 调大看得更远，图更大、更慢 |
| `path_point_spacing` | 0.2 m | 窗口点距 $d_p$（2） | 调小会被 `local_window_max_poses` 自动放大 |
| `local_window_max_poses` | 60 | 顶点数上限 $N_{\max}$（2） | 直接决定单帧图规模与耗时 |
| `slow_down_distance` | 1.5 m | 末端减速距离 $d_{slow}$（2⑥、4.2） | 调大提前刹车，调小则末端急减速 |
| `no_inner_iterations` | 5 | 每次 LM 内部迭代数（1.4） | 急弯处转弯半径不达标时首先调这个（见第 9 节） |
| `no_outer_iterations` | 4 | 外层重复次数（1.4） | 同上，调大=更充分但更慢 |
| `optimization_verbose` | false | g2o 迭代日志开关 | 调试时临时打开 |

### 8.2 车身与控制器参数（`AckermannParams`）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `wheelbase` | 0.60 m | 轴距 $L$（4.1 与 6.3 共用的几何参数） |
| `max_speed` | 0.30 m/s | 速度上限（6.2），同时也是 `TebConfig::max_vel_x` |
| `max_steering_angle` | 18° (0.31416 rad) | 前轮机械极限 $\delta_{\max}$（4.1、6.3） |
| `steering_rate` | 8°/s (0.13963 rad/s) | 前轮回转角速度 $\dot\delta_{\max}$（6.3 斜率限制） |
| `steering_offset` | 0° | 前轮零位偏置 $\delta_{offset}$（6.3） |
| `control_period` | 0.10 s | 控制周期 $\Delta t_c$（6.3 斜率限制、7.4 定时器） |

### 8.3 只在节点里暴露的参数（`teb_ackermann_node.cpp`）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `scan_topic` / `reference_path_topic` / `vehicle_pose_topic` | `/scan` / `/reference_path` / `/vehicle_pose` | 三路输入话题名 |
| `scan_angle_limit_deg` | 90° | 正前方保留扇区半角 $\psi_{lim}$（7.2） |
| `scan_range_min` / `scan_range_max` | 0.5 / 5.0 m | 障碍点量程过滤（7.2） |
| `scan_max_points` | 100 | 抽稀后障碍点上限（7.2），直接影响避障边数量 |
| `laser_x` / `laser_y` / `laser_yaw_deg` | 0 / 0 / 0 | 激光外参 $(l_x,l_y,\psi_l)$（7.2），静态参数、不接 tf |

## 9. 数值行为与已知限制（含实测）

### 9.1 权重比决定"谁让步"

$$w_{kin}:w_{vel}:w_{acc}:w_{obs}:w_{via}:w_{time}=1000:100:10:10:10:1$$

遇到障碍或急弯时，"让步"的代价从低到高是：
**时间变长 → 轨迹偏离参考线 → 加速度轻微越界 → 速度轻微越界 → 运动学几乎不让**。
所以急弯处轨迹会切内/让位、绕障时会牺牲一点贴合度，这是权重设计的直接结果，不是 bug。
想改变行为，顺序是：先调权重（1.2），再调迭代预算（1.4）。

### 9.2 迭代预算与"拐角转弯半径不足"（实测）

测量方法：`test/test_teb_planner.cpp` 里进程内直接构造参考路径并调用 `plannerManager`
（合成数据、障碍为空、其余参数默认 $R_{\min}=1.85$ m、$L_{win}=8$ m、点距 0.2 m），
统计所有 $\lvert\Delta\theta\rvert\ge0.02$ 的段的最小等效转弯半径

$$R_{\min}^{obs}=\min_i\ \frac{\lVert\Delta\mathbf{s}_i\rVert}{\lvert\Delta\theta_i\rvert}$$

**① 参考路径为数学直角**（转角处曲率半径理论上为 0）：

| `no_inner_iterations`（`no_outer_iterations`=4） | 总 LM 迭代 | 实测最小等效转弯半径 | 是否达到 $R_{\min}=1.85$ m |
| --- | --- | --- | --- |
| 5（默认） | 20 | 0.528 m | 否 |
| 10 | 40 | 0.803 m | 否 |
| 20 | 80 | 1.680 m | 接近 |
| 50 | 200 | 1.819 m | 基本达标 |

**② 参考路径为有限曲率圆弧**（$R_{ref}=1.0$ m 的 90° 圆弧再接直线）：

| `no_inner_iterations` | 实测最小等效转弯半径 |
| --- | --- |
| 5（默认） | 1.443 m |
| 20 | 1.833 m |

结论：**同一套公式、同一组权重，只改迭代预算，$R$ 就单调从 0.53 m 升到 1.82 m
（圆弧参考下 1.44 m → 1.83 m）⇒ 直角路径下"不满足 $R_{\min}$"是收敛问题
（默认 20 次迭代不够），不是公式或代码错误**：4.1(2) 的罚项本身可以把 $R$ 收敛到 $R_{\min}$。
另外，直角参考路径的数学尖点曲率半径是 0，与车辆 $R_{\min}>0$ 本质矛盾，
优化器只能在"贴参考线"与"满足转弯半径"之间折中 —— 工程上的正解是让全局路径避免数学尖角
（用圆弧化/曲率连续的路径），而不是继续加迭代。

### 9.3 其它已确认的行为与限制

1. **末端减速只缩放线速度**：$c_i$ 作用在 $v_{fwd}$（速度上界）上，不作用在 $\omega_{\max}$
   （4.2）。因此接近终点时仍允许较大的角速度；若希望端点前也收紧转向速率，
   需要把 $c_i$ 同样乘到 $\omega_{\max}$ 上。
2. **障碍是质点、无膨胀**：$d_{\min}$ 必须一次性包含车体半宽（含车长方向的保守余量）与安全距离；
   障碍点还被 `scan_max_points` 抽稀过，分布本身是稀疏的，靠 $d_{\min}$ 保守取值兜底。
3. **每周期全量重建图、无 warm start**：初值恒为"窗口参考点 + 车当前位姿"，
   轨迹的时间连续性靠**窗口滚动**（车前进 ⇒ 窗口整体平移）保证，不靠上一周期的解。
   代价是精度必须靠迭代预算换（9.2）。
4. **所有约束都是软约束**（除两端固定与 $\Delta T$ 夹紧）：权重不足或迭代不足时允许轻微越界，
   这是罚函数式 TEB 的固有特征；`penalty_epsilon` 决定"预警带"宽度。
5. **单线程执行假设**：`plannerManager` 与 `AckermannController` 内部不加锁，
   三路输入回调与定时器都在同一个执行器线程里串行执行（7.4）。
6. **不做下发**：`AckermannControlInterface::sendCommand()` 由使用方实现（6.4、7.4），
   本包只到 `AckermannCommand` 为止。
7. **没有的部分**：tf、costmap、同伦类/拓扑选择、恢复行为、动态障碍预测、多解择优 ——
   本包只求"单条局部最优轨迹"。

### 9.4 测量与验证入口（`test/test_teb_planner.cpp`）

用例在**进程内**直接构造 `plannerManager` 并调用 `runOptimization()`，
不需要 ROS 运行、不需要话题数据。当前 9 个用例全部通过：

| 用例 | 覆盖的公式/行为 |
| --- | --- |
| `StraightPathProducesDrivableTrajectory` | 4.1、4.2 —— 直线参考下轨迹满足转弯半径与速度限制 |
| `StartPoseIsPinnedToVehiclePose` | 3.1 —— 起点顶点被固定在车辆位姿 |
| `ObstacleWallOnPathSideIsCircumvented` | 4.5 —— 侧向障碍墙把轨迹推开 |
| `SharpCornerRespectsMinTurningRadius` | 4.1(2)（判据是 $R\ge 0.8R_{\min}$，见 9.2 的收敛说明） |
| `ShortPathSlowsDownTowardsPathEnd` | 2⑥ + 4.2 —— 末端减速系数生效 |
| `IncompleteInputsProduceEmptyTrajectory` | 5 —— 输入不足时返回空轨迹 |
| `RepeatedOptimizationCyclesStayConsistent` | 5 —— 连续多周期重建图/优化结果一致 |
| `TebPlannerTmp.IterationBudgetEffect` | 9.2 表①的实测（临时测量用例） |
| `TebPlannerTmp.FiniteCurvatureReference` | 9.2 表②的实测（临时测量用例） |

运行：`colcon test --packages-select teb_ackermann_planner`，
或直接跑 `build/teb_ackermann_planner/test_teb_planner`。

---

## 10. 公式出处与代码索引

### 10.1 来源

| 公式 / 结构 | 出处 |
| --- | --- |
| 顶点-边加权图、信息矩阵、LM 求解（1.2、1.4） | g2o：R. Kümmerle et al., "g2o: A General Framework for Graph Optimization", ICRA 2011 |
| 位姿 + $\Delta T$ 顶点、时间最优边（4.4）、罚函数（1.3）、避障/viapoint 一元边（4.5、4.6） | TEB：C. Rösmann et al., ROBOTIK 2012；ECMR 2013（稀疏模型）；RAS 2017（同伦类）；IROS 2017（car-like 车型） |
| 投影型非完整约束形式（4.1(1)） | 同上（TEB 的 car-like 约束形式，见 4.1 的推导） |

### 10.2 本包相对上述来源的自有取舍

以下都是**本工程的实现选择**，不是论文原文：

- 不引入前轮转角顶点，改用 $R\ge R_{\min}$ 的转弯半径形式（4.1(2)）；
- 控制器用同一个 $R$ 反解前轮转角，与优化器共用一套车辆参数（6.3、7.5）；
- 局部窗口的弧长等距重采样 + 自适应点距（2③④）；
- 末端减速通过缩放速度边上下界实现，且只缩放线速度（2⑥、4.2）；
- 前 `viapoint_start_offset` 个顶点不加跟随约束（4.6）；
- 每周期全量重建图、不做 warm start（5）；
- 转角指令的斜率限制（6.3）。

### 10.3 代码索引

| 文件 | 内容 |
| --- | --- |
| `include/teb_ackermann_planner/base_teb_edges.h` | `TebConfig` 全部默认参数、边基类 |
| `include/teb_ackermann_planner/tools.h` | 1.3 罚函数、`normalizeAngle`、`pathInfo`/`obstacleInfo` |
| `include/teb_ackermann_planner/vertexPoint.h` | 3.1 位姿顶点、3.2 时间顶点（`oplusImpl` 的归一化与夹紧） |
| `include/teb_ackermann_planner/edge_kinematics.h` | 4.1 |
| `include/teb_ackermann_planner/edge_velocity.h` | 4.2（含 `setMaxVelScale`） |
| `include/teb_ackermann_planner/edge_acceleration.h` | 4.3 |
| `include/teb_ackermann_planner/edge_time_optimal.h` | 4.4 |
| `include/teb_ackermann_planner/edge_obstacle_edge.h` | 4.5 |
| `include/teb_ackermann_planner/edge_via_point.h` | 4.6 |
| `include/teb_ackermann_planner/planner_manager.h` | 优化器接口（输入/输出/各 Add* / 私有成员） |
| `src/planner_manager.cpp` | 1.4 求解器、2 局部窗口、3 顶点、5 建图与优化循环 |
| `include/teb_ackermann_planner/ackermann_params.h` | 8.2 车身参数、`AckermannCommand` |
| `src/ackermann_controller.cpp` | 6 控制器全部公式 |
| `include/teb_ackermann_planner/ackermann_control_interface.h` | 下发接口（由使用方实现） |
| `src/teb_ackermann_node.cpp` | 7 节点、参数声明、`main()` 里的注入示例 |
| `test/test_teb_planner.cpp` | 9.4 用例 |
