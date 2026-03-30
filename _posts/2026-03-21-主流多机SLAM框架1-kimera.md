---
layout: post
title: 主流多机SLAM框架1——Kimera-Multi
date: 2026-03-21 14:24:00
description: 【论文阅读】深入理解Kimera-Multi
tags: 
categories: SLAM Distributed-Optimization Paper-Reading
chart:
  plotly: true
---

Kimera-Multi是MIT SPARK实验室提出的一种多机协同SLAM算法，共有[ICRA](https://ieeexplore.ieee.org/abstract/document/9561090?casa_token=RNUxNIr638MAAAAA:E5EbWTn2-9VzpIeRaxqsDYlHpFjnrddeZs-0OCmR6Y3Fzpo1_YWc6sk-lmAatoXlQ4QO3LbkZB9leg)和[TRO](https://ieeexplore.ieee.org/document/9686955)两个版本，本文将对其系统进行深入研究，结合论文以及代码完成从理论到工程实现的全方位解读。

Kimera-Multi [ICRA](https://ieeexplore.ieee.org/abstract/document/9561090?casa_token=RNUxNIr638MAAAAA:E5EbWTn2-9VzpIeRaxqsDYlHpFjnrddeZs-0OCmR6Y3Fzpo1_YWc6sk-lmAatoXlQ4QO3LbkZB9leg) 版：
```
@INPROCEEDINGS{9561090,
  author={Chang, Yun and Tian, Yulun and How, Jonathan P. and Carlone, Luca},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Kimera-Multi: a System for Distributed Multi-Robot Metric-Semantic Simultaneous Localization and Mapping}, 
  year={2021},
  volume={},
  number={},
  pages={11210-11218},
  keywords={Solid modeling;Three-dimensional displays;Simultaneous localization and mapping;Protocols;Computational modeling;Data models;Trajectory},
  doi={10.1109/ICRA48506.2021.9561090}}

```

Kimera-Multi [TRO](https://ieeexplore.ieee.org/document/9686955) 版：
```
@ARTICLE{9686955,
  author={Tian, Yulun and Chang, Yun and Herrera Arias, Fernando and Nieto-Granda, Carlos and How, Jonathan P. and Carlone, Luca},
  journal={IEEE Transactions on Robotics}, 
  title={Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems}, 
  year={2022},
  volume={38},
  number={4},
  pages={2022-2038},
  keywords={Simultaneous localization and mapping;Trajectory;Collaboration;Semantics;Estimation;Robot kinematics;Solid modeling;Multi-robot systems;simultaneous localization and mapping;robot vision systems},
  doi={10.1109/TRO.2021.3137751}}

```

## 1. 理解TSDF, Mesh, surfels之间的关系

### 1.1 TSDF
TSDF (Truncated Signed Distance Function, 截断符号距离函数)：像素（Pixel）是二维图像的基本单位，而体素（Voxel）是三维空间的基本单位。空间被划分为 3D 的体素网格。每个体素里存的不是颜色，而是一个距离值——该体素到最近物体表面的距离。现在我们有了一个由无数个“体素”填满的房间。假设房间中间放着一个真实的苹果。我们怎么在这些体素里记录这个苹果的形状呢？这就是 TSDF 要干的活。

* Distance（距离）：我们在每一个体素里存一个数字，这个数字代表**当前这个体素的中心点，距离苹果表面有多远**。
* Signed（符号）：为了区分物体的里面和外面，我们人为规定：如果体素在苹果外面（空气中），距离记为正数（比如 +2 厘米）。如果体素在苹果里面（果肉里），距离记为负数（比如 -1 厘米）。苹果的表面（表皮），就是数值正好等于 0 的地方！ 这在数学上叫**零交叉点（Zero-crossing）**。
* Truncated（截断）：房间很大，远离苹果的体素（比如离了 3 米远）记录精确距离既浪费内存又没意义。所以我们设定一个“截断范围”（比如贴近苹果表面的前后 5 厘米）。只要超出这个范围的体素，我们统统记成一个最大值（比如统统记为 +5 厘米），不再精细计算。这就极大地节省了算力。

体素是容器（一个个小方块），TSDF 是装在容器里的数据（带有正负号的距离值）。我们通过看哪些体素里存的数值是 0，就能完美地勾勒出物体的表面轮廓。


### 1.2 Mesh

虽然 TSDF 很好地用数学方式描述了物体的表面，但它对于渲染（给人类看）和物理碰撞（让机器人避障）来说不够直观。这时候就需要 Mesh。Mesh 是由顶点（Vertices）、**边（Edges）和面（Faces，通常是三角形）**组成的纯表面几何结构。它内部是空的。

从装满 TSDF 数据的体素阵列中变换成为Mesh用到了图形学中非常著名的算法：移动立方体（Marching Cubes）。Kimera 论文中也明确提到，它使用 Marching Cubes 算法从 TSDF 提取出网格。这个算法的工作原理是：遍历所有的体素，找到那些存着正数和负数交界的体素（说明这里穿过了表面，数值从正变负，一定经过了 0），然后在这些数值为 0 的位置上“打点”（生成顶点），并用三角形把这些点连起来。连完之后，一层 3D Mesh 外壳就被提取出来了。

### 1.3 surfels面元

我记得fast-livo2也用到了这个概念。

不把表面看成是由三角形拼接成的网格，而是把它看成由无数个**带有法向量、半径、颜色和权重的小圆盘（Disks）**堆叠覆盖而成的。像 ElasticFusion 这样的系统极度偏爱 Surfels。因为在高度动态的稠密建图过程中，如果要维护 Mesh，每次更新位姿都需要重新计算拓扑结构，计算代价极其昂贵。而 Surfels 是相互独立的，当你发现闭环（Loop Closure）导致地图发生形变时，只需要直接移动这些“小圆盘”的位置和法向量即可，完美规避了复杂的拓扑维护问题。

从代码看：

* Kimera-Multi 的分布式后端（DPGO）主要优化的是位姿图，不是全局 landmark 图。

* landmark 在系统里主要用于单机 VIO/建图与可视化（前端+本地后端），而不是多机分布式后端的联合变量。

DPGO 的图分类只有 odom、private loop closure、shared loop closure，分布式前端输出给后端的增量图在处理时只看 ODOM 和 LOOPCLOSE，虽然消息定义里有 LANDMARK 枚举，但当前分布式链路没有把它作为后端可优化实体来处理，landmark 的“实质优化”主要在单机 VIO 后端（smart factor + iSAM2）

## 2.Kimera-VIO单机里程计


### 系统的宏观建模：MAP 与因子图

在 Kimera-VIO 的底层（基于 GTSAM），整个系统被建模为一个**最大后验概率估计（Maximum A Posteriori, MAP）**问题。

假设我们在时间段内有一系列的关键帧（状态），定义状态向量 $$x_i = (R_i, p_i, v_i, b_{g_i}, b_{a_i})$$，其中包括相机的旋转、平移、速度以及 IMU 的陀螺仪和加速度计偏置。
定义环境中被观测到的 3D 路标点集合为 $$L = \{l_1, l_2, \dots, l_M\}$$。

给定所有的 IMU 测量值 $\mathcal{Z}_{IMU}$ 和视觉特征点像素观测值 $\mathcal{Z}_{C}$，我们的目标是寻找最优的状态 $X$ 和特征点 $L$，使得后验概率 $$P(X, L | \mathcal{Z}_{IMU}, \mathcal{Z}_{C})$$ 最大化。

在假设观测噪声服从零均值高斯分布的前提下，取负对数后，这个 MAP 问题就等价于求解一个**非线性最小二乘问题（即 Loss Function）**On-Manifold_Preintegration_for_Real-Time_Visual-In.pdf]：

$$\min_{X, L} \left( \sum_{(i,j) \in \mathcal{E}_{IMU}} \| r_{\mathcal{I}_{ij}} \|^2_{\Sigma_{ij}} + \sum_{(i,k) \in \mathcal{E}_{Vision}} \| r_{\mathcal{C}_{ik}} \|^2_{\Sigma_C} + \| r_{prior} \|^2 \right)$$

这里面的核心就是前两项：**IMU 预积分残差** $r_{\mathcal{I}_{ij}}$ 和 **视觉重投影残差** $r_{\mathcal{C}_{ik}}$。


### IMU 预积分残差是怎么来的？（基于文献 [27]）

常规的做法是，状态量 $x_i$ 经过高频的 IMU 积分，预测出 $x_j$。但如果你每次优化改变了 $x_i$，你都要把中间这几百个 IMU 数据重新积分一遍，计算量爆炸。

文献 [27] 的核心贡献是推导了**流形上的预积分（On-Manifold Preintegration）**On-Manifold_Preintegration_for_Real-Time_Visual-In.pdf]。它把 $t_i$ 到 $t_j$ 之间的所有高频 IMU 数据打包成了一个**独立于绝对位姿 $x_i$** 的相对运动观测量：预积分旋转 $\Delta \tilde{R}_{ij}$、预积分速度 $\Delta \tilde{v}_{ij}$ 和预积分平移 $\Delta \tilde{p}_{ij}$。

IMU 的残差项（Loss 的一部分）被定义为预测状态与预积分测量值之间的差异：
$$
r_{\mathcal{I}_{ij}} =
\begin{bmatrix}
r_{\Delta R_{ij}} \\
r_{\Delta v_{ij}} \\
r_{\Delta p_{ij}}
\end{bmatrix}
=
\begin{bmatrix}
\log\left( (\Delta \tilde{R}_{ij}(\bar{b}^g))^T R_i^T R_j \right) \\
R_i^T (v_j - v_i - g \Delta t) - \Delta \tilde{v}_{ij}(\bar{b}^g, \bar{b}^a) \\
R_i^T (p_j - p_i - v_i \Delta t - \frac{1}{2}g \Delta t^2) - \Delta \tilde{p}_{ij}(\bar{b}^g, \bar{b}^a)
\end{bmatrix}
$$

**直观理解**：这就相当于在因子图中，直接在关键帧 $i$ 和 $j$ 之间连了一条“弹簧”（IMU Factor），弹簧的自然长度就是预积分算出来的相对运动，谁违反了这个相对运动，Loss 就会惩罚谁On-Manifold_Preintegration_for_Real-Time_Visual-In.pdf]。

---

### 无结构视觉模型是怎么消元的？（基于文献 [59]）

这个过程在数学上叫 **零空间投影（Null-space Projection）**，在 SLAM 中也常被称为 **Schur Complement（舒尔补）消元**。

#### 常规 Bundle Adjustment (BA) 的痛点
常规的视觉残差（重投影误差）是这样的：相机在位姿 $x_i$ 看到了 3D 点 $l_j$，像素观测值为 $z_{ij}$。
$$r_{ij} = \pi(x_i, l_j) - z_{ij}$$
（其中 $\pi$ 是相机投影模型）

如果你有 10 个关键帧（状态量很少）和 1000 个 3D 点（状态量极大），如果把它们都塞进状态向量里去优化（求解 $H \Delta x = b$），雅可比矩阵 $H$ 的维度会达到三千多维，求解非常慢。

#### DLT 线性三角化的作用
我们要对上面的非线性函数 $r_{ij}$ 进行泰勒一阶展开（线性化），线性化需要一个**工作点（Operating Point）**。
这就是为什么论文里说：“At each iSAM2 iteration, estimates 3D position using DLT”Eliminating_conditionally_independent_sets_in_factor_graphs_A_unifying_perspective_based_on_smart_factors.pdf]。
因为 GTSAM 在做非线性优化迭代时，需要知道这个点大概在什么物理位置，才能求雅可比矩阵。所以它利用当前相机位姿的最优估计，通过 DLT（直接线性变换）快速算出 $l_j$ 的 3D 坐标 $l_{j0}$。

#### 数学推导：雅可比矩阵的零空间投影（重点！）
我们在当前状态 $(x_{0}, l_{j0})$ 处对单个特征点 $j$ 在多个相机上的观测残差进行泰勒展开：
$$e \approx e_0 + F \delta x + E \delta l_j$$
* $e_0$：当前位姿和 3D 点算出的重投影误差向量。
* $F$：误差对**相机位姿**的雅可比矩阵。
* $E$：误差对 **3D 路标点**的雅可比矩阵。
* $\delta x$：我们要优化的相机位姿增量。
* $\delta l_j$：我们要优化的 3D 点坐标增量。

在优化时，我们希望让残差趋近于 0，也就是求解：
$$F \delta x + E \delta l_j = -e_0$$

**这里就是见证奇迹的时刻**：由于我们其实根本不关心 3D 点究竟长啥样（我们只要算准自己的轨迹），我们能不能把 $\delta l_j$ 从方程里干掉？

可以！在数学上，矩阵 $E$（维度通常是 $2N \times 3$，N 是观测到该点的相机数）存在一个**左零空间（Left Null Space）**。我们可以对 $E$ 进行 QR 分解，或者奇异值分解（SVD），找到一个零空间矩阵 $N_{null}$，使得：
$$N_{null}^T E = \mathbf{0}$$

既然有这个性质，我们在原方程两边同时左乘 $N_{null}^T$：
$$N_{null}^T F \delta x + N_{null}^T E \delta l_j = - N_{null}^T e_0$$
因为 $N_{null}^T E = 0$，与 3D 点增量 $\delta l_j$ 相关的项**直接消失了**！

方程变成了：
$$(N_{null}^T F) \delta x = - N_{null}^T e_0$$
$$\Rightarrow F_{proj} \delta x = - e_{proj}$$

这就是论文中所说的 **"analytically eliminates the corresponding 3D points from the VIO state"** 的数学真相Eliminating_conditionally_independent_sets_in_factor_graphs_A_unifying_perspective_based_on_smart_factors.pdf]。

#### 举个具体例子帮你理解
假设：你在房间里走动，在 3 个不同的位置（帧 1、帧 2、帧 3）都看到了桌子上的同一个杯子把手（特征点 $l_1$）。

* **常规方法（带结构）**：状态机里有 `[位姿1, 位姿2, 位姿3, 杯子把手XYZ]`。优化器为了让你对齐图像，会同时去挪动 3 个位姿和这个杯子把手的 XYZ。变量多，算得慢。
* **Kimera (Smart Factor / 零空间) 做法**：
    1. 既然我在这 3 个位置看到了同一个点，说明这 3 条视线（Rays）在物理世界里**必须相交于一点**。
    2. 这个“三线共点”的几何约束，其实**本质上只是限制了这 3 个位姿的相对关系**（就像是对极几何里的本质矩阵约束的多元推广）。
    3. 我们用 DLT 算出一个杯子把手的临时位置（为了求雅可比算梯度）。
    4. 然后我们通过 $N_{null}^T$ 矩阵投影，把误差转化成了**纯粹针对“位姿 1、位姿 2、位姿 3 相对关系”的惩罚项**。
    5. 最后送给 iSAM2 去优化的状态向量仅仅是 `[位姿1, 位姿2, 位姿3]`。那个杯子把手被变成了一个维系这 3 个位姿的“多边暗含弹簧”（Smart Factor），路标点再也没有进入过状态更新的循环。

### 总结

结合这两篇核心文献，Kimera-VIO 求解的优化问题实际上是：
**在一个仅包含“相机状态变量（带 IMU 偏置）”的极小状态空间里，同时最小化“IMU 相对运动违规惩罚”和“多视图共视几何违规惩罚（经过零空间投影后的视觉残差）”。**

这种彻底将路标点边缘化/消元的设计，让 VIO 后端的 Hessian 矩阵保持极其稀疏和极其微小的规模，这也是为什么 Kimera 能够仅仅依靠 CPU 就能实时跑满前端帧率的核心奥秘所在。


### 一 变量定义与 IMU 的“阿喀琉斯之踵” (Bias)

在 VIO（视觉惯性里程计）系统中，我们有两类传感器数据，也就是你问的 $\mathcal{Z}_{IMU}$ 和 $\mathcal{Z}_C$：

* **$\mathcal{Z}_{IMU}$（IMU 测量值）**：它是 IMU 输出的超高频（例如 200Hz）原始数据，包含**三轴加速度** $a_m$ 和**三轴角速度** $\omega_m$。
* **$\mathcal{Z}_C$（视觉观测值）**：它是相机在某一时刻（例如 20Hz）拍下一张照片后，特征点在图像上的 **2D 像素坐标** $(u, v)$。

**为什么 $b_a, b_g$ 是状态变量？它们代表什么？**
$b_a$ 和 $b_g$ 分别是加速度计偏置（Bias of accelerometer）和陀螺仪偏置（Bias of gyroscope）。
当你给像 PX4 这样的飞控主板或者机器人上电后，即使把它们静置在绝对水平、绝对静止的桌面上，你去看底层数据，陀螺仪的读数绝对不是严格的 $0$，加速度计除了重力方向，其他轴也不是严格的 $0$。这个固定的偏移量就是 Bias。

更要命的是，由于温度变化和机械应力，这个 Bias 是**随时间缓慢漂移（Random Walk）**的。如果我们不把它当作一个需要实时估计的“状态变量（State）”并在积分时减去它，哪怕只有极其微小的误差，经过随时间的二次积分，机器人的位置估算就会在几秒钟内飘到九霄云外。因此，在图优化中，偏置必须被当作未知数进行实时估计和修正。

---

### 从 MAP 到非线性最小二乘：取负对数的魔法

在 AI 和机器学习的算法推导中，将最大后验概率（MAP）或最大似然估计（MLE）转化为最小化 Loss 函数是一个非常经典的数学连招。

1. **最大后验概率 (MAP)**：
   根据贝叶斯定理，我们要在给定传感器观测值 $Z$ 的情况下，求出最可能的机器人状态 $X$：
   $$P(X|Z) \propto P(Z|X)P(X)$$
   我们要找一个 $X$，让这个连乘的概率最大。

2. **高斯噪声假设**：
   现实中，传感器的测量误差通常服从零均值的高斯分布。一个高斯分布的概率密度函数长这样：
   $$P(Z|X) \propto \exp\left(-\frac{1}{2} \| h(X) - Z \|^2_{\Sigma}\right)$$
   其中 $h(X)$ 是我们根据当前状态预测出来的理论观测值，$Z$ 是实际观测值，$\Sigma$ 是协方差矩阵（权重）。

3. **取负自然对数 ($-\ln$)**：
   因为 $\ln(x)$ 是单调递增函数，求最大化概率 $P$ 就等价于求最小化 $-\ln(P)$。当我们对上面的高斯函数取 $-\ln$ 时，奇迹发生了：
   * $\exp$ 被 $\ln$ 消掉了。
   * 负号被 $-\ln$ 的负号抵消了。
   * 连乘 $P(Z_1|X) \times P(Z_2|X)$ 在对数下变成了求和 $\sum$。

最终，那个极其复杂的概率最大化公式，变成了一个非常清爽的加法算式：
$$\min_{X} \sum \frac{1}{2} \| h(X) - Z \|^2_{\Sigma}$$
**“预测值减去实际值，求平方，加权重，求和”——这就是标准的非线性最小二乘问题（Nonlinear Least Squares）**！它就是我们在图优化中常说的 Loss Function。


### 什么是“预积分打包”，为什么要这么干？

这是你提问中最核心的部分。我们要弄懂流形上的预积分在干嘛，必须先看看“标准积分”有多坑。

#### 坑人的“标准积分”与“重新积分”

假设机器人在 $t_i$ 时刻拍了一张图（状态为 $x_i$），在 $t_j$ 时刻又拍了一张图（状态为 $x_j$）。这两张图之间隔了 0.1 秒，期间 IMU 输出了 20 个数据。

按照牛顿力学，我们要用 $x_i$ 加上这 20 个 IMU 数据，推导出 $x_j$ 的位置 $p_j$：
$$p_j = p_i + v_i \Delta t + \iint_{t_i}^{t_j} (R_t a_t - g) dt^2$$
注意看积分号里面的 $R_t$（机器人在 $t$ 时刻的绝对姿态）。由于引力 $g$ 是在世界坐标系下的，我们必须把 IMU 测到的局部加速度 $a_t$，乘以**此时刻的绝对姿态 $R_t$**，转到世界坐标系下，才能减去重力并积分。

**灾难发生了**：在后端（比如 GTSAM 或 Ceres）进行非线性优化时，求解器为了找到最优解，会不断地微调 $x_i$ 的值（比如让 $t_i$ 时刻的姿态 $R_i$ 偏转 1 度）。
一旦起点姿态 $R_i$ 变了，后续所有时刻的绝对姿态 $R_t$ 就全变了！这就意味着，**优化器每微调一次起点位姿，你都必须把这 20 个 IMU 数据从头到尾重新乘一遍、积一遍分！** 优化过程可能要迭代几十次，这个计算量在 CPU 上根本吃不消。

#### “预积分打包”的数学魔法

学者们想到了一个天才的主意：**我能不能把这 20 个 IMU 数据算出一个相对于起点 $t_i$ 的内部运动量，让它与世界坐标系无关？**

我们把积分里的绝对姿态 $R_t$ 拆开：$R_t = R_i \Delta R_{i,t}$ （即：当前姿态 = 起点姿态 $\times$ 相对起点的旋转）。
代入刚才的位移公式，并把与起点 $x_i$ 相关的项全提出来：
$$p_j = p_i + v_i \Delta t - \frac{1}{2}g \Delta t^2 + R_i \iint_{t_i}^{t_j} (\Delta R_{i,t} a_t) dt^2$$

仔细看最后一项 $\iint_{t_i}^{t_j} (\Delta R_{i,t} a_t) dt^2$！
它里面的 $\Delta R_{i,t}$ 是靠陀螺仪自身的角速度积分算出来的（相对于 $t_i$ 的偏转），$a_t$ 是加速度计读数。
**这整个双重积分项，完全只依赖于 IMU 的原始数据，和机器人的绝对位姿 $R_i, p_i, v_i$ 没有任何关系！**

因此，当前端收到这 20 个 IMU 数据时，我们可以**一次性**把它们“打包”成一个独立的相对位移观测值 $\Delta \tilde{p}_{ij}$。这就是所谓的“预积分（Preintegration）”。


在后端构建因子图时，两个关键帧之间连着一个 IMU 预积分因子。它的残差就变成了：
$$e = \text{预测相对位移} (x_i, x_j) - \text{预先打包好的独立测量值} (\Delta \tilde{p}_{ij})$$

现在，当优化器疯狂调整 $x_i$ 的位姿时，**“打包好的测量值”不需要做任何改变**！我们只需要用新的 $x_i$ 算一下方程前面的项，做一下简单的加减法对比即可。这就把计算复杂度降低了几个数量级。


---

我们已经把单机版 Kimera-VIO 的底层基石（空间上的舒尔补消元、时间上的 IMU 预积分）彻底夯实了。基于这些概念，你想直接开始探讨 Kimera-Multi 是如何在多机之间建立分布式通信，还是先看看多机场景下令人头疼的“感知混淆”问题是如何毁掉单机位姿图优化的？





## 分布式优化中的“全局”究竟是什么？

$$\min_{X_A,X_B};E_A^{odom}(X_A)+E_A^{intra}(X_A)+E_B^{odom}(X_B)+E_B^{intra}(X_B)+E_{AB}^{inter}(X_A,X_B)$$

$$X_A^{k+1}=\arg\min_{X_A}F(X_A,X_B^k),\quad
X_B^{k+1}=\arg\min_{X_B}F(X_A^{k+1},X_B)$$

$$X_A^{k+1}=\arg\min_{X_A}F(X_A,X_B^k)=\arg\min_{X_A}E_A^{odom}(X_A)+E_A^{intra}(X_A)+E_{AB}^{inter}(X_A,X_B)$$