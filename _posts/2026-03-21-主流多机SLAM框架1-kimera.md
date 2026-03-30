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


这是一个非常经典的困惑。你能精准地把这两张图（公式 28-30 和公式 46-48）放在一起提问，说明你已经敏锐地抓住了这篇论文的“命门”。

在 SLAM 领域的顶会上，很多新手甚至部分老手都会在这里卡壳。因为论文作者在这里完成了一个非常生硬的“概念跳跃”：前一张图还在讲**微积分**，后一张图突然就变成了**流形上的李代数与误差构建**。

作为工程师，我们要把这种学术表达翻译成写代码时的底层逻辑。在构建后端优化器（如 GTSAM 或 Ceres）时，我们心里必须永远装着**“两个世界”**：
1. **测量世界 (Measurement)**：传感器**实际上**读到了什么？
2. **预测世界 (Prediction)**：根据当前的系统状态，理论上**应该**读到什么？

**所谓的“残差（Residual）”，就是“预测世界”减去“测量世界”。**

我们现在就用这两个世界的视角，一步步把复杂的矩阵和残差公式是怎么从打包公式里推导出来的，彻底扒干净。

---

### 第一步：测量世界 —— 你的第一张图（公式 28-30）

这三个公式 $\Delta \tilde{R}_{ij}$、$\Delta \tilde{v}_{ij}$、$\Delta \tilde{p}_{ij}$，就是我们上一轮说的**“打包”**动作的最终产物。

当前端收到这几百个高频的 IMU 读数时，它会在 $t_i$ 时刻的**局部坐标系（相机/IMU 本体系）**下，疯狂地做离散积分。
* **公式 28**：把所有的陀螺仪读数 $\tilde{\omega}$ 乘起来，得到一个纯粹由 IMU 算出来的**相对旋转**。
* **公式 29**：把所有的加速度读数 $\tilde{a}$ 积分一次，得到一个纯粹由 IMU 算出来的**相对速度变化量**。
* **公式 30**：把所有的加速度读数 $\tilde{a}$ 积分两次，得到一个纯粹由 IMU 算出来的**相对位移变化量**。

**工程核心**：算完这三个值之后，这几百个高频 IMU 数据就可以**全部扔掉**了！这三个值作为绝对的“常量”（在偏置不发生大更新的前提下），被死死地钉在内存里。这就是测量世界给出的**“不可辩驳的物理事实”**。

---

### 第二步：预测世界 —— 状态量应该长什么样？

现在，我们来到 GTSAM 优化器的内部。优化器手里捏着的是关键帧 $i$ 和 $j$ 在世界坐标系下的**绝对状态**：
* 帧 $i$ 的状态：$R_i, p_i, v_i$
* 帧 $j$ 的状态：$R_j, p_j, v_j$

优化器会问自己一个问题：“如果我现在瞎猜的这些绝对状态是对的，那么从 $i$ 到 $j$，机器人在 $i$ 的**局部坐标系**下，**理论上**应该发生了怎样的相对运动？”

我们把牛顿运动学公式倒过来写，把 $j$ 的状态减去 $i$ 的状态，并且**统一左乘 $R_i^T$**（因为 $R_i^T$ 的物理意义就是把世界坐标系下的向量，旋转投影回 $i$ 时刻的局部坐标系）：

1. **理论相对旋转**：世界坐标系下，$R_j = R_i \Delta R$，那么相对旋转就是 $R_i^T R_j$。
2. **理论相对速度**：世界速度的差值是 $v_j - v_i$。但别忘了重力 $g$ 一直在加速，所以要减去重力带来的速度增量 $g \Delta t_{ij}$。最后投影到 $i$ 系：
   $$理论 \Delta v = R_i^T (v_j - v_i - g \Delta t_{ij})$$
3. **理论相对位移**：世界位移的差值是 $p_j - p_i$。我们要减去初速度带来的位移 $v_i \Delta t_{ij}$，再减去重力带来的位移 $\frac{1}{2}g \Delta t_{ij}^2$。最后投影到 $i$ 系：
   $$理论 \Delta p = R_i^T (p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2}g \Delta t_{ij}^2)$$

---

### 残差的碰撞 —— 你的第二张图（公式 46-48）

现在，“预测世界”和“测量世界”要发生碰撞了。残差的定义就是：**理论预测值 $\ominus$ IMU 测量值**。

我们直接看你传的第二张图，看它是怎么完美对应的：

#### 速度残差 (公式 47, $\delta v_{ij}$) 和 位移残差 (公式 48, $\delta p_{ij}$)
因为速度和位移都是三维向量（存在于欧式空间 $\mathbb{R}^3$），它们可以直接做减法！
你仔细看图 2 中的公式 47 和 48：

$$r_{\Delta v_{ij}} = \underbrace{R_i^T (v_j - v_i - g \Delta t_{ij})}_{\text{优化器预测的理论值}} - \underbrace{\Delta \tilde{v}_{ij}}_{\text{图1打包好的 IMU 测量值}}$$

$$r_{\Delta p_{ij}} = \underbrace{R_i^T (p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2}g \Delta t_{ij}^2)}_{\text{优化器预测的理论值}} - \underbrace{\Delta \tilde{p}_{ij}}_{\text{图1打包好的 IMU 测量值}}$$

你看，这两项是不是就是极其简单的算术相减？没有任何黑魔法。

#### 旋转残差的“黑魔法” (公式 46, $\delta R_{ij}$)
为什么图 2 中的公式 46 长得这么奇怪，外面还套了一个 $\log$？
因为旋转矩阵 $R$ 属于 $SO(3)$ 李群，**矩阵之间不能直接相减！** （两个旋转矩阵相减得到的根本不是一个合法的旋转）。

在李群上，求两个旋转矩阵 $A$ 和 $B$ 的“差”，必须用乘法：$A^{-1} B$ 或者 $A^T B$。
所以：

$$\text{旋转误差矩阵} = (\text{IMU 测量值})^{-1} \times (\text{优化器预测值})$$

$$\text{旋转误差矩阵} = (\Delta \tilde{R}_{ij})^T (R_i^T R_j)$$

但是，GTSAM 优化器在求解高斯-牛顿方程 $H \Delta x = b$ 时，残差 $b$ 必须是一个普通的 3D 向量（属于李代数 $\mathfrak{so}(3)$），而不能是一个 $3 \times 3$ 的矩阵。
怎么把 $3 \times 3$ 的旋转误差矩阵“拍扁”成一个 3D 向量？答案就是**对数映射（Log Map）**！
所以公式 46 变成了：

$$r_{\Delta R_{ij}} = \log \left( (\Delta \tilde{R}_{ij})^T R_i^T R_j \right)^\vee$$

（注：外面的那个小 $\vee$ 符号，在李代数里叫 vee 算子，意思是把反对称矩阵转成 3 维向量，很多论文里省略不写，但代码里必须有）。

---

### 总结：这三个公式在代码里到底是怎么跑的？

1. **预处理线程（跑在前端）**：只用图 1 的公式。来一个 IMU 数据就乘一次，算出一个纯净的 $\Delta \tilde{R}, \Delta \tilde{v}, \Delta \tilde{p}$。算完就把这堆 IMU 数据扔进垃圾桶。
2. **优化线程（跑在后端）**：拿着前端给的这三个常数，构建图 2 的公式（也就是 GTSAM 里的 `ImuFactor`）。
3. **迭代求解**：GTSAM 疯狂地改变 $R_i, p_i, v_i$ 的值，每次改变，都把它代入图 2 的公式中算出新的残差 $r$。当这三个残差无限逼近于 $0$ 的时候，说明**“优化器的预测”完美契合了“IMU 的物理观测”**，此时的绝对轨迹就是最优解。


---

### 第一步：系统建模与全局 Loss 函数

首先，在优化器眼中，世界是由**状态变量（待求未知数）**和**残差项（约束条件）**组成的。

**1. 状态向量 (State Vector)**
假设我们要优化 $N$ 个关键帧的轨迹。在优化器的内存里，有一个巨大的状态向量 $\mathcal{X}$，包含了每一个关键帧 $i$ 的绝对姿态、速度、以及**这一时刻的偏置**：
$$\mathcal{X} = \{x_1, x_2, \dots, x_N\}$$
其中，每一个节点的状态为：$x_i = [R_i, p_i, v_i, b_{a_i}, b_{g_i}]$。
*注：这里的 $b_{a_i}, b_{g_i}$ 就是优化器当前时刻对偏置的“最新信念（信念值）”，它们是活的变量，是我们要去更新的目标！*

**2. 全局 Loss 函数 (Cost Function)**
我们将整个系统的 MAP（最大后验估计）转化为一个巨大的非线性最小二乘问题。我们要寻找一组最优的 $\mathcal{X}$，使得总 Loss 最小：
$$\min_{\mathcal{X}} \left( \sum_{(i,k)} \| r_{\mathcal{C}_{ik}} \|^2_{\Sigma_C} + \sum_{(i,j)} \| r_{\mathcal{I}_{ij}} \|^2_{\Sigma_I} + \sum_{(i,j)} \| r_{b_{ij}} \|^2_{\Sigma_b} \right)$$
* 第 1 项：视觉重投影残差（我们前面讲过的，消元了 3D 点之后的视觉约束）。
* **第 2 项：IMU 预积分残差（维系位姿和速度的物理定律）。**
* **第 3 项：偏置游走残差（维系偏置随时间缓慢漂移的特性）。**

---

### 第二步：写出残差的“完全体”公式

现在，我们把第 2 项和第 3 项彻底展开，看看包含了偏置的残差到底长什么样。

**1. 预积分常数与雅可比（来自前端的遗产）**
在进入优化器之前，前端已经用**某个旧的偏置初始猜测值** $\bar{b}_{a}, \bar{b}_{g}$ 把高频 IMU 数据打包完毕，生成了死死钉在内存里的常数：
* 测量值常数：$\Delta \tilde{R}(\bar{b}), \Delta \tilde{v}(\bar{b}), \Delta \tilde{p}(\bar{b})$
* 雅可比常数：$J_{b_g}^{\Delta R}, J_{b_a}^{\Delta v}, J_{b_g}^{\Delta v}, J_{b_a}^{\Delta p}, J_{b_g}^{\Delta p}$

**2. 组装 IMU 预积分残差 $r_{\mathcal{I}_{ij}}$**
优化器在当前迭代步，手里有变量 $b_{a_i}, b_{g_i}$。它首先计算出当前变量与前端那个“旧偏置”的差值（补丁）：
$$\delta b_a = b_{a_i} - \bar{b}_a$$
$$\delta b_g = b_{g_i} - \bar{b}_g$$

然后，把带有补丁的残差完全体写出来（以速度残差为例，旋转和位移同理）：
$$r_{\Delta v_{ij}} = \underbrace{R_i^T(v_j - v_i - g\Delta t)}_{\text{预测世界：来自当前状态 } R_i, v_i, v_j} - \underbrace{(\Delta \tilde{v}(\bar{b}) + J_{b_a}^{\Delta v} \delta b_a + J_{b_g}^{\Delta v} \delta b_g)}_{\text{测量世界：用泰勒展开修正后的预积分值}}$$

**3. 组装偏置游走残差 $r_{b_{ij}}$**
这个非常简单，IMU 偏置在短时间内应该是基本不变的（服从随机游走高斯模型）：
$$r_{b_a} = b_{a_j} - b_{a_i}$$
$$r_{b_g} = b_{g_j} - b_{g_i}$$

---

### 第三步：偏置究竟是怎么被更新的？（高斯-牛顿法求解）

重头戏来了。现在的 Loss 函数是一个关于 $\mathcal{X}$（包含 $b_a, b_g$）的极其复杂的非线性函数。我们怎么把它们算出来？

现代 SLAM 求解器（如 iSAM2 或 Ceres）使用**高斯-牛顿法（Gauss-Newton）**进行迭代求解。

**1. 对残差本身求雅可比**
设我们将所有的残差组合成一个大向量 $r(\mathcal{X})$。由于它是非线性的，我们在**当前的系统状态 $\mathcal{X}$ 处**对它进行一阶泰勒展开，寻找一个状态增量 $\Delta \mathcal{X}$：
$$r(\mathcal{X} \oplus \Delta \mathcal{X}) \approx r(\mathcal{X}) + \mathbf{H} \Delta \mathcal{X}$$
*注意：这里的 $\mathbf{H}$ 是**残差关于系统状态变量的雅可比矩阵**，不要和前面那个预积分对偏置的雅可比搞混了！*

其中，状态增量 $\Delta \mathcal{X}$ 包含了所有关键帧的位姿增量和**偏置增量**：
$$\Delta x_i = [\delta \theta_i, \delta p_i, \delta v_i, \mathbf{\Delta b_{a_i}}, \mathbf{\Delta b_{g_i}}]^T$$

**2. 构建正规方程并求解**
我们要让展开后的残差平方和最小，令导数为 0，会得到著名的正规方程（Normal Equation）：
$$(\mathbf{H}^T \Sigma^{-1} \mathbf{H}) \Delta \mathcal{X} = -\mathbf{H}^T \Sigma^{-1} r(\mathcal{X})$$
求解这个巨大的线性方程组，我们就得到了这一个迭代步的最优状态增量 $\Delta \mathcal{X}$。**你朝思暮想的偏置更新量 $\mathbf{\Delta b_{a_i}}, \mathbf{\Delta b_{g_i}}$ 就在这个解里面！**

**3. 状态更新**
把算出来的增量加到当前状态上（对于旋转使用李群乘法 $\oplus$，对于偏置和位置直接用加法）：
$$b_{a_i}^{new} = b_{a_i}^{old} + \mathbf{\Delta b_{a_i}}$$
$$b_{g_i}^{new} = b_{g_i}^{old} + \mathbf{\Delta b_{g_i}}$$

---

### 第四步：完整的代码执行大循环（点睛之笔）

为了让你彻底看清这套体系的精妙，我们以代码的视角模拟一次完整的运行：

1. **[前端]** $t_0 \rightarrow t_1$：来了 100 个 IMU 数据。前端随便猜一个初始偏置 $\bar{b} = 0$，积出一个常数 $\Delta \tilde{v}(0)$ 和雅可比 $J$。传给后端。
2. **[后端 第 1 次迭代]**
   * 当前状态机里的变量 $b = 0$。
   * 算补丁 $\delta b = 0 - 0 = 0$。带入公式，残差 $r = \text{预测} - \Delta \tilde{v}(0)$。
   * 构建 $\mathbf{H}$ 矩阵，解方程，算出增量 $\Delta \mathcal{X}$，发现偏置应该加上 $0.05$。
   * **更新状态：$b^{new} = 0.05$。**
3. **[后端 第 2 次迭代]**
   * 当前状态机里的变量变成了 $b = 0.05$。
   * **核心绝杀**：前端的常数还是 $\Delta \tilde{v}(0)$，不需要重新积分！
   * 算补丁：$\delta b = 0.05 - 0 = 0.05$。
   * 把补丁代入完全体公式：新的等效测量值变成了 $\Delta \tilde{v}(0) + J \times 0.05$。
   * 用这个新的等效测量值算出新的残差 $r$，再次构建 $\mathbf{H}$，解方程，再次更新 $b^{new}$。
4. **[收敛]**
   * 迭代几次后，增量 $\Delta \mathcal{X} \approx 0$。此时的 $b$ 就是我们从头到尾要估计的极其精确的真实偏置！
   * 如果此时 $\delta b$ 变得很大（比如超过了泰勒展开的线性近似有效范围），系统就会触发一种叫 "Re-preintegration" 的机制，强迫前端用当前的 $b$ 作为新的 $\bar{b}$，老老实实地去把那 100 个数据重新积分一次（但这极少发生）。

**总结：**
偏置 $b_a, b_g$ 是作为**系统状态机里的核心一等公民**，在后端的 Gauss-Newton 迭代中，与相机的位姿 $R, p$ 放在同一个方程组里被联合求解出来的。而预积分的泰勒展开机制，仅仅是为了在每一次迭代计算残差时，能够“瞬间”模拟出改变偏置后的物理结果，从而免去了重新微积分的算力地狱。


在 GTSAM 的 `ImuFactor` 底层源码中，位移、速度和旋转的残差“完全体”公式如下。它们都遵循着 **“预测世界 $\ominus$ 测量世界（带偏置一阶泰勒修正）”** 的核心逻辑。

### 1. 位移残差完全体（包含二重积分项标记）

位移残差处于欧式空间，可以直接使用向量减法。你之前最关心的**二重积分**，正是对应着这里的 $\Delta \tilde{p}_{ij}(\bar{b})$。

$$r_{\Delta p_{ij}} = \underbrace{R_i^T \left( p_j - p_i - v_i \Delta t_{ij} - \frac{1}{2}g \Delta t_{ij}^2 \right)}_{\text{预测世界：由当前的 } R_i, p_i, p_j, v_i \text{ 计算出理论相对位移}} - \underbrace{\left( \overbrace{\Delta \tilde{p}_{ij}(\bar{b})}^{\text{物理本质：}\iint (\Delta R \cdot a) dt^2} + J_{b_a}^{\Delta p} \delta b_a + J_{b_g}^{\Delta p} \delta b_g \right)}_{\text{测量世界：用泰勒展开补丁修正后的等效测量值}}$$

### 2. 速度残差完全体（包含一重积分项标记）

速度残差同样处于欧式空间，直接相减。它的核心测量值 $\Delta \tilde{v}_{ij}(\bar{b})$ 对应的是物理上的**一重积分**（加速度对时间的单次积分）。

$$r_{\Delta v_{ij}} = \underbrace{R_i^T \left( v_j - v_i - g \Delta t_{ij} \right)}_{\text{预测世界：由当前的 } R_i, v_i, v_j \text{ 计算出理论相对速度}} - \underbrace{\left( \overbrace{\Delta \tilde{v}_{ij}(\bar{b})}^{\text{物理本质：}\int (\Delta R \cdot a) dt} + J_{b_a}^{\Delta v} \delta b_a + J_{b_g}^{\Delta v} \delta b_g \right)}_{\text{测量世界：用泰勒展开补丁修正后的等效测量值}}$$

### 3. 旋转残差完全体（李群上的特殊写法）

**注意：** 1. 旋转只依赖陀螺仪，所以它的补丁只有 $\delta b_g$，没有 $\delta b_a$。
2. 旋转的物理本质是角速度对时间的**一重积分**。
3. 由于旋转矩阵不能直接相加减，**“打补丁”必须用右乘 $\text{Exp}$ 映射**，**“算残差（相减）”必须用矩阵转置相乘并套上 $\log$ 映射**。

$$r_{\Delta R_{ij}} = \log \left( \underbrace{\left( \overbrace{\Delta \tilde{R}_{ij}(\bar{b})}^{\text{物理本质：}\int \omega dt} \cdot \text{Exp}(J_{b_g}^{\Delta R} \delta b_g) \right)^T}_{\text{测量世界：修正后的预积分旋转（取转置代表在李群上求逆 / 相减）}} \cdot \underbrace{(R_i^T R_j)}_{\text{预测世界}} \right)^\vee$$

---


在 3D 空间中，状态量、残差和雅可比矩阵的维度会发生质的变化。例如，平移和速度变成了 $3 \times 1$ 的向量，而雅可比矩阵变成了 $3 \times 3$ 的矩阵块。

为了让你看清 3D 空间下的“舒尔补零空间消元”和“预积分打补丁”究竟是怎么用矩阵乘法运算的，我为你设计了这个**极简版的 3D 矩阵推演实例**。

（注：为了避免复杂的李代数 BCH 公式展开导致公式过长，本例假设机器人在 3D 空间中**纯平移**，旋转矩阵始终为单位阵 $I_{3 \times 3}$。但所有平移、速度、偏置和坐标均严格按照 3D 向量和 $3 \times 3$ 雅可比矩阵进行计算。）

---

### 第一阶段：设定 3D 世界的“上帝视角”

设 $\Delta t = 1$ 秒。所有的状态都是 $3 \times 1$ 的列向量。

**1. 物理真实情况（Ground Truth）**
* **起点状态**：$p_1 = [0, 0, 0]^T$, $v_1 = [1, 0, 0]^T$, $R_1 = I_{3 \times 3}$
* **真实运动**：在 X 轴有恒定加速度，真值为 $a_{true} = [2, 0, 0]^T$
* **终点状态**：$p_2 = [2, 0, 0]^T$, $v_2 = [3, 0, 0]^T$, $R_2 = I_{3 \times 3}$
* **真实 IMU 偏置**：加速度计偏置 $b_a = [0.1, 0, 0]^T$
* **真实 3D 路标点 (Landmark)**：$l = [5, 1, 0]^T$

**2. 传感器的实际观测值 (Measurements)**
* **IMU 读数**：$\tilde{a} = a_{true} + b_a = [2.1, 0, 0]^T$
* **3D 相机观测值**（假设使用 RGB-D 或双目算出相对 3D 坐标 $z_i = R_i^T(l - p_i)$）：
    * $t_1$ 时刻观测：$z_1 = [5, 1, 0]^T - [0, 0, 0]^T = [5, 1, 0]^T$
    * $t_2$ 时刻观测：$z_2 = [5, 1, 0]^T - [2, 0, 0]^T = [3, 1, 0]^T$

**3. 优化器的初始猜测（它猜错了！）**
* 固定起点：$p_1 = [0, 0, 0]^T$, $v_1 = [1, 0, 0]^T$
* **错误终点**：$p_2 = [1.8, 0, 0]^T$, $v_2 = [2.8, 0, 0]^T$
* **错误偏置**：$\bar{b}_a = [0, 0, 0]^T$

---

### 第二阶段：前端预积分（3D 向量的打包装车）

前端收到 3D 加速度 $\tilde{a} = [2.1, 0, 0]^T$，使用旧偏置 $\bar{b}_a = [0, 0, 0]^T$ 进行积分：

**1. 测量值常数（$3 \times 1$ 向量）**
* $\Delta \tilde{v} = (\tilde{a} - \bar{b}_a) \Delta t = [2.1, 0, 0]^T$
* $\Delta \tilde{p} = \frac{1}{2}(\tilde{a} - \bar{b}_a) \Delta t^2 = [1.05, 0, 0]^T$

**2. 雅可比常数（$3 \times 3$ 矩阵）**
* 对偏置的雅可比：$J_{b_a}^{\Delta v} = -I_{3 \times 3} \cdot \Delta t = \begin{bmatrix} -1 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & -1 \end{bmatrix} = -I$
* 对偏置的雅可比：$J_{b_a}^{\Delta p} = -0.5 I_{3 \times 3} \cdot \Delta t^2 = -0.5 I$

---

### 第三阶段：3D 无结构视觉残差（零空间消元矩阵推演）

现在进入 GTSAM 后端，处理相机数据。

**1. DLT 三角化求 3D 点**
利用 $p_1$ 的准确位姿和 $z_1$，我们临时推测路标点：
$l_{guess} = p_1 + R_1 z_1 = [0,0,0]^T + [5,1,0]^T = [5, 1, 0]^T$

**2. 计算初始视觉误差向量 ($3 \times 1$)**
公式：$e_i = (R_i^T(l_{guess} - p_i)) - z_i$
* $e_1 = ([5,1,0]^T - [0,0,0]^T) - [5,1,0]^T = [0, 0, 0]^T$
* $e_2 = ([5,1,0]^T - [1.8,0,0]^T) - [3,1,0]^T = [3.2, 1, 0]^T - [3, 1, 0]^T = [0.2, 0, 0]^T$

我们将它们堆叠成一个 $6 \times 1$ 的大误差向量 $e_0 = \begin{bmatrix} e_1 \\ e_2 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0.2 \\ 0 \\ 0 \end{bmatrix}$

**3. 构建 3D 雅可比矩阵并消元**
视觉残差的线性化展开：$e \approx e_0 + F \delta p_2 + E \delta l$
* 残差对路标点 $\delta l$ 的雅可比 ($6 \times 3$ 矩阵)：$E = \begin{bmatrix} I_{3 \times 3} \\ I_{3 \times 3} \end{bmatrix}$
* 残差对位姿 $\delta p_2$ 的雅可比 ($6 \times 3$ 矩阵)：$F = \begin{bmatrix} 0_{3 \times 3} \\ -I_{3 \times 3} \end{bmatrix}$

**核心：寻找左零空间矩阵 $N_{null}^T$**
我们需要一个矩阵乘在 $E$ 左边等于 $0$。显然，一个 $3 \times 6$ 的矩阵可以做到：
$$N_{null}^T = \begin{bmatrix} I_{3 \times 3} & -I_{3 \times 3} \end{bmatrix}$$
验证：$\begin{bmatrix} I & -I \end{bmatrix} \begin{bmatrix} I \\ I \end{bmatrix} = I - I = 0_{3 \times 3}$

两边左乘 $N_{null}^T$：
* **投影后的残差向量 ($3 \times 1$)**：
    $r_{vis} = N_{null}^T e_0 = \begin{bmatrix} I & -I \end{bmatrix} \begin{bmatrix} e_1 \\ e_2 \end{bmatrix} = e_1 - e_2 = [0,0,0]^T - [0.2, 0, 0]^T = [-0.2, 0, 0]^T$
* **投影后的位姿雅可比 ($3 \times 3$)**：
    $H_{vis\_p} = N_{null}^T F = \begin{bmatrix} I & -I \end{bmatrix} \begin{bmatrix} 0 \\ -I \end{bmatrix} = I_{3 \times 3}$

**至此，拥有 3 个自由度的 3D 路标点 $\delta l$ 被完美消灭！**
视觉约束变为了：$I_{3 \times 3} \cdot \delta p_2 = -r_{vis}$

---

### 第四阶段：组装巨大的 3D 矩阵方程

我们要优化的增量是 $9 \times 1$ 的巨大向量：$\Delta \mathcal{X} = \begin{bmatrix} \delta p_2 \\ \delta v_2 \\ \delta b_a \end{bmatrix}$

**1. IMU 速度残差向量 ($3 \times 1$)**
* 当前偏置补丁：$\delta b_a = [0,0,0]^T$
* $r_v = \text{预测} - \text{测量} = (v_2 - v_1) - (\Delta \tilde{v} + J \delta b_a)$
    $= ([2.8, 0, 0]^T - [1, 0, 0]^T) - [2.1, 0, 0]^T = [-0.3, 0, 0]^T$
* 雅可比矩阵块 ($3 \times 3$)：$\frac{\partial r_v}{\partial v_2} = I_{3 \times 3}$，$\frac{\partial r_v}{\partial b_a} = -J_{b_a}^{\Delta v} = I_{3 \times 3}$

**2. IMU 位移残差向量 ($3 \times 1$)**
* $r_p = (p_2 - p_1 - v_1 \Delta t) - (\Delta \tilde{p} + J \delta b_a)$
    $= ([1.8, 0, 0]^T - 0 - [1, 0, 0]^T) - [1.05, 0, 0]^T = [-0.25, 0, 0]^T$
* 雅可比矩阵块 ($3 \times 3$)：$\frac{\partial r_p}{\partial p_2} = I_{3 \times 3}$，$\frac{\partial r_p}{\partial b_a} = -J_{b_a}^{\Delta p} = 0.5 I_{3 \times 3}$

**3. 组装终极 Gauss-Newton 方程：$H \Delta \mathcal{X} = -r$**

我们将所有的 $3 \times 3$ 矩阵块和 $3 \times 1$ 向量块拼成一个 $9 \times 9$ 的大矩阵和 $9 \times 1$ 的大向量：

$$
\underbrace{
\begin{bmatrix}
I_{3 \times 3} & 0_{3 \times 3} & 0_{3 \times 3} \\
0_{3 \times 3} & I_{3 \times 3} & I_{3 \times 3} \\
I_{3 \times 3} & 0_{3 \times 3} & 0.5 I_{3 \times 3}
\end{bmatrix}
}_{9 \times 9 \text{ 雅可比矩阵 } H_{\text{total}}}
\begin{bmatrix}
\delta p_2 \\
\delta v_2 \\
\delta b_a
\end{bmatrix}_{9 \times 1}
=
- \underbrace{
\begin{bmatrix}
r_{vis} \\
r_v \\
r_p
\end{bmatrix}
}_{9 \times 1 \text{ 残差}}
=
\begin{bmatrix}
[0.2, 0, 0]^T \\
[0.3, 0, 0]^T \\
[0.25, 0, 0]^T
\end{bmatrix}
$$

---

### 第五阶段：矩阵求解与 3D 状态更新

虽然这是一个 $9 \times 9$ 的矩阵，但由于它是由清晰的对角块组成的，我们可以直接按块求解（也就是底层 Ceres / GTSAM 调用的 Eigen 库稀疏矩阵求解过程）：

1.  **解第一行块**：$I_{3 \times 3} \cdot \delta p_2 = [0.2, 0, 0]^T \implies \mathbf{\delta p_2 = [0.2, 0, 0]^T}$
2.  **解第三行块**：$I_{3 \times 3} \cdot \delta p_2 + 0.5 I_{3 \times 3} \cdot \delta b_a = [0.25, 0, 0]^T$
    代入 $\delta p_2$，得到 $0.5 \delta b_a = [0.05, 0, 0]^T \implies \mathbf{\delta b_a = [0.1, 0, 0]^T}$
3.  **解第二行块**：$I_{3 \times 3} \cdot \delta v_2 + I_{3 \times 3} \cdot \delta b_a = [0.3, 0, 0]^T$
    代入 $\delta b_a$，得到 $\delta v_2 + [0.1, 0, 0]^T = [0.3, 0, 0]^T \implies \mathbf{\delta v_2 = [0.2, 0, 0]^T}$

**终极 3D 更新：**
把算出的 3D 增量加到错误的初始猜测上：
* $p_2^{new} = [1.8, 0, 0]^T + [0.2, 0, 0]^T = \mathbf{[2.0, 0, 0]^T}$
* $v_2^{new} = [2.8, 0, 0]^T + [0.2, 0, 0]^T = \mathbf{[3.0, 0, 0]^T}$
* $b_a^{new} = [0, 0, 0]^T + [0.1, 0, 0]^T = \mathbf{[0.1, 0, 0]^T}$

**总结**：在真实的 3D SLAM 代码中，矩阵的维度虽然膨胀到了 $9 \times 9$ 甚至成千上万维，但其核心的**分块雅可比、零空间投影 ($N_{null}^T$)**以及**一阶泰勒修正**的逻辑，与这个例子是严丝合缝的完全一致！

## 分布式优化中的“全局”究竟是什么？

$$\min_{X_A,X_B};E_A^{odom}(X_A)+E_A^{intra}(X_A)+E_B^{odom}(X_B)+E_B^{intra}(X_B)+E_{AB}^{inter}(X_A,X_B)$$

$$X_A^{k+1}=\arg\min_{X_A}F(X_A,X_B^k),\quad
X_B^{k+1}=\arg\min_{X_B}F(X_A^{k+1},X_B)$$

$$X_A^{k+1}=\arg\min_{X_A}F(X_A,X_B^k)=\arg\min_{X_A}E_A^{odom}(X_A)+E_A^{intra}(X_A)+E_{AB}^{inter}(X_A,X_B)$$