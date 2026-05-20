---
layout: post
title: 主流多机SLAM框架2——Swarm-LIO2
date: 2026-03-23 14:24:00
description: 【论文阅读】深入理解Swarm-LIO2
tags: 
categories: SLAM Distributed-KF Paper-Reading
chart:
  plotly: true
---

Swarm-LIO2 是近几年一篇非常有代表性的完整的多机协同odometry系统工作，面向多无人机协同工作场景。它并不是一篇从0到1的开创性工作（多机工作大多如此），而是在多篇论文的积累之上叠加组合的方法融合框架。Swarm-LIO2主要涉及以下工作：

### Swarm-LIO2论文：

[1] Zhu F, Ren Y, Yin L, et al. **[Swarm-LIO2: Decentralized efficient LiDAR-inertial odometry for aerial swarm systems](https://ieeexplore.ieee.org/abstract/document/10816004)**[J]. IEEE Transactions on Robotics, 2024, 41: 960-981.

#### 前序工作Swarm-LIO：

[2] Zhu F, Ren Y, Kong F, et al. **[Swarm-LIO: Decentralized Swarm LiDAR-inertial Odometry](https://ieeexplore.ieee.org/abstract/document/10161355)**[C]. 2023 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2023: 3254-3260.

#### 单机框架：

[3] Xu W, Cai Y, He D, et al. **[Fast-lio2: Fast direct lidar-inertial odometry](https://ieeexplore.ieee.org/abstract/document/9697912)**[J]. IEEE Transactions on Robotics, 2022, 38(4): 2053-2073.

#### 滤波理论：

[4] He D, Xu W, Zhang F. **[Symbolic representation and toolkit development of iterated error-state extended kalman filters on manifolds](https://ieeexplore.ieee.org/abstract/document/10024988)**[J]. IEEE Transactions on Industrial Electronics, 2023, 70(12): 12533-12544.

#### 单机框架前序工作：

[5] Xu W, Zhang F. **[Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter](https://ieeexplore.ieee.org/document/9372856)**[J]. IEEE Robotics and Automation Letters, 2021, 6(2): 3317-3324.

#### 时间同步工作：

[6] Watt S T, Achanta S, Abubakari H, et al. **[Understanding and applying precision time protocol](https://ieeexplore.ieee.org/abstract/document/7449285)**[C]. 2015 Saudi Arabia Smart Grid (SASG). IEEE, 2015: 1-7.

其中除了最后的\[6\]，其他工作全部来自于Zhang Fu老师课题组之前发过的文章。

[Video](https://www.youtube.com/watch?v=Q7cJ9iRhlrY)

---

## 1. 背景

单一机器人SLAM/Odometry只需要知道“我在哪里”，但多机系统不够，它还必须知道“队友在哪里”。多机系统之间的区别总结在了下面这篇笔记中：

[主流多机SLAM框架——多机SLAM问题思考与总结](https://effun.xyz/blog/2026/%E4%B8%BB%E6%B5%81%E5%A4%9A%E6%9C%BASLAM%E6%A1%86%E6%9E%B64-%E5%A4%9A%E6%9C%BASLAM%E9%97%AE%E9%A2%98%E6%80%9D%E8%80%83%E6%80%BB%E7%BB%93/)

在我看来，Swarm-LIO2更看重的是自身的状态，队友的状态用于辅助把自己的位置定的更准，这和关注mapping追求全局一致性的系统在导向上是不同的，但由于多机系统都要在初始化阶段估计机器人之间的相对位姿变换，并在后续过程中不断优化更新，因此Swarm-LIO2在某种程度上也能得到一个global坐标系下所有无人机的轨迹。

Swarm-LIO2没有针对一个具体的应用场景展开，而是基于以下四类现有方法存在的问题进行设计：

* **GPS / RTK-GPS 只适合部分室外环境**：传统室外无人机可以用 GPS 或 RTK-GPS 做定位，但问题是很多 swarm 任务发生在 GPS-denied environments，比如室内、树林、隧道、灾害现场、城市峡谷等。在这些场景里 GPS 不可靠甚至不可用。论文因此把 GPS-denied 场景作为一个重要动机。

* **Motion capture / anchor-based UWB 精度高，但不适合真实部署**:室内可以用 motion capture，或者 anchor-based UWB，但是它们依赖外部固定基础设施，比如动捕相机、UWB anchor、地面站等。这样系统就变成了中心化系统，安装复杂，而且容易有 single point of failure。对真实无人机集群任务来说，这种方式不够自主，也不够鲁棒。

* **视觉方案轻量，但深度和鲁棒性不足**: 相机便宜、轻、小，而且有丰富纹理信息，所以很多多机系统用 VIO 或视觉检测队友。但它的问题是：低光照、纹理弱、运动模糊时容易退化。

* **多机器人 SLAM 通信和计算太重**: 很多多机器人 SLAM 依赖地图、子图、描述子、回环检测、点云配准等信息交换。对地面机器人还可以接受，但对空中无人机来说，载荷、算力、续航和通信带宽都很有限。空中平台则更需要 lightweight sensors、efficient algorithms 和 low-bandwidth communication。

### 解决的具体问题：

前面说了这么多都是一些宏观的内容，现在我们来看它到底解决了哪些具体的问题。Swarm-LIO2一开始就把问题分成两类：

* ego state estimation：每架无人机估计自己的位姿、速度等状态
* mutual state estimation：每架无人机估计其他队友相对于自己的状态

其中对队友的状态估计又涉及到**如何发现队友**、**发现后如何知道这是哪一架无人机**、**不同无人机之间如何时间同步**、**每架无人机如何估计队友位置，但又不让状态维度爆炸**这几个问题。

## 2.创新点总结

Swarm-LIO2 的核心贡献是提出了一个完整的去中心化多机 LiDAR-inertial 状态估计框架。它首先通过 ad hoc 网络和 PTP-like 时间校准实现队友发现与时间对齐；然后利用反光贴、LiDAR reflectivity 检测和 trajectory matching 完成队友识别与外参初始化；进一步通过 factor graph 推断未直接观测队友的 global extrinsic，从而提升 plug-and-play 初始化效率。

在状态估计阶段，它将 IMU、LiDAR 点云和主动/被动 mutual observation 统一放入 ESIKF（误差状态不变卡尔曼滤波） 中紧耦合优化，并通过 temporal compensation 保证异步多机观测的一致性。为了支持大规模集群，它提出 marginalization 策略降低状态维度，并结合 LiDAR degeneration evaluation 在点云退化时利用队友互观测增强定位鲁棒性。

* **基于因子图优化高效地标定与队友之间的外参**：很大程度上降低了群体初始化的复杂度和能耗。用N个AAV初始化一个swarm所需的初始化飞行次数从O(N)减少到O(1)。

* **把 LiDAR、IMU、互观测一起放进 ESIKF 中紧耦合估计**：Swarm-LIO2 的状态估计继承 FAST-LIO2/IKFoM 的单机 LIO 框架，但扩展到了多机。每架 AAV 的状态主要包括：自己的 pose、速度、IMU bias、重力加速度以及与队友 global frame 之间的 global extrinsic。它融合IMU measurements、LiDAR point cloud measurements和mutual observation measurements（本机看到队友，或队友看到本机，作为额外几何约束）。

* **mutual observation 的严谨建模和时间补偿**：Swarm-LIO2 中的 mutual observation 有两种。首先是主动观测（active observation）AAV i 用自己的 LiDAR 看到 AAV j，也就是“我看到队友”。然后是被动观测（passive observation）AAV j 用它的 LiDAR 看到 AAV i，然后把这个观测发给 i。也就是“队友看到我，并告诉我”。这两类观测都被建模成 measurement residual，进入 ESIKF 更新。但多机系统存在时间错位：AAV i 的 LiDAR scan end time、AAV j 的状态时间戳、网络传输延迟都可能不同。因此 Swarm-LIO2 加入了 temporal compensation，用 constant velocity model 把队友状态或本机状态补偿到统一时间。因为如果不做时间补偿，互观测约束其实是在错误时间上建立的，会破坏滤波器一致性。

* **冗余状态信息边缘化提升可扩展性**：如果每架 AAV 都把所有队友的 global extrinsic 都放进 ESIKF 状态里，那么随着队友数量增加，状态维度会增长，滤波更新的计算复杂度会急剧上升。当前 scan 中没有被观测到、也没有提供有效互观测的队友外参状态，可以被 marginalize 掉，只把必要信息以先验或扩展测量噪声形式保留下来。该 marginalization 策略可以把状态估计复杂度增长从 cubic 降到 sublinear。

* **LiDAR 退化估计与模式切换**：Swarm-LIO2 还加入了 LiDAR 退化检测。它通过分析 LiDAR 点云残差对 ego pose 的 Jacobian 奇异值，判断当前点云是否足以约束完整位姿。如果 LiDAR 没有退化，主要用点云约束优化 ego state，同时 refine global extrinsic。如果 LiDAR 退化，系统更依赖 global extrinsic 和 mutual observation 来帮助确定 ego state。即非退化时用 LiDAR refine 外参；退化时用队友互观测约束支撑定位。

其实ESIKF在之前的Swarm-LIO中已经应用了，这里我把它总结为Swarm-LIO2的创新点一起学习一下。

## 3.方法

符号定义（懒得写了，直接截图）

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260323/2.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    系统符号定义
</div>

### 3.1 System overview

现在我们首先考虑一个核心问题：多机状态估计到底怎么建模？

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260323/1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    系统建模
</div>

Swarm-LIO2首先考虑一个由 $N$ 架 AAV 组成的空中集群，每架 AAV 都搭载 LiDAR 和 IMU。每架 AAV 要做两件事：

第一，估计自己的状态，也就是 **ego state estimation**。  
第二，估计其他队友的状态，也就是 **mutual state estimation**。

但是，如果让每一架 AAV 都在自己的滤波器里直接估计所有队友的完整状态，状态维度会非常高，计算量也会很大。因此 Swarm-LIO2 没有采用“每架无人机估计所有无人机完整状态”的方式，而是采用一个更轻量的思路：

> 每架 AAV 主要估计自己的 ego state，并广播出去；其他 AAV 收到后，通过两机 global frame 之间的外参 $${}^{G_i}\mathbf{T}_{G_j}$$，把队友的 ego state 投影到自己的 global frame 中，从而得到 mutual state。

这就是 Section III 最重要的建模思想。

换句话说，Swarm-LIO2 不是让 AAV $i$ 直接完整估计 AAV $j$ 的所有状态，而是：

$$
\text{AAV } j \text{ 自己估计自己}
\xrightarrow{\text{broadcast}}
\text{AAV } i \text{ 接收 } j \text{ 的 ego state}
\xrightarrow{\,{}^{G_i}\mathbf{T}_{G_j}\,}
\text{投影到 } G_i \text{ 下}
$$

所以，**mutual state estimation 的关键就变成了 global extrinsic calibration**。



所以说对于Swarm-LIO2来说，全局外参是其核心内容，但这里要区分一点：**所谓 global extrinsic 并不是所有无人机相对于某一个公共世界坐标系的外参，而是不同 AAV 自己的 global reference frame 之间的相对变换。论文里说这个 global frame 通常是该 AAV 的第一个 IMU frame。**所以不能理解为所有无人机有一个共同的全局外参。

Swarm-LIO2是一个滤波框架，这里我提前给出他的状态向量：

  $$\mathbf{x}_{i} = \left[{}^{G_i}R_{b_i}^T, {}^{G_i}p_{b_i}^T, {}^{G_i}v_{b_i}^T, b_{g_i}^T, b_{a_i}^T, {}^{G_i}g^T, \cdots, {}^{G_i}R_{G_j}^T, {}^{G_i}p_{G_j}^T, \cdots \right]^T$$

从这里我们也能看出，全局外参也是状态向量中的一组待估计状态，对每一个队友 j，都会增加一个 6 维外参状态块，整体状态维度是 18+6(N−1)。

下图是
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260415/11.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    SwarmLIO2 系统框架
</div>

蓝色部分是初始化模块，包含三个子模块。

第一，New Teammate Monitoring on Network & Temporal Calibration。这个模块通过 ad hoc network 监听新出现的队友。每架 AAV 会广播自己的 identity/time information。收到新的队友信息后，系统会估计两者之间的 temporal offset。（可能没有直接观测但有通信）。

第二，New Teammate Detection From LiDAR Observations & Extrinsic Calibration。这个模块从 LiDAR 点云中检测新队友。检测依赖 reflective tape 的高反射特性。检测到候选目标后，系统会跟踪它的轨迹，并通过 trajectory matching 识别它是哪架 AAV，同时得到两机 global frame 之间的初始外参。

第三，FGO-based Extrinsic Calibration of Non-Directly Observed Teammates。这是 Swarm-LIO2 比 Swarm-LIO 更进一步的地方。即使 AAV i 没有直接在 LiDAR 中看到 AAV j，只要网络里收到了其他 AAV 之间的 global extrinsic 约束，就可以通过 factor graph optimization 推断出 $${}^{G_i}\mathbf{T}_{G_j}$$ 。就是说无人机不一定要亲眼看到所有队友，只要整个外参约束图连通，我也能标定和未直接观测队友之间的 global extrinsic。

当某个队友 j 的 global extrinsic $${}^{G_i}\mathbf{T}_{G_j}$$被标定后，论文说它就会被认为是一个 valid teammate，并且它的相关状态会加入到后续 state estimation 模块中继续估计和 refine。。

绿色部分是状态估计模块。它的状态包括两部分：ego state+global extrinsics



##### Swarm-LIO2的总体流程可以总结如下：

网络发现队友→时间同步→LiDAR 观测队友→身份识别→global extrinsic 初始化→加入 ESIKF 状态→融合 LiDAR/IMU/互观测→更新 ego state 与 global extrinsic→广播给其他 AAV





<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260415/17.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    基于雷达反射率的目标检测
</div>

