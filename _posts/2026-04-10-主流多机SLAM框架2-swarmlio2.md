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

**每架 AAV 主要估计自己的 ego state，并广播出去；其他 AAV 收到后，通过两机 global frame 之间的外参 $${}^{G_i}\mathbf{T}_{G_j}$$，把队友的 ego state 投影到自己的 global frame 中，从而得到 mutual state。**

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

##### 蓝色部分是初始化模块，包含三个子模块。

第一，New Teammate Monitoring on Network & Temporal Calibration。这个模块通过 ad hoc network 监听新出现的队友。每架 AAV 会广播自己的 identity/time information。收到新的队友信息后，系统会估计两者之间的 temporal offset。（可能没有直接观测但有通信）。

第二，New Teammate Detection From LiDAR Observations & Extrinsic Calibration。这个模块从 LiDAR 点云中检测新队友。检测依赖 reflective tape 的高反射特性。检测到候选目标后，系统会跟踪它的轨迹，并通过 trajectory matching 识别它是哪架 AAV，同时得到两机 global frame 之间的初始外参。

第三，FGO-based Extrinsic Calibration of Non-Directly Observed Teammates。这是 Swarm-LIO2 比 Swarm-LIO 更进一步的地方。即使 AAV i 没有直接在 LiDAR 中看到 AAV j，只要网络里收到了其他 AAV 之间的 global extrinsic 约束，就可以通过 factor graph optimization 推断出 $${}^{G_i}\mathbf{T}_{G_j}$$ 。就是说无人机不一定要亲眼看到所有队友，只要整个外参约束图连通，我也能标定和未直接观测队友之间的 global extrinsic。

当某个队友 j 的 global extrinsic $${}^{G_i}\mathbf{T}_{G_j}$$ 被标定后，论文说它就会被认为是一个 valid teammate，本机状态向量中会加入与该队友相关的 global extrinsic 状态块，并在后续 ESIKF 中继续在线 refinement；但队友的完整 ego state 并不会直接作为本机滤波状态估计而是通过队友广播的 ego state 和 $${}^{G_i}\mathbf{T}_{G_j}$$ 投影得到。

##### 绿色部分是状态估计模块。

它的状态包括两部分：ego state + global extrinsics。当新队友完成初始化后，对应的 global extrinsic 会被加入状态估计器，作为初始值。State Prediction利用 IMU 进行状态预测。这是 ESIKF 的 prediction step。Marginalization为了降低状态维度，Swarm-LIO2 会对当前没有参与观测的 teammate extrinsic 做边缘化。这是它保持大规模集群实时性的关键之一。Degeneration Evaluation系统会判断当前 LiDAR 观测是否退化。如果 LiDAR 退化，说明点云本身不足以约束完整 ego state，此时要更依赖 global extrinsic 和 mutual observation。Modeling of Measurement构建所有测量残差。Temporal Compensation由于队友状态、互观测和本机状态可能来自不同时间戳，所以要用 temporal offset 和运动模型做时间补偿。State Update (ESIKF)经过测量建模和时间补偿后，所有约束进入 ESIKF 进行迭代更新，得到更新后的 ego state 和 global extrinsic。Covariance Re-initialization更新结束后，需要对协方差进行重新初始化，以进入下一轮滤波。

Swarm-LIO2 的状态估计器本质上是一个扩展后的 FAST-LIO2/IKFoM 滤波器，只不过状态里额外包含了 global extrinsic，并且 measurement 里额外加入了 active/passive mutual observation。

##### 右侧是通信模块

Swarm-LIO2 通过 fully decentralized ad hoc network 交换信息。论文明确说，它使用 IEEE 802.11 架构下的 IBSS 模式，这种网络可以由常见 Wi-Fi 模块支持。

##### Swarm-LIO2的总体流程可以总结如下：

网络发现队友→时间同步→LiDAR 观测队友→身份识别→global extrinsic 初始化→加入 ESIKF 状态→融合 LiDAR/IMU/互观测→更新 ego state 与 global extrinsic→广播给其他 AAV

### 3.2 Online initialization

Section IV 解决的是 Swarm-LIO2 的 online initialization 问题。由于每架 AAV 都在自己的 global frame 中估计 ego state，系统必须在状态估计前自动完成三件事：第一，通过 ad hoc network heartbeat 发现队友，并基于 PTP peer-delay 机制估计 pairwise temporal offset；第二，通过 LiDAR reflectivity filtering 和 Euclidean clustering 检测贴有 reflective tape 的候选 AAV，再用 temporary tracker 积累轨迹，并通过 trajectory matching 同网络中收到的 teammate trajectory 进行匹配，从而识别队友身份并求得直接观测队友的 global extrinsic；第三，将所有直接标定得到的 pairwise global extrinsic 作为 factor graph 的边，以各 AAV global frame 为节点，通过 fixing self frame 和 iSAM2 优化，推断未直接观测队友的 global extrinsic。这样 Swarm-LIO2 就实现了 decentralized、plug-and-play 的初始化，并且相比 Swarm-LIO 显著降低了大规模集群初始化复杂度。


#### A. 新队友发现（网络）与时间标定

多机系统中，不同 AAV 的电脑时钟不是天然同步的。AAV $i$ 收到 AAV $j$ 发来的 ego state 或 mutual observation 时，这些数据的时间戳是由 AAV $j$ 的本地时钟打上的。如果直接拿来用，就会出现时间错位。

所以 AAV $i$ 必须知道：

$$
{}^i\tau_j
$$

也就是 AAV $j$ 的时钟相对于 AAV $i$ 的 temporal offset。

这个量后面在 Section V 的 temporal compensation 中会直接用到。比如论文后面 active observation 的补偿项是：

$$
t_{i,k} - t_{j,k} + {}^i\tau_j
$$

这说明 ${}^i\tau_j$ 的作用是把 AAV $j$ 的时间戳转换到 AAV $i$ 的时间轴下。

**网络发现 heartbeat packet**： 每架 AAV 都会在 ad hoc network 里以固定频率广播自己的 identity information，也就是 heartbeat packet。里面主要包含：

- AAV ID;
- IP address;
- time information.

AAV $i$ 收到某个新 AAV 的 heartbeat 后，就把它加入自己的 teammate list。这个 teammate 会有两种状态：

$$
\text{connected}
\quad \text{or} \quad
\text{disconnected}
$$

如果 2 秒内没有收到某个 teammate 的 heartbeat，则把它设为 disconnected；如果之后重新收到 heartbeat，则恢复为 connected。这个设计是为了支持 plug-and-play，也就是队友可以中途加入，也可以短时掉线后重新回来。论文明确说初始化模块通过 heartbeat 维护 teammate list，并在发现新 teammate 后进行 temporal calibration。

**基于 PTP peer-delay 机制标定时间偏差：**Swarm-LIO2 使用的是类似 Precision Time Protocol, PTP 中 peer-delay mechanism 的思想。PTP 本质上是通过消息往返时间戳，估计两个设备之间的 clock offset 和 path delay。PTP 论文中也说明，peer-delay 机制用于测量 peer-to-peer delay，不需要所有 slave 都向 grandmaster 请求 end-to-end delay，因此更适合可扩展网络。

Swarm-LIO2 并不是建立一个完整的 PTP master-slave 全局时钟系统，而是借用了它的 pairwise request-response 思路：每一对 AAV 之间单独估计 temporal offset。因此它仍然是 decentralized 的，不需要指定 master clock。

那么如何由四个时间戳估计 offset? 数学推导如下：

考虑 AAV $i$ 和 AAV $j$。假设 AAV $i$ 向 AAV $j$ 发送 request，AAV $j$ 回复 response。我们定义四个时间戳：

$$
t_1^i：\text{AAV } i \text{ 发送 request 的本地时间}
$$

$$
t_2^j：\text{AAV } j \text{ 接收 request 的本地时间}
$$

$$
t_3^j：\text{AAV } j \text{ 发送 response 的本地时间}
$$

$$
t_4^i：\text{AAV } i \text{ 接收 response 的本地时间}
$$

设两机之间的单向网络延迟为 $d$，并假设短时间内双向延迟近似对称。设 AAV $j$ 的时钟相对 AAV $i$ 的时钟偏移为：

$$
{}^i\tau_j = t^j - t^i
$$

也就是说，如果同一个真实时刻下，AAV $j$ 的本地时间比 AAV $i$ 大，那么 ${}^i\tau_j > 0$。

于是有：

$$
t_2^j = t_1^i + d + {}^i\tau_j
$$

因为 AAV $i$ 在 $t_1^i$ 发出消息，经过网络延迟 $d$，到达 AAV $j$。到达时如果换成 AAV $j$ 的时钟，需要加上 offset。

同理，response 从 AAV $j$ 发出后到达 AAV $i$：

$$
t_4^i = t_3^j + d - {}^i\tau_j
$$

因为 $t_3^j$ 是 AAV $j$ 的时间，转换回 AAV $i$ 的时间要减去 ${}^i\tau_j$，再加上传输延迟 $d$。

整理两个式子：

$$
t_2^j - t_1^i = d + {}^i\tau_j
$$

$$
t_4^i - t_3^j = d - {}^i\tau_j
$$

两式相加得到 path delay：

$$
\hat{d}
=
\frac{
(t_2^j - t_1^i) + (t_4^i - t_3^j)
}{2}
$$

两式相减得到 temporal offset：

$$
{}^i\hat{\tau}_j
=
\frac{
(t_2^j - t_1^i) - (t_4^i - t_3^j)
}{2}
$$

这和 PTP 中 offset/path delay 的思想是一致的。PTP 论文中也给出 offset 可以由 master-slave 消息时间戳和 path delay 计算，path delay 则由双向消息时间差估计。

Swarm-LIO2 中为了降低随机误差和网络抖动的影响，会重复这个 request-response 过程 30 次，然后取平均值作为最终的 ${}^i\tau_j$。论文指出，典型 AAV 飞行时间内 clock drift 可以忽略，所以通常只需要标定一次；如果 clock drift 明显，也可以固定频率重新估计，例如 1 Hz。

最后，估计出的 ${}^i\tau_j$ 会存到 Hash table 中：

$$
\text{key : AAV ID,}
\qquad
\text{value : } {}^i\tau_j
$$

之后 AAV $i$ 收到 AAV $j$ 的任意数据，就可以快速查表修正时间戳。


#### B. 直接lidar观测下的队友检测与外参标定

上面解决“网络中有谁”和“时钟差多少”，但它不知道：

1. LiDAR 里看到的某个物体是不是队友
2. 如果是队友，它是哪一架 AAV；
3. AAV j 的 global frame 和 AAV i 的 global frame之间的变换 $${}^{G_i}\mathbf{T}_{G_j}$$ 是多少。


<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260415/17.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    基于雷达反射率的目标检测
</div>

**Step 1：点云预处理与高反射点提取**：

AAV $i$ 收到一帧 LiDAR scan 后，首先对原始点云做 undistortion。这里和 FAST-LIO2 的思想有关：LiDAR 一帧 scan 中的点不是同一时刻采集的，如果无人机在运动，点云会有 motion distortion。因此需要利用 IMU 进行运动补偿，把点投影到 scan end time。FAST-LIO2 中就把 motion compensation/back-propagation 作为 LiDAR-inertial odometry 的关键步骤。

设去畸变后的点云为：

$$
{}^{b_i}\mathbf{P}
$$

其中 $b_i$ 是 AAV $i$ 当前 body frame。

由于每架 AAV 上贴了 reflective tapes，这些点在 LiDAR 中会有较高反射强度。因此系统进行 reflectivity filtering：

$$
{}^{b_i}\mathbf{P}_h
=
\left\{
{}^{b_i}\mathbf{p}_n \in {}^{b_i}\mathbf{P}
\mid
\rho({}^{b_i}\mathbf{p}_n) > \rho_{\mathrm{thr}}
\right\}
$$

其中：

- $\rho({}^{b_i}\mathbf{p}_n)$ 是 LiDAR 点的 reflectivity；
- $\rho_{\mathrm{thr}}$ 是预设阈值；
- ${}^{b_i}\mathbf{P}_h$ 是高反射点集合。

论文中 Algorithm 1 第一行就是：

$$
{}^{b_i}\mathbf{P}_h
=
\mathrm{ReflectivityFiltering}({}^{b_i}\mathbf{P})
$$

这个设计的优点是非常直接：不需要训练视觉检测网络，也不依赖光照条件。

**Step 2：高反射点聚类，得到候选队友位置**：

接下来对高反射点做 Euclidean clustering：

$$
{}^{b_i}\breve{\mathbf{p}}_m
=
\mathrm{FastEuclideanClustering}({}^{b_i}\mathbf{P}_h)
$$

每个 cluster 的 centroid 被认为是一个候选 AAV 位置：

$$
{}^{b_i}\breve{\mathbf{p}}_m
$$

这里的 $\breve{}$ 表示 measured quantity，也就是测量值。

需要注意，高反射点不一定全是队友，也可能是环境中其他反光物体。因此这里只是 potential teammate AAV，还不能直接加入状态估计。

Swarm-LIO 的前序工作里也提到，反射强度会受距离和激光入射角影响，所以高反射点可能不能完整覆盖无人机本体，因此需要结合 tracker 和 predicted region 进一步提高聚类稳定性。

**Step 3： temporary tracker 临时跟踪候选目标**：

对于每个候选目标，Swarm-LIO2 使用一个 Kalman-filter-based temporary tracker 进行跟踪，假设目标近似满足 constant velocity model。

可以把 temporary tracker 的状态写成：

$$
\mathbf{s}_{m,k}
=
\begin{bmatrix}
{}^{G_i}\mathbf{p}_{m,k} \\
{}^{G_i}\mathbf{v}_{m,k}
\end{bmatrix}
$$

其中：

- ${}^{G_i}\mathbf{p}_{m,k}$ 是第 $m$ 个候选目标在 AAV $i$ 的 global frame 下的位置；
- ${}^{G_i}\mathbf{v}_{m,k}$ 是速度。

由于 LiDAR 聚类得到的是 body frame 下的位置 ${}^{b_i}\breve{\mathbf{p}}_{m,k}$，需要先转换到 AAV $i$ 的 global frame：

$$
{}^{G_i}\breve{\mathbf{p}}_{m,k}
=
{}^{G_i}\mathbf{T}_{b_i,k}
\circ
{}^{b_i}\breve{\mathbf{p}}_{m,k}
$$

其中论文使用 $\circ$ 表示刚体变换：

$$
T \circ p = Rp + t
$$

temporary tracker 的预测模型可以写成：

$$
{}^{G_i}\hat{\mathbf{p}}_{m,k}
=
{}^{G_i}\mathbf{p}_{m,k-1}
+
{}^{G_i}\mathbf{v}_{m,k-1}\Delta t
$$

$$
{}^{G_i}\hat{\mathbf{v}}_{m,k}
=
{}^{G_i}\mathbf{v}_{m,k-1}
$$

矩阵形式为：

$$
\mathbf{s}_{m,k}
=
\mathbf{F}\mathbf{s}_{m,k-1}
+
\mathbf{w}_k
$$

$$
\mathbf{F}
=
\begin{bmatrix}
I_3 & \Delta t I_3 \\
0 & I_3
\end{bmatrix}
$$

测量模型为：

$$
\mathbf{z}_{m,k}
=
\mathbf{H}\mathbf{s}_{m,k}
+
\mathbf{n}_k
$$

$$
\mathbf{z}_{m,k}
=
{}^{G_i}\breve{\mathbf{p}}_{m,k},
\qquad
\mathbf{H}
=
\begin{bmatrix}
I_3 & 0
\end{bmatrix}
$$

因此可以用标准 Kalman filter 更新 temporary tracker。

这一步的作用不是最终估计队友状态，而是积累一段候选目标轨迹：

$$
{}^{G_i}\mathcal{T}_m
=
\left\{
{}^{G_i}\breve{\mathbf{p}}_{m,1},
{}^{G_i}\breve{\mathbf{p}}_{m,2},
\cdots,
{}^{G_i}\breve{\mathbf{p}}_{m,K}
\right\}
$$

这条轨迹后面用于和网络中收到的各个 AAV ego trajectory 做匹配。

**Step 4： trajectory matching 识别队友并求 global extrinsic**：

现在 AAV $i$ 有一条 LiDAR 观测得到的目标轨迹：

$$
{}^{G_i}\mathcal{T}_m
=
\left\{
{}^{G_i}\breve{\mathbf{p}}_{m,k}
\right\}_{k=1}^{K}
$$

同时，AAV $i$ 会从网络中收到每个 teammate $j$ 广播的 ego trajectory：

$$
{}^{G_j}\mathcal{T}_j
=
\left\{
{}^{G_j}\breve{\mathbf{p}}_{b_j,k}
\right\}_{k=1}^{K}
$$

如果这个 temporary tracker 实际上跟踪的是 AAV $j$，那么应该存在一个 rigid transformation：

$$
{}^{G_i}\mathbf{T}_{G_j}
=
\left(
{}^{G_i}\mathbf{R}_{G_j},
{}^{G_i}\mathbf{p}_{G_j}
\right)
$$

使得：

$$
{}^{G_i}\breve{\mathbf{p}}_{m,k}
\approx
{}^{G_i}\mathbf{T}_{G_j}
\circ
{}^{G_j}\breve{\mathbf{p}}_{b_j,k}
$$

也就是：

$$
{}^{G_i}\breve{\mathbf{p}}_{m,k}
\approx
{}^{G_i}\mathbf{R}_{G_j}
{}^{G_j}\breve{\mathbf{p}}_{b_j,k}
+
{}^{G_i}\mathbf{p}_{G_j}
$$

因此 trajectory matching 求解下面的 least-squares problem：

$$
\min_{{}^{G_i}\mathbf{R}_{G_j}, {}^{G_i}\mathbf{p}_{G_j}}
\sum_{k=1}^{K}
\frac{1}{2}
\left\|
{}^{G_i}\breve{\mathbf{p}}_{m,k}
-
\left(
{}^{G_i}\mathbf{R}_{G_j}
{}^{G_j}\breve{\mathbf{p}}_{b_j,k}
+
{}^{G_i}\mathbf{p}_{G_j}
\right)
\right\|^2
$$

这就是论文 Eq. (1) 的本质。论文也说明，只选取时间戳接近的数据对参与匹配，并且最近 $K$ 个位置组成 sliding window，避免通信丢包和计算量过大。

下面推导trajectory matching 的闭式解

令：

$$
p_k
=
{}^{G_i}\breve{\mathbf{p}}_{m,k}
$$

$$
q_k
=
{}^{G_j}\breve{\mathbf{p}}_{b_j,k}
$$

要求：

$$
p_k
\approx
Rq_k + t
$$

目标函数：

$$
J(R,t)
=
\sum_{k=1}^{K}
\frac{1}{2}
\left\|
p_k - Rq_k - t
\right\|^2
$$

其中：

$$
R \in SO(3),
\qquad
t \in \mathbb{R}^3
$$

先对 $t$ 求最优解。令：

$$
\bar{p}
=
\frac{1}{K}
\sum_{k=1}^{K}
p_k,
\qquad
\bar{q}
=
\frac{1}{K}
\sum_{k=1}^{K}
q_k
$$

对 $t$ 求导：

$$
\frac{\partial J}{\partial t}
=
-
\sum_{k=1}^{K}
\left(
p_k - Rq_k - t
\right)
=
0
$$


#### C. 基于factor graph的

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260323/3.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    外参初始化因子图
</div>





### 3.3 分布式状态估计


Swarm-LIO2 使用的是流形上的误差状态迭代卡尔曼滤波。它并不是直接在欧氏空间中对完整状态做普通加法更新，而是维护一个位于流形上的名义状态 $\hat{x}$，同时用误差状态 $$\delta x = x \boxminus \hat{x}$$ 描述真实状态相对于名义状态的局部小扰动。对于旋转 $R \in SO(3)$，误差写成 $$R = \hat{R}\mathrm{Exp}(\delta \theta)$$，因此滤波器只需要在三维小角度误差 $\delta \theta$ 上做线性化和协方差传播，而不会破坏旋转矩阵约束，也避免了对 9 个冗余旋转矩阵元素直接求导。协方差 $P$ 描述的是误差状态的不确定性，例如旋转部分的协方差描述的是 $\delta \theta$ 的不确定性，而不是旋转矩阵 $R$ 本身的协方差。

在状态预测阶段，Swarm-LIO2 采用 IKFoM 的 canonical form:

$$
x_{i,\tau+1}
=
x_{i,\tau}
\boxplus
\left(
\Delta t_\tau f_i(x_{i,\tau},u_{i,\tau},w_{i,\tau})
\right)
$$

这里的 $\Delta t_\tau f_i$ 是由 IMU 动力学产生的状态传播增量，而不是误差状态。实际预测名义状态时令过程噪声为零：

$$
\hat{x}_{i,\tau+1}
=
\hat{x}_{i,\tau}
\boxplus
\left(
\Delta t_\tau f_i(\hat{x}_{i,\tau},u_{i,\tau},0)
\right)
$$

然后通过定义真实状态与名义状态之间的误差 $$\delta x = x \boxminus \hat{x}$$，推导误差传播函数，并在 $$\delta x = 0, w = 0$$ 附近线性化得到：

$$
\delta x_{\tau+1}
\approx
F_x\delta x_\tau + F_w w_\tau
$$

从而传播协方差：

$$
P_{\tau+1}
=
F_x P_\tau F_x^T + F_w Q F_w^T
$$

在测量更新阶段，LiDAR 点云残差、active mutual observation 和 passive mutual observation 都被线性化为：

$$
r
\approx
H\delta x + Dv
$$

prediction prior 和 measurement residual 共同构成 MAP 问题，求解得到误差修正量 $\widehat{\delta x}$，再通过 $\hat{x} \leftarrow \hat{x} \boxplus \widehat{\delta x}$ 注入名义状态。由于该过程是 iterated 的，系统会在更新后的名义状态附近重新线性化测量模型，从而减小非线性测量带来的线性化误差。