---
layout: post
title: 多机器人协同感知1——问题建模
date: 2026-03-20 14:24:00
description: 综合阅读一些相关论文，搞清楚多机分布式优化问题的本质，记录和老师讨论的一些比较关键的点（入门记录o((>ω< ))o）
tags: 
categories: Distributed-Optimization SLAM Paper-Reading
chart:
  plotly: true
---

## 1. 基于滤波or基于平滑？

首先感叹一下《视觉SLAM 14讲》这本书真的常看常新，上次阅读这本书可能已经是2年之前了，但回过头来发现那时我并未真正理解SLAM后端，也不理解滤波方法和优化方法的区别到底是什么。在阅读多机协同SLAM的论文时我终于有点感受到两种方法之间的差异，我觉得从简单来说，滤波就算一步，在多机通信受限的时候能够提供快速的状态估计，[Swarm-LIO2](https://ieeexplore.ieee.org/document/10816004)在多机交互的时候使用了这种思想。滤波的方法遵循马尔可夫性，认为当前状态只与前一/几个时刻的状态相关，而与其他时刻的状态与观测都无关。而优化的方法则综合利用了所有时刻的信息，即使状态变化较大也更加可靠，虽然计算开销变大了，但优化方法依然更适用于大型场景。

以下是今天翻阅14讲时读到的一个小节，以前在读的时候由于知识浅薄不能明白这一节的意义，现在看来当时错过了很多这种细节，惭愧惭愧。
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260320/1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    
</div>


## 2. Landmark&系统一致性

下面这张图表示多个机器人的位姿图和landmark：
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260320/3.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    pose graph & landmark 示意图
</div>

这里衍生出几个问题：什么是landmark？landmark在单机和多机系统中的作用分别是什么，对应的formulate的问题又是什么？对于包含全局地图（Mesh或点云）来说，是否需要优化landmark，以及如何保证不同机器人看到的landmark的一致性以及构建的全局地图的一致性。

### 2.1 什么是 Landmark？

在 SLAM 系统中，理解 Landmark（路标/特征点）需要将其在物理前端与数学后端剥离开来看。

* **物理视角（前端锚点）：** Landmark 是现实环境中的静止、显著且能被重复观测的物理特征。不考虑线特征或语义信息的时候，landmark就是前端提取的特征点对应的3D空间点。在视觉 SLAM 中，它们是被提取出的角点、边缘（如 ORB、FAST 特征点），甚至可以是高阶的线、面或语义物体。前端的核心任务是通过描述符匹配和几何校验，完成**数据关联（Data Association）**，即认出“不同视角下看到的是同一个物理锚点”。
* **数学视角（后端变量）：** 一旦进入优化器，物理特征就变成了纯粹的数学变量 $l_j \in \mathbb{R}^3$。它们在因子图中作为独立的节点，与机器人的位姿节点 $x_i$ 相连，构成了约束网络的基础。

### 2.2 Landmark 在单机与多机系统中的作用及问题建模 (Formulation)

Landmark 在单机和多机系统中的处理方式不同，这直接反映在优化问题的数学建模上：

#### 2.2.1 单机系统：联合优化 (Joint Optimization)
在以 ORB-SLAM 或 VIO（如 VINS-Fusion 的局部滑动窗口）为代表的高精度单机模块中，位姿和 Landmark 通常会被**联合优化（Bundle Adjustment, BA）**。

* ORB-SLAM2 的做法：无论是局部建图线程（Local BA），还是全局回环线程（Global BA），都在同时优化相机位姿（Pose）和 3D 地图点（Landmark）。这就导致它精度极高，但也使得它的计算复杂度非常庞大。

* VINS-Fusion 的做法：在它的前端和 VIO 滑动窗口阶段（Estimator）：VINS-Fusion 是一个紧耦合框架。它在滑动窗口内，不仅优化位姿、速度、IMU零偏，而且显式地优化了Landmark。 只不过它为了数值稳定性，没有直接优化 Landmark 的 (X,Y,Z)，而是优化了 Landmark 在相机坐标系下的逆深度。在它的全局回环阶段（Pose Graph Thread）为了保证实时的计算效率，VINS-Fusion 在检测到回环后，丢弃了所有的 Landmark，仅仅构建了一个只包含位姿的4自由度的位姿图进行优化。

* **作用：** 显式地参与地图构建，通过重投影误差不断修正自身的 3D 坐标，同时拉扯相机的位姿。
* **数学建模：** 这是一个标准的无约束非线性最小二乘问题（假设不考虑零空间固定）：

  $$\min_{\mathcal{X}, \mathcal{L}} \sum_{(i,j)} \| z_{ij} - h(x_i, l_j) \|_{\Sigma}^2$$

  其中，$\mathcal{X}$ 是所有位姿节点，$ \mathcal{L}$ 是所有 Landmark 节点，$z_{ij}$ 是观测值（如像素坐标），$h(\cdot)$ 是投影函数。这种模型精度极高，但矩阵维度随 Landmark 数量呈爆炸式增长。

#### 2.2.2 多机系统：边缘化与纯位姿图优化 (DPGO)
在多机协同系统（如分布式后端）中，为了满足严苛的通信带宽限制和分布式计算要求，系统通常采取降维策略。

* **作用转变：** Landmark 不再是后端优化的变量。两个机器人在前端匹配到共享的 Landmark 集合后，会在本地利用这些点计算出一个**相对位姿变换（Relative Pose, $T_{AB}$）**。一旦 $T_{AB}$ 计算完成，这些 Landmark 的历史使命就结束了，直接被丢弃（或边缘化），不进入全局分布式优化器。

* **数学建模：** 多机后端被降维成了一个纯粹的**分布式位姿图优化（Distributed Pose Graph Optimization, DPGO）**，仅包含位姿节点：

  $$\min_{X_\alpha} \sum_{\alpha \in \mathcal{R}} \sum_{i} \| r_{\alpha_i}(X_{\alpha_i}, X_{\alpha_{i+1}}) \|^2 + \sum_{(\alpha_i, \beta_j) \in \mathcal{L}} \rho\Big( \| r_{\beta_j}^{\alpha_i}(X_{\alpha_i}, X_{\beta_j}) \|^2 \Big)$$

  公式中只剩下单机局部里程计残差 $r_{\alpha_i}$ 和多机相对位姿残差 $r_{\beta_j}^{\alpha_i}$，并引入鲁棒核函数 $\rho$ 抵抗错误闭环。**Landmark 在此模型中被完全剥离。**

### 2.3 全局地图的一致性：是否需要优化 Landmark？

**结论：不需要在全局后端直接优化 Landmark。** 全局地图（Mesh 或稠密点云）由数以百万计的顶点组成。它们与稀疏的 Landmark 同源，但它们是“被动”的血肉，依附于“主动”的位姿骨架。保证地图一致性的核心逻辑是：**只要位姿轨迹在全局达成了一致，附着其上的地图就必然会重合。** 这在不同的地图表征下有不同的数学实现：

#### 2.3.1 基于 Mesh 的地图融合 (形变图优化)
Mesh 具有表面拓扑结构，直接改变部分顶点会造成网格撕裂。系统可以通过构建**形变图（Deformation Graph）**来实现一致性。

当 DPGO 算出了全局一致的位姿 $\bar{X}_i$ 后，机器人在本地执行**局部 Mesh 优化（Local Mesh Optimization, LMO）**：

$$\min_{X^x_i, M_k} \underbrace{\sum_{i} \|X_i \ominus \bar{X}_i\|_{\Sigma_x}^2}_{\text{位姿（骨架）锚定}} + \underbrace{\sum_{k} \sum_{l} \|R_k^M(g_l - g_k) + t_k^M - t_l^M\|_{\Sigma}^2}_{\text{Mesh表面刚性}} + \underbrace{\sum_{i} \sum_{l} \|R_i^x g_{il} + t_i^x - t_l^M\|_{\Sigma}^2}_{\text{骨肉刚性}}$$

这个公式强迫 Mesh 顶点像贴了硅胶皮肤一样，平滑地跟随优化后的骨架 $\bar{X}_i$ 移动，同时保持局部几何形状不被破坏。由于各机器人的骨架在回环处已经被拉齐，它们形变后的局部 Mesh 放在同一个全局坐标系下时，边缘自然会完美拼接。

#### 2.3.2 基于点云的地图融合 (子图刚体变换)
点云是无结构的，不需要维持表面刚性。其一致性过程退化为极其简单的**子图刚体变换**。

全局点云 $\mathcal{P}_{global}$ 是所有局部子图 $\mathcal{P}_i$ 通过最新骨架位姿 $\bar{X}_i$ 映射到全局坐标系的并集：

$$\mathcal{P}_{global} = \bigcup_{i=0}^N \left\{ \bar{X}_i \cdot p_k^{local} \mid p_k^{local} \in \mathcal{P}_i \right\}$$

为解决因微小位姿误差导致的“两面墙重叠（Ghosting）”问题，工程上通常会在重投影后，引入**体素滤波下采样（Voxel Grid Filtering）**或更新**3D 概率占据网格（OctoMap）**，将微小的物理偏差在概率上抹平。

