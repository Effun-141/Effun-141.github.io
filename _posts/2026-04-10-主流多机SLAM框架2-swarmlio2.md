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

Swarm-LIO2 是近几年一篇非常有代表性的完整的多机协同odometry系统工作，面向多无人机协同工作场景。它并不是一篇从0到1的开创性工作，而是在多篇论文的积累之上叠加组合的方法融合框架。Swarm-LIO2主要涉及以下工作：

### Swarm-LIO2论文：

[1] Zhu F, Ren Y, Yin L, et al. **[Swarm-LIO2: Decentralized efficient LiDAR-inertial odometry for aerial swarm systems](https://ieeexplore.ieee.org/abstract/document/10816004)**[J]. IEEE Transactions on Robotics, 2024, 41: 960-981.

### 前序工作Swarm-LIO：

[2] Zhu F, Ren Y, Kong F, et al. **[Swarm-LIO: Decentralized Swarm LiDAR-inertial Odometry](https://ieeexplore.ieee.org/abstract/document/10161355)**[C]. 2023 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2023: 3254-3260.

### 单机框架：

[3] Xu W, Cai Y, He D, et al. **[Fast-lio2: Fast direct lidar-inertial odometry](https://ieeexplore.ieee.org/abstract/document/9697912)**[J]. IEEE Transactions on Robotics, 2022, 38(4): 2053-2073.

### 滤波理论：

[4] He D, Xu W, Zhang F. **[Symbolic representation and toolkit development of iterated error-state extended kalman filters on manifolds](https://ieeexplore.ieee.org/abstract/document/10024988)**[J]. IEEE Transactions on Industrial Electronics, 2023, 70(12): 12533-12544.

### 单机框架前序工作：

[5] Xu W, Zhang F. **[Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter](https://ieeexplore.ieee.org/document/9372856)**[J]. IEEE Robotics and Automation Letters, 2021, 6(2): 3317-3324.

### 时间同步工作：

[6] Watt S T, Achanta S, Abubakari H, et al. **[Understanding and applying precision time protocol](https://ieeexplore.ieee.org/abstract/document/7449285)**[C]. 2015 Saudi Arabia Smart Grid (SASG). IEEE, 2015: 1-7.

其中除了最后的\[6\]，其他工作全部来自于Zhang Fu老师课题组之前发过的文章。

---

## 1. Background

单一机器人SLAM/Odometry只需要知道“我在哪里”，但多机系统不够，它还必须知道“队友在哪里”。多机系统之间的区别总结在了下面这篇笔记中：

[主流多机SLAM框架——多机SLAM问题思考与总结](https://effun.xyz/blog/2026/%E4%B8%BB%E6%B5%81%E5%A4%9A%E6%9C%BASLAM%E6%A1%86%E6%9E%B64-%E5%A4%9A%E6%9C%BASLAM%E9%97%AE%E9%A2%98%E6%80%9D%E8%80%83%E6%80%BB%E7%BB%93/)

在我看来，Swarm-LIO2更看重的是自身的状态，队友的状态用于辅助把自己的位置定的更准，这和关注mapping追求全局一致性的系统在导向上是不同的，但由于多机系统都要在初始化阶段估计机器人之间的相对位姿变换，并在后续过程中不断优化更新，因此Swarm-LIO2在某种程度上也能得到一个global坐标系下所有无人机的轨迹。

Swarm-LIO2没有针对一个具体的应用场景展开，而是基于以下四类现有方法存在的问题进行设计：

* **GPS / RTK-GPS 只适合部分室外环境**：传统室外无人机可以用 GPS 或 RTK-GPS 做定位，但问题是很多 swarm 任务发生在 GPS-denied environments，比如室内、树林、隧道、灾害现场、城市峡谷等。在这些场景里 GPS 不可靠甚至不可用。论文因此把 GPS-denied 场景作为一个重要动机。

* **Motion capture / anchor-based UWB 精度高，但不适合真实部署**:室内可以用 motion capture，或者 anchor-based UWB，但是它们依赖外部固定基础设施，比如动捕相机、UWB anchor、地面站等。这样系统就变成了中心化系统，安装复杂，而且容易有 single point of failure。对真实无人机集群任务来说，这种方式不够自主，也不够鲁棒。

* **视觉方案轻量，但深度和鲁棒性不足**: 相机便宜、轻、小，而且有丰富纹理信息，所以很多多机系统用 VIO 或视觉检测队友。但它的问题是：低光照、纹理弱、运动模糊时容易退化。

* **多机器人 SLAM 通信和计算太重**: 很多多机器人 SLAM 依赖地图、子图、描述子、回环检测、点云配准等信息交换。对地面机器人还可以接受，但对空中无人机来说，载荷、算力、续航和通信带宽都很有限。空中平台则更需要 lightweight sensors、efficient algorithms 和 low-bandwidth communication。

Swarm-LIO2一开始就把问题分成两类：

* ego state estimation：每架无人机估计自己的位姿、速度等状态
* mutual state estimation：每架无人机估计其他队友相对于自己的状态

