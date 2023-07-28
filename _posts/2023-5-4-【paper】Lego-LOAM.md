---
layout: post
title: 【close reading】LeGO-LOAM:Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
date: 2023-5-4
tags: Paper Lego-LOAM
---

*This is the close reading of the [paper](https://ieeexplore.ieee.org/document/8594299)*


# I.引言

&emsp;&emsp;SLAM（Simultaneous Localization and Mapping）即同时定位与建图技术，用于解决移动机器人（UAV、UGV等）在未知环境中实时估计自身位置并在线建立和更新周围环境地图的问题。定位和建图是两个相辅相成的过程，这是因为在未知环境中缺少绝对位置信息时，要实现准确的定位需要依赖于高精度的地图，而构建高精度的地图则需要运动估计的结果。根据采集数据所使用的传感器的不同，SLAM可初步分为基于激光雷达的激光SLAM和基于相机的视觉SLAM。但随着应用场景的复杂化以及对定位和建图精度要求的不断提高，基于相机、雷达、IMU、轮式编码器、GPS等多种传感器融合的SLAM系统已经成为主流，比较典型的有VINS、ORB-SLAM3、V-LOAM、LIO-SAM、Fast-LIO和LVI-SAM等。

&emsp;&emsp;工程中应用较多的激光SLAM算法是由Zhang Ji提出的LOAM框架。但是LOAM存在运算量大，迭代次数多的问题。LeGO-LOAM算法在LOAM算法基础上进行改进。通过点云分割聚类、点云降采样、特征点筛选与空间分配等方法，减少了激光里程计中匹配过程的迭代计算量。通过对点云与地图特征点的匹配得到更为准确的位姿估计结果。根据该结果校正点云，完成点云地图的构建与更新。

# II.LeGO-LOAM算法原理

## A.LeGO-LOAM算法流程
&emsp;&emsp;LeGO-LOAM与LOAM相同，将位姿估计问题划分为高频低精度位姿估计和低频高精度位姿估计两个不同的部分。最后将两种估计结果融合，实现高频高精度的位姿估计。此外，LeGo-LOAM在分割和优化的过程中利用了地面的约束，能够在减少计算量的情况下提升定位和建图的精度。

&emsp;&emsp;LeGO-LOAM的输入为激光雷达点云数据，输出为估计的位姿，系统结构框架如下：
<figure>
    <img src="https://effun.xyz/images/LeGO-LOAM/算法框架.png" width=300px>
    <center>
    <figcaption>图2.1 LeGO-LOAM系统框架</figcaption>
    </center>
</figure>
由图2.1可知，LeGO-LOAM系统由五个子模块组成。点云分割模块负责分割分离地面点，并对点云进行聚类处理。特征提取模块基于点云分割的结果提取点云特征，并进行帧间特征匹配。激光里程计模块基于特征点在相邻帧之间的位置变化以10Hz的频率进行位姿估计。雷达建图模块对激光里程计输出的位姿信息进一步处理，同时将去畸变点云注册到全局地图中，并以1Hz的频率更新地图和位姿。变换融合模块接收来自激光里程计模块和雷达建图模块的高低频位姿估计结果，并以10Hz的频率对其进行融合输出。

## B.点云分割与聚类

&emsp;&emsp;在进行点云分割与聚类之前需要先对点云进行重投影操作，将每一帧点云的所有点投影到一张深度图像（range image）上，并为每一个点计算其行号（rowIdn）和列号(columnIdn)。对于一个16线激光雷达来说，若其水平分辨率(ang_res_x)为$$0.2^\circ$$，垂直分辨率(ang_res_y)为$$2^\circ$$，则每扫描一圈会得到一个16×1800的点云阵列，垂直角度$$θ_v$$范围为[-15°,15°]，水平角度$$θ_h$$范围为[0°,360°]。





