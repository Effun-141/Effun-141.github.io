---
layout: post
title: 【close reading】SG-SLAM:A Real-Time RGB-D Visual SLAM Toward Dynamic Scenes With Semantic and Geometric Information
date: 2023-7-28
tags: Paper 
---

*This is the close reading of the [paper](https://ieeexplore.ieee.org/abstract/document/9978699)*


# I.Abstract

&emsp;&emsp;这篇论文面向动态场景，旨在为度量地图添加语义信息。为此作者提出了SG-SLAM框架，S代表语义，G代表几何，即该框架是基于语义和几何方法相结合实现的RGB-D SLAM系统。SG-SLAM为ORB-SLAM2添加了两个并行线程，一个是2D语义信息提取(检测动态目标)线程和一个语义信息映射到地图的线程。

# II.Contributions

* 提出了一个完整的SLAM系统——SG-SLAM，该系统在动态环境下表现优秀，且能够发布度量语义地图。
* 提出一种基于几何和语义信息的快速动态特征点剔除算法。几何信息通过极线约束获得，语义信息提供NCNN神经网络获得。
* 基于ROS发布度量语义地图和八叉树地图，能够很好地支持后续定位导航和目标识别等任务。

# III.Method

## A.Overall framework

<figure>
    <img src="https://effun.xyz/images/SG-SLAM/sg-slam-system-overview-zh.png" width=800px>
    <center>
    <figcaption>
    系统框图
    </figcaption>
    </center>
</figure>

&emsp;&emsp;添加物体检测线程的目的是利用神经网络获取二维语义信息。然后，该二维语义信息为动态特征剔除策略提供先验动态对象信息。语义映射线程集成关键帧的2D语义信息和3D点云信息，生成3D语义对象数据库。
