---
layout: post
title: 【close reading】LIW-OAM：Lidar-Inertial_Wheel Odometry and Mapping
date: 2023-4-26
tags: Paper
---

*This is the close reading of the [paper](http://arxiv.org/abs/2302.14298)*

# 摘要

&emsp;&emsp;本文提出一种激光雷达、IMU和轮式里程计融合的激光SLAM方案，称为LIW-OAM。该算法基于BA进行后端优化。轮式编码器数据既可以协助LI-OAM进行状态估计，同时也能约束速度变量，极大提高了状态估计的精度。实验表明，相比与主流的LI-OAM系统，LIW-OAM有更小的绝对轨迹误差（ATM）。

# 引言

&emsp;&emsp;传统LI-OAM融合激光雷达和IMU数据进行实时状态估计（估计位姿和速度），利用解算得到的状态变量进行点云配准（registration）并把配准后的点云融合进地图中。LI-OAM可分为松耦合和紧耦合。松耦合，如Lego-LOAM，用IMU数据标定雷达点云的运动偏移，并给ICP提供运动先验信息用于位姿估计。这种方法的问题是计算当前的运动先验信息需要依赖上一个速度状态量，但是没有任何传感器能够直接测量速度也没有对速度的优化程序。这会导致误差不断累积。





