---
layout: post
title:  "Design of a Navigation and Localization System for a Quadruped Bionic Robot"
info: "Researched Visual Simultaneous Localization and Mapping (VSLAM) techniques for undergraduate thesis project."
tech: "Outcomes: An undergraduate thesis that won the second prize for excellent graduation design in Jiangsu Province."
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

1. 
2. 
3. 

研究内容：主要研究内容为同时定位与建图技术（SLAM），机器人平台为宇树科技A1。在Ubuntu18.04系统下，基于ORB-SLAM2算法框架和ROS，使用Intel D435i深度相机，实现机器人室内定位精度5cm ，室外定位精度0.5m ,并建立了点云地图和服务于导航的八叉树地图。

研究方法：
定位子系统：完成了基于ORB特征匹配的前端视觉里程计、基于Bundle Adjsutment的后端优化、基于词袋模型的回环检测模块设计。基于TUM数据集验证了定位精度，使用g2o库自带的球形数据集验证了后端优化和回环检测效果。
建图子系统：利用相机采集的深度和颜色信息,基于Pcl库和Octomap库建立了半稠密点云地图和服务于导航的八叉树地图。
导航子系统：设计了一种 A*-DWA 融合导航算法，兼顾全局最优路径和局部动态避障性能。

研究结论：机器人室内实时定位精度5cm ，室外实时定位精度0.5m ,并建立了点云地图和服务于导航的八叉树地图。


#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Jan. 2021 - Jun. 2021

## Contributions

1. Learned visual SLAM fundamental theories and ORB-SLAM2 source code.
2. Reproduced ORB-SLAM2 for indoor/outdoor high-precision localization using TUM dataset and D435i camera.
3. Developed dense point cloud and octree mapping programs using ROS, OpenCV, and PCL for environment perception and navigation.

## Conclusion

1. Concluded that the designed navigation and localization system can achieve indoor localization accuracy of less than 5cm and outdoor accuracy of less than 0.3m.
2. Produced the point cloud map and the octree map.


## Outcomes
 
An undergraduate thesis that won the second prize for excellent graduation design in Jiangsu Province.

## Project Showcase

### Robot used for experiments:

### Experimental processing and results:
