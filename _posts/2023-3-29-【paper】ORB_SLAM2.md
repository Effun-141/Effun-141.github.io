---
layout: post
title: ORB-SLAM2:An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras
date: 2023-3-29 
tags: Paper
---

*This is the translation of the paper*

# 摘要：

本文提出了一种能够使用单目相机、双目相机和RGB-D相机的完整SLAM系统，称为ORB-SLAM2。该系统具有地图复用、回环检测和重定位功能。ORB-SLAM2能够实时在标准的中央处理器上运行，并能够在从小型室内手持设备到工业环境中飞行的无人机再到绕城市行驶的汽车等多种环境下工作。该系统基于带有单双目观测的光束平差法（BA）进行后端优化，从而实现精确的轨迹估计。本文所提出的系统包含一个轻量级的定位模式，该模式利用视觉里程计来跟踪未建图区域并匹配地图点以实现零漂移定位。在29个广泛使用的公开数据集下测试的结果表明，本文所提出的SLAM方法能够在多数情况下获得最高的精度。本文开源代码，不仅为了造福SLAM社区，更是为了给其他研究领域的研究人员提供一个可以直接使用的SLAM解决方案。

关键词：定位，建图，RGB-D，同时定位与建图（SLAM），双目
