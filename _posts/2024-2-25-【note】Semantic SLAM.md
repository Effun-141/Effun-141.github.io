---
layout: post
title: 【note】Semantic SLAM
date: 2024-2-25
tags: Semantic-SLAM 
---

*This is the record of relating knowledge about semantic SLAM from paper or blogs I read.*

&emsp;&emsp;**Geometry-based Dynamic SLAM**: The main idea of geometry-based methods for dealing with dynamic objects is to treat them as outliers and reject them using robust weighting functions or motion consistency constraints.[1] 主要思想是假设只有静态特征能够满足几何约束。

&emsp;&emsp;用鲁棒加权函数或运动一致性约束剔除动态点。

&emsp;&emsp;**Learning-based SLAM:**一些基于学习的SLAM方法会结合几何方法来获得更加准确的动态和静态目标区分结果。例如：DS-SLAM，DynaSLAM等。[1]

&emsp;&emsp;K-Means聚类算法

&emsp;&emsp;极线约束（对极几何）：当前图像中的静态特征点必须位于与前一图像中相同特征点对应的极线上，如果特征点与相应极线的距离超过经验阈值，则该特征点被认为是动态的。系统的基本矩阵是在里程表的帮助下计算的。

&emsp;&emsp;语义分割方法在一定程度上解决了由于边界框导致的错误识别问题。

&emsp;&emsp;[OpenCV数据类型](https://waltpeter.github.io/open-cv-basic/opencv-datatypes/index.html)

# Reference

[1] Towards Real-time Semantic RGB-D SLAM in Dynamic Environments
