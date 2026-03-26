---
layout: post
title: 主流多机SLAM框架1——Kimera-Multi
date: 2026-03-21 14:24:00
description: 【论文阅读】深入理解Kimera-Multi
tags: 
categories: SLAM Distributed-Optimization Paper-Reading
chart:
  plotly: true
---

Kimera-Multi是MIT SPARK实验室提出的一种多机协同SLAM算法，共有ICRA和TRO两个版本，本文将对其系统进行深入理解。

Kimera-Multi [ICRA](https://ieeexplore.ieee.org/abstract/document/9561090?casa_token=RNUxNIr638MAAAAA:E5EbWTn2-9VzpIeRaxqsDYlHpFjnrddeZs-0OCmR6Y3Fzpo1_YWc6sk-lmAatoXlQ4QO3LbkZB9leg) 版：
```
@INPROCEEDINGS{9561090,
  author={Chang, Yun and Tian, Yulun and How, Jonathan P. and Carlone, Luca},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Kimera-Multi: a System for Distributed Multi-Robot Metric-Semantic Simultaneous Localization and Mapping}, 
  year={2021},
  volume={},
  number={},
  pages={11210-11218},
  keywords={Solid modeling;Three-dimensional displays;Simultaneous localization and mapping;Protocols;Computational modeling;Data models;Trajectory},
  doi={10.1109/ICRA48506.2021.9561090}}

```

Kimera-Multi [TRO](https://ieeexplore.ieee.org/document/9686955) 版：
```
@ARTICLE{9686955,
  author={Tian, Yulun and Chang, Yun and Herrera Arias, Fernando and Nieto-Granda, Carlos and How, Jonathan P. and Carlone, Luca},
  journal={IEEE Transactions on Robotics}, 
  title={Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems}, 
  year={2022},
  volume={38},
  number={4},
  pages={2022-2038},
  keywords={Simultaneous localization and mapping;Trajectory;Collaboration;Semantics;Estimation;Robot kinematics;Solid modeling;Multi-robot systems;simultaneous localization and mapping;robot vision systems},
  doi={10.1109/TRO.2021.3137751}}

```

从代码看：

* Kimera-Multi 的分布式后端（DPGO）主要优化的是位姿图，不是全局 landmark 图。

* landmark 在系统里主要用于单机 VIO/建图与可视化（前端+本地后端），而不是多机分布式后端的联合变量。

DPGO 的图分类只有 odom、private loop closure、shared loop closure，分布式前端输出给后端的增量图在处理时只看 ODOM 和 LOOPCLOSE，虽然消息定义里有 LANDMARK 枚举，但当前分布式链路没有把它作为后端可优化实体来处理，landmark 的“实质优化”主要在单机 VIO 后端（smart factor + iSAM2）

## 分布式优化中的“全局”究竟是什么？

