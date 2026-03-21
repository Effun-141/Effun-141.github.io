---
layout: post
title: 多机器人协同感知1——问题建模
date: 2026-03-20 14:24:00
description: 综合阅读一些相关论文，搞清楚多机分布式优化问题的本质
tags: 
categories: Distributed-Optimization Paper-Reading
chart:
  plotly: true
---

## 基于滤波or基于平滑？

首先感叹一下《视觉SLAM 14讲》这本书真的常看常新，上次阅读这本书可能已经是2年之前了，但回过头来发现那时我并未真正理解SLAM后端，也不理解滤波方法和优化方法的区别到底是什么。在阅读多机协同SLAM的论文时我终于有点感受到两种方法之间的差异，我觉得从简单来说，滤波就算一步，在多机通信受限的时候能够提供快速的状态估计，[Swarm-LIO2](https://ieeexplore.ieee.org/document/10816004)在多机交互的时候使用了这种思想。滤波的方法遵循马尔可夫性，认为当前状态只与前一/几个时刻的状态相关，而与其他时刻的状态与观测都无关。而优化的方法则综合利用了所有时刻的信息，即使状态变化较大也更加可靠，虽然计算开销变大了，但优化方法依然更适用于大型场景。

以下是今天翻阅14讲时读到的一个小节，以前在读的时候由于知识浅薄不能明白这一节的意义，现在看来当时错过了很多这种细节，惭愧惭愧。
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260320/1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    
</div>
