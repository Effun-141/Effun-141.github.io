---
layout: post
title: 主流多机SLAM框架3——SlideSLAM
date: 2026-04-01 14:24:00
description: 【论文阅读】深入理解SlideSLAM
tags: 
categories: SLAM Distributed-Optimization Paper-Reading
chart:
  plotly: true
---

## 等价关系的严格数学推导

### Step 1：贝叶斯公式的展开

论文的公式(12)使用**最大后验估计（MAP）**：

$$P(\mathbf{x}_{1:t}, \mathcal{M} | \mathcal{I}_t) \propto P(\mathcal{I}_t | \mathbf{x}_{1:t}, \mathcal{M}) \cdot P(\mathbf{x}_{1:t}, \mathcal{M})$$

其中：
- $\mathcal{I}_t$ = 所有收集到的信息（传感器观测 + 其他机器人的观测）
- $\mathbf{x}_{1:t}$ = 轨迹（所有时刻的位姿）
- $\mathcal{M}$ = 地图（所有地标）

### Step 2：高斯假设 → 最小二乘形式

在**高斯噪声假设**下（SLAM中的标准假设），似然函数可以写成：

$$P(\mathcal{I}_t | \mathbf{x}_{1:t}, \mathcal{M}) \propto \exp\left( -\frac{1}{2} \sum_i \|e_i(\mathbf{x}, \mathcal{M})\|_{\Sigma_i^{-1}}^2 \right)$$

取对数（负号）：

$$-\log P(\mathcal{I}_t | \mathbf{x}, \mathcal{M}) = \underbrace{\frac{1}{2} \sum_i \|e_i\|_{\Sigma_i^{-1}}^2}_{\text{所有观测的误差项}}$$

### Step 3：最大后验 → 最小化负对数似然

最大化 $P(\mathcal{I}_t | \mathbf{x}, \mathcal{M})$ 等价于最小化 $-\log P(\mathcal{I}_t | \mathbf{x}, \mathcal{M})$：

$$\arg\max P(\mathcal{I}_t | \mathbf{x}, \mathcal{M}) = \arg\min \left[ \sum_i \|e_i\|_{\Sigma_i^{-1}}^2 \right]$$

### Step 4：对应实现

在SLIDE SLAM中，$\sum_i \|e_i\|_{\Sigma_i^{-1}}^2$ 被具体细分为：

```
所有观测的误差项 = 
  {Robot 1的里程计误差} 
  + {Robot 1的语义观测误差}
  + {Robot 2的转换后观测误差}
  + {Loop Closure约束误差 × 100倍权重}
  + ...
```

**即**：

$$\sum_i \|e_i\|_{\Sigma_i}^2 = \sum_{odom} \|e_{odom}\|^2 + \sum_{obs} \|e_{obs}\|^2 + 100 \sum_{LC} \|e_{LC}\|^2$$