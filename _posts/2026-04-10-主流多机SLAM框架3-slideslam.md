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



### 一、回环检测算法（Loop Closure Detection）

#### 1.1 算法结构：分层网格搜索 + RANSAC

当Robot 1和Robot 2的地图有重叠时，代码使用以下步骤检测回环：

**第一步：地图预处理与中心对齐**

代码位置：`place_recognition.cpp` 第736-807行

```cpp
bool PlaceRecognition::findTransformation(
    const std::vector<Eigen::Vector7d> &reference_objects_input,
    const std::vector<Eigen::Vector7d> &query_objects_input,
    std::vector<double> &xyz_yaw_out, 
    Eigen::Matrix4d &transform_out) {
  
  // 步骤1：计算两个地图的质心（中心点）
  Eigen::Vector2d centroid_reference = getCentroid(reference_objects_input);
  Eigen::Vector2d centroid_query = getCentroid(query_objects_input);
  
  // 步骤2：将两个地图的中心都对齐到原点
  // 这样做的目的是减少搜索范围
  for (int i = 0; i < reference_objects_input.size(); i++) {
    reference_objects[i][1] -= centroid_reference[0];  // ← X坐标中心化
    reference_objects[i][2] -= centroid_reference[1];  // ← Y坐标中心化
  }
  // query_objects 也做同样处理
  
  // 步骤3：自动计算搜索范围
  Eigen::Vector2d boundaries_reference = getMapBoundaries(reference_objects);
  Eigen::Vector2d boundaries_query = getMapBoundaries(query_objects);
  
  // 使用地图的最大范围 × 膨胀系数 作为搜索空间
  match_x_half_range_ = max_x_region * dilation_factor_;  // 膨胀因子通常为1.2
  match_y_half_range_ = max_y_region * dilation_factor_;
}
```

**关键参数含义**：

```
Reference Map（Robot 1的地图）：
  规模：宽度 2*max_x_region_ref，高度 2*max_y_region_ref
  
Query Map（Robot 2的地图）：
  规模：宽度 2*max_x_region_query，高度 2*max_y_region_query

Search Range（搜索范围）：
  x范围：[-max_x_region × dilation, +max_x_region × dilation]
  y范围：[-max_y_region × dilation, +max_y_region × dilation]
  yaw范围：[-180°, +180°] 或更小的范围
```

**第二步：分层网格搜索（Anytime Algorithm）**

代码位置：`place_recognition.cpp` 第140-250行

```cpp
void PlaceRecognition::MatchMaps(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects, 
    Eigen::Matrix3d &R_t_out, 
    int &best_num_inliers_out, ...) {
  
  // 准备候选方向
  std::vector<double> yaw_candidates;
  for (double yaw = -match_yaw_half_range_; 
       yaw < match_yaw_half_range_; 
       yaw += match_yaw_angle_step_size_) {  // 步长例如2°
    yaw_candidates.push_back(yaw);
  }
  
  // ✅ 关键：分层扩大搜索范围（Anytime Algorithm）
  // 这样做的好处是可以快速找到大致匹配，然后细化
  
  double outer_loop_step_size_x = match_x_half_range_ / outer_loop_steps;
  double outer_loop_step_size_y = match_y_half_range_ / outer_loop_steps;
  
  // 外层循环：逐步扩大搜索范围
  for (int cur_step = 0; cur_step < outer_loop_steps; cur_step++) {
    // 检查是否超过时间预算（防止过长的搜索）
    if (duration > compute_budget_sec_) {
      break;  // ← 5秒时间限制
    }
    
    // 计算当前步的搜索范围（环形区域）
    double x_positive_end = (cur_step + 1) * outer_loop_step_size_x;
    double y_positive_end = (cur_step + 1) * outer_loop_step_size_y;
    
    // 内层循环：逐点网格搜索
    for (double x = x_negative_start; x <= x_positive_end; 
         x += match_xy_step_size_) {  // 网格步长例如0.5m
      for (double y = y_negative_start; y <= y_positive_end; 
           y += match_xy_step_size_) {
        
        // 对每个(x, y)位置，尝试所有yaw角度
        Eigen::Matrix3d cur_R_t = Eigen::Matrix3d::Identity();
        for (double yaw : yaw_candidates) {
          // 构造变换矩阵（2D变换）
          cur_R_t(0, 0) = cos(yaw);
          cur_R_t(0, 1) = -sin(yaw);
          cur_R_t(0, 2) = x;      // ← tx
          cur_R_t(1, 0) = sin(yaw);
          cur_R_t(1, 1) = cos(yaw);
          cur_R_t(1, 2) = y;      // ← ty
          
          // 使用这个变换，尝试配对地标
          // 计数有多少个查询地标能在参考地图中找到对应的地标
          int num_inliers = 0;
          for (int i = 0; i < query_objects.size(); i++) {
            // 将查询地标变换到参考坐标系
            Eigen::Vector2d query_point = 
                Eigen::Vector2d(query_objects[i][1], query_objects[i][2]);
            Eigen::Vector2d transformed_point = 
                cur_R_t.block<2, 2>(0, 0) * query_point + 
                cur_R_t.block<2, 1>(0, 2);
            
            // 在参考地图中搜索距离最近的地标
            for (int j = 0; j < reference_objects.size(); j++) {
              Eigen::Vector2d ref_point = 
                  Eigen::Vector2d(reference_objects[j][1], 
                                reference_objects[j][2]);
              
              // 检查距离和语义标签是否匹配
              double dist = (transformed_point - ref_point).norm();
              bool label_match = 
                  (query_objects[i][0] == reference_objects[j][0]);
              
              if (dist < match_threshold_ && label_match) {
                num_inliers++;  // ← 计数匹配点
                break;
              }
            }
          }
          
          // 保存找到的最好结果
          if (num_inliers > best_num_inliers) {
            best_num_inliers = num_inliers;
            best_R_t = cur_R_t;
            // 也保存这些匹配的地标对
            best_matched_map_objects = ...;
            best_matched_detection_objects = ...;
          }
        }
      }
    }
  }
}
```

**这个算法的步骤总结**：

```
步骤1：准备阶段
  ✅ 计算两个地图的质心
  ✅ 中心化地图坐标
  ✅ 自动确定搜索范围

步骤2：分层搜索
  ✅ 外层循环：从小到大扩广搜索区域
  ✅ 中层循环：网格点搜索(x, y)
  ✅ 内层循环：尝试不同的yaw角

步骤3：质量评估
  ✅ 对每个候选(x, y, yaw)：
  ✅   将Query地标变换到Reference坐标系
  ✅   检查是否能在Reference地图中找到对应的地标
  ✅   计数匹配点数

步骤4：验证
  ✅ 要求匹配点数 ≥ min_num_inliers（例如5个）
  ✅ 要求重叠率 ≥ 某个阈值

步骤5：输出
  ✅ 返回最好的(x, y, yaw)和对应的匹配地标对
```

#### 1.2 关键参数配置

```yaml
# 在ROS参数文件中配置
place_recognition:
  # 网格搜索参数
  search_xy_step_size: 0.5        # 网格步长 0.5米
  search_yaw_step_size_degrees: 2.0  # 方向步长 2°
  
  # 搜索范围自动扩展
  dilation_factor: 1.2            # 将地图范围扩大20%作为搜索区域
  
  # 匹配允许的误差
  match_threshold_position: 0.5   # 位置误差 <0.5m
  match_threshold_dimension: 1.0  # 尺寸误差 <1.0m
  
  # 有效回环需要的条件
  min_num_inliers: 5              # 最少需要5个匹配点
  min_loop_closure_overlap_percentage: 0.1  # 10%重叠度
  
  # 计算时间预算
  compute_budget_sec: 5.0         # 最多搜索5秒
```

### 二、变换矩阵 T 的计算方法

一旦找到了匹配的地标对，代码使用**最小二乘法 + SVD分解**来精确计算T矩阵。

#### 2.1 问题表述

已知：
- Reference地图中的地标：$\{P_1^{ref}, P_2^{ref}, ..., P_N^{ref}\}$
- Query地图中的地标：$\{P_1^{query}, P_2^{query}, ..., P_N^{query}\}$
- 对应关系：第i个Query地标对应第i个Reference地标

求解：变换矩阵 $T = [R|t]$ 使得：

$$T^* = \arg\min_T \sum_{i=1}^{N} \| T \cdot P_i^{query} - P_i^{ref} \|^2$$

#### 2.2 SVD分解求解算法

代码位置：`place_recognition.cpp` 第632-700行

```cpp
void PlaceRecognition::solveLSQ(
    const std::vector<Eigen::Vector3d> &map_objects_matched_out,
    const std::vector<Eigen::Vector3d> &detection_objects_matched_out,
    std::vector<double> &xyzyaw_out, 
    Eigen::Matrix4d &transform_out) {
  
  // 记号：
  // A = detection_objects_matched_out（Query地标）
  // B = map_objects_matched_out（Reference地标）
  
  // ──────────────────────────────────────────────
  // 第1步：计算质心
  // ──────────────────────────────────────────────
  Eigen::Vector3d centroidA = detection_objects_matched_out_matrix.colwise().mean();
  Eigen::Vector3d centroidB = map_objects_matched_out_matrix.colwise().mean();
  
  // ──────────────────────────────────────────────
  // 第2步：中心化点云（移除平移的影响）
  // ──────────────────────────────────────────────
  Eigen::MatrixXd centeredA = A - centroidA;  // ← 每个点减去质心
  Eigen::MatrixXd centeredB = B - centroidB;  // ← 每个点减去质心
  
  // 目的：将问题简化为只求旋转R
  // min_R ∑||R·(A_i - centroidA) - (B_i - centroidB)||²
  
  // ──────────────────────────────────────────────
  // 第3步：计算交叉协方差矩阵
  // ──────────────────────────────────────────────
  // H = ∑(A_i - centroidA)(B_i - centroidB)ᵀ
  Eigen::Matrix3d H = centeredA * centeredB.transpose();
  
  // 矩阵H的大小为 3×3
  // H包含了两个点集之间的相对位置关系
  
  // ──────────────────────────────────────────────
  // 第4步：SVD分解
  // ──────────────────────────────────────────────
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      H, 
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  // SVD分解：H = U·Σ·Vᵀ
  // U: 3×3 左奇异向量矩阵
  // Σ: 3×3 奇异值对角矩阵（Σ₁ ≥ Σ₂ ≥ Σ₃ ≥ 0）
  // V: 3×3 右奇异向量矩阵
  
  // ──────────────────────────────────────────────
  // 第5步：计算最优旋转矩阵
  // ──────────────────────────────────────────────
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  // R = V·Uᵀ
  
  // 这个R就是使得 ∑||R·A'ᵢ - B'ᵢ||² 最小的旋转矩阵
  // （Kabsch算法 / Umeyama算法）
  
  // ──────────────────────────────────────────────
  // 第6步：处理旋转矩阵的行列式（Proper Rotation）
  // ──────────────────────────────────────────────
  if (R.determinant() < 0) {
    // det(R) < 0 意味着计算出的不是旋转而是反射+旋转
    // 需要修正：翻转第3列（最小奇异值对应的轴）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) = -V.col(2);  // ← 翻转第3列
    R = V * svd.matrixU().transpose();
  }
  
  // ──────────────────────────────────────────────
  // 第7步：计算平移向量
  // ──────────────────────────────────────────────
  // t = centroidB - R·centroidA
  Eigen::Vector3d t = centroidB - R * centroidA;
  
  // 含义：要使Query点云对齐到Reference点云：
  // 1. 先将Query点云减去其质心
  // 2. 旋转
  // 3. 加上Reference点云的质心
  // P_ref = R·P_query + t  就是这个过程
  
  // ──────────────────────────────────────────────
  // 第8步：组装4×4变换矩阵并提取(x,y,z,yaw)
  // ──────────────────────────────────────────────
  Eigen::Matrix4d transform_out = Eigen::Matrix4d::Identity();
  transform_out.block<3, 3>(0, 0) = R;  // 旋转3×3块
  transform_out.block<3, 1>(0, 3) = t;  // 平移列向量
  
  getxyzYawfromTF(transform_out, xyzyaw_out);
}
```

#### 2.3 从4×4矩阵提取(x, y, z, yaw)

代码位置：`place_recognition.cpp` 第703-718行

```cpp
void PlaceRecognition::getxyzYawfromTF(
    const Eigen::Matrix4d &tf, 
    std::vector<double> &xyzYaw) {
  
  // 提取平移部分：第0,1,2行的第3列
  Eigen::Vector3d translation = tf.block<3, 1>(0, 3);
  x = translation[0];
  y = translation[1];
  z = translation[2];
  
  // 从旋转矩阵提取欧拉角
  // R = [r11 r12 r13]
  //     [r21 r22 r23]
  //     [r31 r32 r33]
  
  // 对于主要是Z轴旋转的情况（2D旋转），只需yaw角：
  double yaw = atan2(R(1, 0), R(0, 0));
  
  // 如果需要完整的欧拉角（Roll, Pitch）
  double roll = atan2(R(2, 1), R(2, 2));
  double pitch = atan2(-R(2, 0), sqrt(R(2, 1)² + R(2, 2)²));
  
  // 返回[x, y, z, yaw]
  xyzYaw = {x, y, z, yaw};
}
```

#### 2.4 数学验证：为什么SVD分解能求解旋转

**定理（Kabsch/Umeyama算法）**：

对于点集 $\{A_i\}$ 和 $\{B_i\}$，求解最小二乘意义下的旋转R和平移t使得：

$$\min_{R,t} \sum_i \|R·A_i + t - B_i\|^2$$

最优解为：

1. $\bar{A} = \frac{1}{N}\sum A_i$（A的质心）
2. $\bar{B} = \frac{1}{N}\sum B_i$（B的质心）
3. $H = \sum (A_i - \bar{A})(B_i - \bar{B})ᵀ$（交叉协方差）
4. $H = U Σ Vᵀ$（SVD分解）
5. $R^* = V U^T$（最优旋转）
6. $t^* = \bar{B} - R^* \bar{A}$（最优平移）

**证明思路**：

通过展开平方和：
$$\sum \|RA_i + t - B_i\|^2 = \sum \|R(A_i - \bar{A}) - (B_i - \bar{B})\|^2 + N\|\bar{B} - R\bar{A}\|^2$$

第一项只依赖于R，第二项对t的最小化给出 $t = \bar{B} - R\bar{A}$。

因此只需最小化第一项。通过SVD可以证明最优旋转为 $R^* = VU^T$。

### 三、完整的回环检测+变换计算流程

```
【Robot 1和Robot 2相遇】
      ↓
【地图交换】
  Robot 1收到Robot 2的地图
  
      ↓
【 findInterLoopClosure 调用】
  入参：
    - reference_objects: Robot 1的地图 {L_A, L_B, L_C, ...}
    - query_objects: Robot 2的地图 {L_X, L_Y, L_Z, ...}
  出参：
    - tfFromQueryToRef: 4×4 变换矩阵 T_2^1
      ↓
【 findTransformation 函数】
  
  ├─→ 步骤1：地图预处理
  │   ├─→ 计算质心
  │   ├─→ 中心化坐标
  │   └─→ 自动确定搜索范围
  │
  ├─→ 步骤2：MatchMaps（网格搜索）
  │   └─→ 分层扩大搜索区域
  │   └─→ 逐个尝试(x, y, yaw)组合
  │   └─→ 计算每个组合的匹配点数
  │   └─→ 输出：最好的匹配及其对应的地标对
  │
  └─→ 步骤3：solveLSQ（精确求解T）
      ├─→ 使用SVD分解
      ├─→ 计算最优旋转矩阵R
      ├─→ 计算最优平移向量t
      └─→ 输出：T = [R|t] 以及 (x, y, z, yaw)
      
      ↓
【验证质量】
  ✅ 匹配点数 ≥ 5
  ✅ 重叠率 ≥ 10%
  
      ↓
【存储和使用】
  dbManager.loopClosureTf[robot_id] = T;
  ← T被"钉死"，后续所有的坐标变换都用这个T
```

### 四、代码中两个重要算法的对比

#### 方法1：基于网格搜索+数据关联（SlideMatch）

```cpp
// 优点：
// ✅ 快速初步搜索
// ✅ 对部分重叠堡垒（robust）
// ✅ 不需要完全精确的初始猜测

// 使用场景：
inter_loopCloser_.findInterLoopClosure(...)
// 或
inter_loopCloser_.findInterLoopClosureWithClipper(...)
```

#### 方法2：基于图匹配（CLIPPER，可选）

代码位置：`place_recognition.cpp` 第541行

```cpp
#if USE_CLIPPER
bool PlaceRecognition::findInterLoopClosureWithClipper(
    const std::vector<Eigen::Vector7d> &reference_objects,
    const std::vector<Eigen::Vector7d> &query_objects,
    Eigen::Matrix4d &tfFromQueryToRef) {
  
  // CLIPPER = Consistent Landmark Identification with Provable 
  // Efficiency and Robustness
  
  // 这个方法使用图论的方法寻找一致的地标对应
  // 比网格搜索更精确，但计算量更大
  
  // 优点：
  // ✅ 更可靠的地标对应（graph-theoretic方法）
  // ✅ 自动处理多个可能的连续变换
  
  // 缺点：
  // ❌ 计算昂贵
  // ❌ 需要更多的地标
}
#endif
```

### 五、关键代码行引用总结

| 功能 | 代码位置 | 行数 | 说明 |
|------|---------|------|------|
| 回环检测入口 | place_recognition.cpp | 498-540 | `findInterLoopClosure()` |
| 网格搜索 | place_recognition.cpp | 105-250 | `MatchMaps()`，逐点尝试(x,y,yaw) |
| 验证匹配质量 | place_recognition.cpp | 830-850 | 检查是否有足够的inliers |
| SVD求解T | place_recognition.cpp | 665-685 | `solveLSQ()`中的SVD分解 |
| 提取欧拉角 | place_recognition.cpp | 703-718 | `getxyzYawfromTF()` |
| 存储T矩阵 | sloamNode.cpp | 605 | `dbManager.loopClosureTf[id] = T` |
| 使用T矩阵 | sloamNode.cpp | 850 | `T * robot2_measurement` |

---