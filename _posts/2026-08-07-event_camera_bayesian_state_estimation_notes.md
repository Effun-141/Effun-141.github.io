---
layout: post
title: 从事件相机6-DoF跟踪理解贝叶斯状态估计：似然、KL散度、EKF与MAP优化
date: 2026-08-06 14:58:00
description: 以 Gallego 等人的事件相机6-DoF跟踪论文为主线，从事件生成模型出发，理解贝叶斯滤波、鲁棒似然、KL后验近似，以及它们与SLAM因子图和MAP优化的统一关系
tags: Event-Camera Bayesian-Filtering EKF MAP SLAM
categories: Event-Camera Paper-Reading SLAM
chart:
  plotly: false
---

> 阅读论文：**Event-Based, 6-DOF Camera Tracking from Photometric Depth Maps**  
> Guillermo Gallego, Jon E. A. Lund, Elias Mueggler, Henri Rebecq, Tobi Delbruck, Davide Scaramuzza  
> IEEE TPAMI, 2018

这篇论文最吸引我的地方，并不只是“事件相机可以高速跟踪”这一结论，而是它提供了一个非常完整的概率状态估计例子：从**传感器物理生成机制**出发构造残差，再将残差写成**概率似然**，利用贝叶斯公式更新状态后验，最后通过 **KL 散度最小化**把复杂后验重新近似成可处理的分布，并得到一个类似 EKF 的闭式更新。

这套思路并不局限于事件相机。它其实对应了机器人状态估计中一个非常 general 的范式：

$$
\boxed{
\text{状态传播}
\rightarrow
\text{观测残差}
\rightarrow
\text{似然}
\rightarrow
\text{后验}
\rightarrow
\text{滤波或优化求解}
}
$$

这篇笔记先梳理论文方法，再把它抽象到更一般的 SLAM / 状态估计框架中。

---

# 1. 论文到底在解决什么问题？

论文并不是完整 SLAM，而是一个 **map-based tracking / localization** 问题。

已知一张提前建立好的 photometric depth map：

$$
\mathcal M
=
\left\{
I_l^r,\,
D_l^r,\,
\xi_l^r
\right\}_{l=1}^{N_r},
$$

其中包含：

- 参考强度图 $I_l^r$；
- 对应的深度图 $D_l^r$；
- 参考相机位姿 $\xi_l^r$。

在线阶段只有事件流：

$$
e_k=(x_k,y_k,t_k,p_k),
$$

系统需要在每个事件到达时估计当前 6-DoF 位姿。

因此问题可以概括成：

$$
\boxed{
\text{预建光度深度地图}
+
\text{异步事件流}
\rightarrow
\text{逐事件6-DoF位姿}
}
$$

论文的核心目标不是在线建图，而是利用事件相机：

- 微秒级时间分辨率；
- 高动态范围；
- 无传统曝光积分导致的 motion blur；

来实现低延迟、高速运动下的位姿跟踪。

---

# 2. 事件相机的物理生成模型

理解整篇论文首先要理解事件到底测量了什么。

事件相机不像传统相机输出绝对亮度，而是检测**同一个像素随时间的对数亮度变化**：

$$
\Delta \ln I
=
\ln I(\mathbf u,t)
-
\ln I(\mathbf u,t-\Delta t),
$$

其中：

- $\mathbf u=(x,y)^\top$ 是像素坐标；
- $t-\Delta t$ 是该像素上一次触发事件的时刻；
- $\Delta t$ 不是固定帧间隔，而是该像素自己的 inter-event time。

当对数亮度变化超过阈值时触发事件：

$$
\Delta\ln I
\gtrless
C_{\mathrm{th}}.
$$

如果考虑极性，可以近似理解为：

$$
\Delta \ln I
\approx
pC_{\mathrm{th}},
\qquad
p\in\{-1,+1\}.
$$

这里的一个关键认识是：

> 事件输出的不是“当前像素多亮”，而是“当前像素相对于上次触发时变亮了还是变暗了”。

由于

$$
\ln I(t)-\ln I(t-\Delta t)
=
\ln\frac{I(t)}{I(t-\Delta t)},
$$

事件相机本质上响应的是**相对亮度变化**。

在小变化情况下：

$$
\Delta\ln I
\approx
\frac{\Delta I}{I}.
$$

这也是事件相机高动态范围的重要原因之一。

---

# 3. 从物理模型到残差：论文最关键的一步


<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/img/20260806/1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    
</div>


理想事件应满足：

$$
\Delta\ln I=C_{\mathrm{th}}.
$$

作者没有直接使用绝对光度差，而是定义了一个无量纲的 implicit measurement function：

$$
\boxed{
M
=
\frac{\Delta\ln I}{C_{\mathrm{th}}}-1
}
$$

理想情况下：

$$
M=0.
$$

这里非常值得强调：

> **$M$ 首先是 residual / measurement function，而不是 loss function。**

正确状态下，事件应该满足 $M\approx 0$。  
于是问题转化为：

> 给定一个候选状态，这个状态预测出来的亮度变化是否符合事件触发机制？

这是一个很有启发性的建模方式。相比人为构造“事件离边缘有多远”，作者直接从传感器生成机制出发构造观测残差。

---

# 4. 如何从地图预测 $\Delta\ln I$？

问题来了：事件本身只有 $(x,y,t,p)$，没有绝对强度，那么 $\Delta\ln I$ 从哪里得到？

作者利用已经存在的 photometric depth map。

对于事件像素 $\mathbf u$，在时刻 $t$，首先根据深度反投影到 3D：

$$
\mathbf P_C(t)
=
\pi^{-1}(\mathbf u;Z(t)).
$$

然后通过事件相机到参考相机的变换：

$$
\mathbf T_{RC}(t),
$$

把三维点转换到参考相机坐标系：

$$
\mathbf P_R(t)
=
\mathbf T_{RC}(t)\mathbf P_C(t).
$$

最后投影到参考图像：

$$
\boxed{
\mathbf u'(t)
=
\pi
\left(
\mathbf T_{RC}(t)
\pi^{-1}
\left(
\mathbf u;Z(t)
\right)
\right).
}
$$

于是可以在参考强度图中查询：

$$
I^r(\mathbf u'(t)).
$$

对于当前事件时刻 $t_k$ 和同一像素的上一次事件时刻 $t_k-\Delta t$，分别得到：

$$
\mathbf u'(t_k),
\qquad
\mathbf u'(t_k-\Delta t).
$$

于是：

$$
\boxed{
\Delta\ln I
\approx
\ln I^r(\mathbf u'(t_k))
-
\ln I^r(\mathbf u'(t_k-\Delta t)).
}
$$

因此整条链路是：

$$
\boxed{
\text{位姿}
\rightarrow
\text{地图投影}
\rightarrow
\text{两个光度查询}
\rightarrow
\Delta\ln I
\rightarrow
M
}
$$

---

# 5. 当前位姿明明未知，如何计算 $\mathbf T_{RC}$？

这是我阅读时最先产生的疑问之一。

真实状态当然未知，但滤波器在第 $k$ 个事件到达前已经有一个**预测状态**：

$$
p(\mathbf s_k\mid o_{1:k-1}).
$$

其预测均值记为：

$$
\bar{\mathbf s}_k
=
E[
\mathbf s_k
\mid o_{1:k-1}
].
$$

因此实际计算 $\mathbf T_{RC}$ 时使用的是当前预测位姿：

$$
\bar{\xi}_{c,k},
$$

而不是未知真值。

如果：

- $\mathbf T_{WC}$ 表示事件相机在世界坐标系中的位姿；
- $\mathbf T_{WR}$ 表示参考相机的已知位姿；

那么一种常见约定下：

$$
\mathbf T_{RC}
=
\mathbf T_{WR}^{-1}
\mathbf T_{WC}.
$$

于是实际流程是：

$$
\boxed{
\text{上一轮状态估计}
\rightarrow
\text{当前预测位姿}
\rightarrow
\mathbf T_{RC}
\rightarrow
M_k
\rightarrow
\text{修正状态}
}
$$

这并不存在“必须先知道真值才能构造观测”的循环。  
本质上与 ICP、PnP、直接法、EKF 完全相同：**先用当前估计预测观测，再用观测残差修正当前估计。**

---

# 6. 参考帧频率低，会不会限制事件更新频率？

不会直接限制。

这篇论文里的参考图像不是在线输入的低频帧流，而是提前建立好的**静态地图**。

在线运行时，不是：

$$
\text{等新图像到来}
\rightarrow
\text{更新位姿},
$$

而是：

$$
\text{每来一个事件}
\rightarrow
\text{在静态参考地图中查询光度}
\rightarrow
\text{更新位姿}.
$$

因此事件更新频率与参考帧采集频率没有直接关系。

但是，这里有一个重要限制：

> 如果参考地图本身存在严重 motion blur，那么事件测量模型仍然会受到影响。

因为算法依赖：

$$
\nabla I^r,
\qquad
I^r(\mathbf u'),
$$

模糊会削弱梯度，并使预测的光度变化偏离真实事件生成过程。

所以更准确的说法是：

$$
\boxed{
\text{在线事件跟踪摆脱了帧率与在线运动模糊限制，}
\quad
\text{但仍依赖预建地图的光度质量。}
}
$$

---

# 7. 贝叶斯滤波：论文真正的骨架

论文把状态定义为：

$$
\mathbf s
=
\left(
\xi_c,\,
\xi_i,\,
\xi_j,\,
C_{\mathrm{th}},\,
\pi_m,\,
\sigma_m^2
\right)^\top.
$$

其中：

- $\xi_c$：当前位姿；
- $\xi_i,\xi_j$：用于插值历史时刻位姿的两个轨迹节点；
- $C_{\mathrm{th}}$：事件触发阈值；
- $\pi_m$：内点概率；
- $\sigma_m^2$：内点事件残差方差。

这里很有意思的一点是：

> 作者不仅估计运动状态，还把传感器模型参数一起放入状态。

系统要维护的不是单个最优位姿，而是后验：

$$
p(\mathbf s_k\mid o_{1:k}).
$$

贝叶斯滤波分成两个步骤。

## 7.1 Prediction

$$
p(\mathbf s_k\mid o_{1:k-1})
=
\int
p(\mathbf s_k\mid\mathbf s_{k-1})
p(\mathbf s_{k-1}\mid o_{1:k-1})
\,d\mathbf s_{k-1}.
$$

## 7.2 Correction

$$
\boxed{
p(\mathbf s_k\mid o_{1:k})
\propto
p(o_k\mid\mathbf s_k)
p(\mathbf s_k\mid o_{1:k-1}).
}
$$

一句话就是：

$$
\boxed{
\text{后验}
\propto
\text{当前观测似然}
\times
\text{预测先验}.
}
$$

---

# 8. 零均值随机扩散到底是什么？

论文没有采用常速度、常加速度或 IMU 驱动的运动模型，而是假设状态执行 slowly varying zero-mean random diffusion。

可以写成：

$$
\mathbf s_k
=
\mathbf s_{k-1}
+
\mathbf w_k,
$$

其中：

$$
\mathbf w_k
\sim
\mathcal N(\mathbf 0,\mathbf Q_k).
$$

因为：

$$
E[\mathbf w_k]=0,
$$

所以：

$$
\boxed{
\mu_k^-=\mu_{k-1}^+
}
$$

预测均值不变。

但是协方差增加：

$$
\boxed{
\mathbf P_k^-
=
\mathbf P_{k-1}^+
+
\mathbf Q_k.
}
$$

这就是“扩散”：

- 分布中心不主动移动；
- 不确定性范围逐渐扩大。

这并不表示相机被假设为静止，而是表示：

> 在没有新事件证据时，系统不知道状态应向哪个具体方向移动，只知道旧状态会逐渐变得不可信。

因此这篇论文中：

$$
\boxed{
\text{运动模型主要负责扩大不确定性，}
\quad
\text{事件观测主要负责推动状态均值。}
}
$$

---

# 9. $t_k-\Delta t$ 时刻的历史位姿怎么得到？

一个事件的生成依赖两个时刻：

$$
t_k
\quad\text{和}\quad
t_k-\Delta t.
$$

因此仅有当前位姿 $\xi_c$ 不够。

作者在轨迹中找到两个节点：

$$
\xi_i=\xi(t_i),
\qquad
\xi_j=\xi(t_j),
$$

满足：

$$
t_i
\le
t_k-\Delta t
\le
t_j.
$$

定义：

$$
\alpha
=
\frac{
(t_k-\Delta t)-t_i
}{
t_j-t_i
}.
$$

位置采用线性插值：

$$
\mathbf p(t_k-\Delta t)
=
(1-\alpha)\mathbf p_i
+
\alpha\mathbf p_j.
$$

旋转则对 exponential coordinates 分别做线性插值：

$$
\theta(t_k-\Delta t)
=
(1-\alpha)\theta_i
+
\alpha\theta_j.
$$

论文也提到更严格的 Lie group 插值可以写成：

$$
\mathbf T(t)
=
\mathbf T_i
\operatorname{Exp}
\left[
\alpha
\operatorname{Log}
\left(
\mathbf T_i^{-1}\mathbf T_j
\right)
\right],
$$

但计算代价更高。

由于 $\xi_i,\xi_j$ 也属于状态，它们并不是永久固定的。当前事件的残差对历史插值位姿也有导数，因此新事件能够对相关历史位姿做有限修正。

这有一点类似轻量 fixed-lag smoothing，但作者不会重新处理全部历史事件。

---

# 10. 从残差 $M$ 到似然

仅有：

$$
M=0
$$

还不是概率模型。

首先考虑理想传感器：

$$
p(o_k\mid\mathbf s_k)
=
\delta(M),
$$

即只有 $M=0$ 才有概率。

真实传感器有噪声，所以作者先将其放宽成高斯：

$$
\boxed{
p(o_k\mid\mathbf s_k)
=
\mathcal N
\left(
M(o_k,\tilde{\mathbf s}_k);
0,\sigma_m^2
\right).
}
$$

这一步非常关键。

它回答的是：

> 如果状态是 $\mathbf s_k$，当前事件产生这样的残差 $M$ 有多大可能？

因此：

$$
\boxed{
M
\text{ 是残差，}
\quad
p(M)
\text{ 才是概率模型。}
}
$$

---

# 11. 为什么还需要高斯-均匀混合模型？

事件流中存在大量离群事件，例如：

- 传感器背景噪声；
- 动态物体；
- 遮挡；
- 地图错误；
- 深度错误；
- 光照变化；
- 阈值波动。

如果所有事件都强制使用单高斯：

$$
p(M)=\mathcal N(M;0,\sigma_m^2),
$$

一个大残差会被解释成“当前位姿很错”，滤波器可能产生错误的大幅更新。

作者因此采用：

$$
\boxed{
p(o_k\mid\mathbf s_k)
=
\pi_m
\mathcal N(M_k;0,\sigma_m^2)
+
(1-\pi_m)
\mathcal U(M_k).
}
$$

可以引入潜变量：

$$
z_k=
\begin{cases}
1,&\text{内点}\\
0,&\text{离群点}
\end{cases}
$$

其中：

$$
P(z_k=1)=\pi_m.
$$

观察到当前残差后，当前事件是内点的后验概率为：

$$
\boxed{
w_k
=
\frac{
\pi_m\mathcal N(M_k;0,\sigma_m^2)
}{
\pi_m\mathcal N(M_k;0,\sigma_m^2)
+
(1-\pi_m)\mathcal U(M_k)
}.
}
$$

因此要区分：

- $\pi_m$：总体模型中的内点先验比例；
- $w_k$：当前这个具体事件的 posterior inlier probability。

---

# 12. 线性化点到底取在哪里？

论文对测量函数进行一阶线性化：

$$
M(o_k,\tilde{\mathbf s}_k)
\approx
M_k
+
\mathbf J_k
\Delta\tilde{\mathbf s}_k.
$$

其中线性化点是当前事件校正前的预测均值：

$$
\boxed{
\bar{\mathbf s}_k
=
E[
\mathbf s_k\mid o_{1:k-1}
].
}
$$

因此：

$$
M_k
=
M(o_k,\bar{\tilde{\mathbf s}}_k),
$$

$$
\mathbf J_k
=
\left.
\frac{\partial M}
{\partial\tilde{\mathbf s}}
\right|_{\bar{\tilde{\mathbf s}}_k},
$$

$$
\Delta\tilde{\mathbf s}_k
=
\tilde{\mathbf s}_k
-
\bar{\tilde{\mathbf s}}_k.
$$

这和普通 EKF 完全一样：

> 用预测状态作为线性化点，在其局部邻域中用一阶函数近似真实非线性观测模型。

---

# 13. 附录里的指数族重写到底在干什么？

线性化后：

$$
M
\approx
M_k+\mathbf J_k\Delta\tilde{\mathbf s}.
$$

也可以写成绝对状态形式：

$$
M
\approx
\mathbf J_k\tilde{\mathbf s}
+
\bar M_k,
$$

其中：

$$
\bar M_k
=
M_k-\mathbf J_k\bar{\tilde{\mathbf s}}_k.
$$

高斯内点项：

$$
\pi_m
\mathcal N
\left(
\mathbf J_k\tilde{\mathbf s}
+\bar M_k;
0,\sigma_m^2
\right)
$$

展开指数中的平方：

$$
(
\mathbf J_k\tilde{\mathbf s}
+\bar M_k
)^2
=
\tilde{\mathbf s}^\top
\mathbf J_k^\top\mathbf J_k
\tilde{\mathbf s}
+
2\bar M_k
\mathbf J_k\tilde{\mathbf s}
+
\bar M_k^2.
$$

所以所有依赖状态的项可以整理成：

$$
T(\mathbf s)
=
\left[
\frac{s_is_j}{\sigma_m^2},
\frac{s_i}{\sigma_m^2},
\frac{1}{\sigma_m^2},
\ln\sigma_m,
\ln\pi_m,
\ln(1-\pi_m)
\right].
$$

最终似然被改写成两个指数族项之和：

$$
p(o_k\mid\mathbf s_k)
=
\sum_j
h(\mathbf s_k)
\exp
\left(
\eta_{o,j}^\top
T(\mathbf s_k)
-
A_{o,j}
\right).
$$

这里并没有创造一个新的传感器模型。

它只是：

$$
\boxed{
\text{高斯内点 + 均匀离群}
\rightarrow
\text{指数族统一表示}
}
$$

目的是方便后续做 KL 后验近似。

---

# 14. 为什么需要近似后验？

标准贝叶斯更新：

$$
p(\mathbf s_k\mid o_{1:k})
\propto
p(o_k\mid\mathbf s_k)
p(\mathbf s_k\mid o_{1:k-1}).
$$

如果先验是高斯，而似然也是单高斯，在合适线性条件下后验仍然容易处理。

但这里似然是：

$$
\text{Gaussian}
+
\text{Uniform}.
$$

因此更新后的后验变成 mixture。

如果每个事件都有“内点/离群”两种分支，连续处理事件后理论分量数会：

$$
2,\ 4,\ 8,\ 16,\ldots
$$

指数增长。

因此作者始终只维护一个 tractable distribution：

$$
q(\mathbf s_k;\eta_k)
\approx
p(\mathbf s_k\mid o_{1:k}).
$$

论文式（6）可以更清楚地写成：

$$
\boxed{
q_k^+
\approx
C\,
p(o_k\mid\mathbf s_k)
q_k^-.
}
$$

其中：

- $q_k^-$：当前事件到达前的近似先验；
- $p(o_k\mid\mathbf s_k)$：当前事件似然；
- 右侧：贝叶斯更新后的复杂目标后验；
- $q_k^+$：重新压缩后的简单近似后验。

---

# 15. KL 散度到底做什么？

KL 散度定义为：

$$
D_{\mathrm{KL}}(f\|q)
=
\int
f(\mathbf s)
\ln
\frac{f(\mathbf s)}
{q(\mathbf s)}
d\mathbf s.
$$

作者想找到：

$$
\eta_k
=
\arg\min_\eta
D_{\mathrm{KL}}
\left(
f_k
\|
q(\cdot;\eta)
\right),
$$

其中：

$$
f_k(\mathbf s)
=
C\,
p(o_k\mid\mathbf s)
q_k^-(\mathbf s)
$$

是当前事件融合后的复杂目标后验。

所以 KL 的作用不是“替代似然”，而是：

$$
\boxed{
\text{似然负责产生新的后验信息，}
\quad
\text{KL负责把复杂后验压缩回简单分布。}
}
$$

---

# 16. 为什么 KL 最小化最终变成矩匹配？

指数族写成：

$$
q(\mathbf s;\eta)
=
h(\mathbf s)
\exp
\left(
\eta^\top T(\mathbf s)-A(\eta)
\right).
$$

最小化：

$$
D_{\mathrm{KL}}(f\|q_\eta)
$$

的一阶条件是：

$$
\boxed{
E_f[T(\mathbf s)]
=
E_q[T(\mathbf s)].
}
$$

也就是：

> 目标复杂分布与近似分布具有相同的充分统计量期望。

对于高斯分布：

$$
T(\mathbf x)
=
\{
\mathbf x,\,
\mathbf x\mathbf x^\top
\}.
$$

所以 KL 投影就是匹配：

$$
E_f[\mathbf x]
=
E_q[\mathbf x],
$$

$$
E_f[\mathbf x\mathbf x^\top]
=
E_q[\mathbf x\mathbf x^\top].
$$

也就是近似匹配：

$$
\boxed{
\text{均值 + 协方差}.
}
$$

这也是为什么作者不需要真的每次都数值积分 KL 并做通用优化。

---

# 17. 一个直观的 KL 近似例子

假设真实目标后验是：

$$
f(x)
=
0.8\mathcal N(x;1,0.25)
+
0.2\mathcal N(x;5,1).
$$

要用单个高斯：

$$
q(x)=\mathcal N(x;\mu,\sigma^2)
$$

近似。

KL 最优高斯的均值：

$$
\mu
=
0.8\times1+0.2\times5
=
1.8.
$$

方差：

$$
\sigma^2
=
0.8[
0.25+(1-1.8)^2
]
+
0.2[
1+(5-1.8)^2
]
=
2.96.
$$

因此：

$$
q(x)
=
\mathcal N(x;1.8,2.96).
$$

原始分布有两个峰，单高斯无法保留多峰结构，但会通过增大方差覆盖重要概率区域。

这就是：

$$
\boxed{
\text{复杂概率形状}
\rightarrow
\text{保留主要一、二阶统计信息}
}
$$

---

# 18. 最终为什么得到一个加权 EKF？

作者对多元高斯位姿后验进行显式推导后，得到：

$$
K_k
=
P_kJ_k^\top
\left(
J_kP_kJ_k^\top+\sigma_m^2
\right)^{-1},
$$

$$
w_k
=
\frac{
\pi_m\mathcal N(M_k;0,\sigma_m^2)
}{
\pi_m\mathcal N(M_k;0,\sigma_m^2)
+
(1-\pi_m)\mathcal U
},
$$

$$
\xi_{k+1}
=
\xi_k+w_kK_kM_k,
$$

$$
P_{k+1}
=
(I-w_kK_kJ_k)P_k.
$$

论文采用自己的残差和增量符号约定，因此状态更新式中写为 $+w_kK_kM_k$；在常见 implicit residual 写法 $0\approx M+J\delta x$ 中，也经常把创新定义为 $-M$，两者只要符号约定前后一致即可。

这里两个权重的作用不同：

### $K_k$：Kalman gain

由：

- 状态不确定性 $P_k$；
- 测量灵敏度 $J_k$；
- 测量噪声 $\sigma_m^2$；

共同决定。

### $w_k$：当前事件可信度

回答：

> 当前事件更像正常事件还是离群事件？

因此：

$$
\boxed{
K_k
\text{ 决定“如果事件可信，该信多少”；}
}
$$

$$
\boxed{
w_k
\text{ 决定“这个事件本身是否值得信”。}
}
$$

最终有效增益为：

$$
w_kK_k.
$$

---

# 19. $\pi_m$、$\sigma_m^2$ 和 $C_{\mathrm{th}}$ 如何理解？

这三项都是传感器模型参数，但含义不同。

## $C_{\mathrm{th}}$

直接进入：

$$
M
=
\frac{\Delta\ln I}{C_{\mathrm{th}}}-1.
$$

所以：

$$
\frac{\partial M}
{\partial C_{\mathrm{th}}}
=
-\frac{\Delta\ln I}
{C_{\mathrm{th}}^2}.
$$

它可以像其他状态变量一样通过测量 Jacobian 被修正。

## $\pi_m$

表示总体内点比例：

$$
P(z_k=1)=\pi_m.
$$

它与单个事件的：

$$
w_k=P(z_k=1\mid M_k)
$$

不是一回事。

## $\sigma_m^2$

描述正常事件残差：

$$
M\mid z_k=1
\sim
\mathcal N(0,\sigma_m^2).
$$

论文把 $\pi_m$、$\sigma_m^2$ 纳入指数族状态与后验参数中，但正文对它们没有像位姿那样给出非常直观的独立递推式。因此从论文阅读角度，更重要的是理解：

$$
\boxed{
\pi_m,\sigma_m^2
\text{ 不是简单固定超参数，而是被纳入概率模型联合估计。}
}
$$

---

# 20. 论文实验真正证明了什么？

我认为这篇论文实验最值得借鉴的不是某一个数字，而是它的论证层次。

## 20.1 正常运动下先证明基础精度

室内 8 条序列上，事件方法平均：

- position RMS：$1.63$ cm；
- orientation RMS：$2.21^\circ$。

标准帧相机方法约为：

- position RMS：$1.08$ cm；
- orientation RMS：$1.04^\circ$。

因此论文并没有证明“事件相机正常情况下更准”，而是证明：

> 即使 DVS 空间分辨率明显更低，逐事件跟踪仍能获得具有竞争力的基础精度。

这是非常合理的实验逻辑：先证明方法正常情况下不是“只能工作但误差很大”。

## 20.2 再证明高速运动优势

高速阶段，帧方法由于 motion blur 在大约 $8.66$ s 后失去跟踪，而事件方法继续输出有效位姿。

这是论文最有力的证据链：

$$
\boxed{
\text{相同运动}
\rightarrow
\text{帧图像模糊}
\rightarrow
\text{帧方法失效}
\rightarrow
\text{事件方法继续工作}
}
$$

相比只报告一个平均 RMSE，这种“机理—现象—性能”的闭环更有说服力。

## 20.3 自然室外场景

ivy / graffiti / building 三组室外序列的 translation RMS 相对于平均深度分别约为：

- $4.37\%$；
- $5.88\%$；
- $7.40\%$。

building 场景中还有行人产生大量动态事件，系统仍能持续跟踪。

## 20.4 大深度变化与遮挡

boxes、pipe、bicycles 场景用于说明方法不只是近似平面场景特例，还能处理：

- 明显深度变化；
- 更强视差；
- 遮挡；
- 局部弱纹理。

## 20.5 计算效率

单核 CPU 单事件处理时间约：

$$
32\ \mu s,
$$

约为：

$$
31,000\ \text{events/s}.
$$

这证明闭式逐事件更新具有较低单事件计算成本，但也暴露出一个限制：极高速情况下事件率可能远高于该处理能力。

---

# 21. 实验没有充分证明的部分

从今天的论文标准看，这篇工作理论很完整，但模块级消融并不充分。

它没有严格量化：

- 高斯-均匀混合似然相比单高斯到底提升多少；
- KL 近似相比其他后验近似策略提升多少；
- 在线估计 $C_{\mathrm{th}},\pi_m,\sigma_m^2$ 的独立收益；
- 地图深度误差对系统的影响；
- 参考强度图 motion blur 对系统的影响。

所以它更像是在证明：

$$
\boxed{
\text{完整系统可行，并且事件相机的高速优势确实能够体现。}
}
$$

而不是通过大量消融证明每个理论模块分别贡献了多少。

---

# 22. 抛开论文：贝叶斯状态估计的 general 范式

读完这篇论文以后，一个更重要的认识是：

> “传播状态不确定性 + 构造观测似然 + 更新后验”是非常 general 的状态估计范式。

考虑状态：

$$
\mathbf x_k,
$$

状态演化：

$$
\mathbf x_k
=
f(\mathbf x_{k-1},\mathbf u_k)
+
\mathbf w_k,
$$

对应：

$$
p(\mathbf x_k\mid\mathbf x_{k-1},\mathbf u_k).
$$

观测模型：

$$
\mathbf z_k
=
h(\mathbf x_k)+\mathbf v_k,
$$

或隐式形式：

$$
\mathbf r(\mathbf z_k,\mathbf x_k)=0.
$$

对应似然：

$$
p(\mathbf z_k\mid\mathbf x_k).
$$

然后：

$$
\boxed{
p(\mathbf x_k\mid\mathbf z_{1:k})
\propto
p(\mathbf z_k\mid\mathbf x_k)
p(\mathbf x_k\mid\mathbf z_{1:k-1}).
}
$$

事件相机论文只是选择了：

$$
\mathbf r=M
=
\frac{\Delta\ln I}{C_{\mathrm{th}}}-1.
$$

这个残差完全可以替换。

---

# 23. residual、likelihood 和 loss 到底是什么关系？

这是我认为最需要彻底分开的三个概念。

假设构造残差：

$$
r(\mathbf z,\mathbf x).
$$

## 第一层：Residual

$$
\boxed{
r(\mathbf z,\mathbf x)
}
$$

回答：

> 什么叫“预测与观测不一致”？

例如：

- 重投影误差；
- 光度误差；
- 点到平面误差；
- IMU 预积分误差；
- 事件对比度残差 $M$。

## 第二层：Noise model / likelihood

假设：

$$
r\sim\mathcal N(0,\Sigma),
$$

则：

$$
p(\mathbf z\mid\mathbf x)
\propto
\exp
\left[
-\frac12
r^\top\Sigma^{-1}r
\right].
$$

回答：

> 不同大小的残差应该有多可能？

## 第三层：Loss

取负对数：

$$
\boxed{
\mathcal L(\mathbf x)
=
-\log p(\mathbf z\mid\mathbf x).
}
$$

高斯情况下：

$$
\mathcal L
=
\frac12
r^\top\Sigma^{-1}r
+\text{const}.
$$

所以：

$$
\boxed{
\text{Residual}
\rightarrow
\text{Probability distribution}
\rightarrow
\text{Likelihood}
\rightarrow
-\log
\rightarrow
\text{Loss}.
}
$$

论文中的 $M$ 本身不是 loss。

---

# 24. 换一个残差，概率框架还成立吗？

当然可以。

例如特征法：

$$
r_{ij}
=
z_{ij}
-
\pi(T_iP_j).
$$

直接法：

$$
r_{ij}
=
I_j(
\pi(T_{ji}\pi^{-1}(u_i,D_i))
)
-
I_i(u_i).
$$

LiDAR ICP：

$$
r_i
=
n_i^\top
(Tp_i-q_i).
$$

IMU：

$$
r_{ij}^{\mathrm{imu}}
=
[
r_R,\,
r_v,\,
r_p,\,
r_b
]^\top.
$$

事件相机还可以构造：

- event-to-edge residual；
- time-surface photometric residual；
- geometric reprojection residual；
- contrast maximization objective；
- event rate residual；
- optical-flow residual。

真正需要回答的是：

$$
\boxed{
\text{在正确状态下，这个残差应当服从什么统计分布？}
}
$$

如果这一点有合理的物理或统计依据，就可以自然地进入贝叶斯似然模型。

---

# 25. 不是所有 metric 都天然是 likelihood

这一点同样重要。

像：

- 熵；
- 结构得分；
- 特征数量；
- 梯度能量；
- 网络置信度；
- 匹配数量；

这些指标可以非常适合作为：

$$
\text{utility / surrogate objective},
$$

但不一定天然是概率残差。

例如一个结构分数：

$$
S(\tau)
$$

如果目标是：

$$
\tau^*
=
\arg\max_\tau S(\tau),
$$

它首先是一个**决策效用函数**。

当然可以定义 energy-based probability：

$$
p(\text{data}\mid\tau)
\propto
\exp(\beta S(\tau)),
$$

但必须进一步回答：

- 为什么这个指数形式合理？
- $\beta$ 如何确定？
- 分布是否可归一化？
- $S$ 是否经过统计校准？
- 它表示真实 data likelihood，还是只是任务代理？

所以：

$$
\boxed{
\text{一个好用的优化指标，不自动等价于严格的概率似然。}
}
$$

---

# 26. 从滤波转成优化，概率模型并没有变

这篇论文最后使用的是 EKF-like filter，但并不意味着同一个模型只能滤波求解。

完整轨迹后验可以写成：

$$
p(\mathbf x_{0:K}\mid\mathbf z_{1:K}).
$$

贝叶斯公式：

$$
p(\mathbf x_{0:K}\mid\mathbf z_{1:K})
\propto
p(\mathbf z_{1:K}\mid\mathbf x_{0:K})
p(\mathbf x_{0:K}).
$$

在一阶 Markov 假设下：

$$
p(\mathbf x_{0:K})
=
p(\mathbf x_0)
\prod_{k=1}^{K}
p(\mathbf x_k\mid\mathbf x_{k-1}).
$$

在观测条件独立假设下：

$$
p(\mathbf z_{1:K}\mid\mathbf x_{0:K})
=
\prod_{k=1}^{K}
p(\mathbf z_k\mid\mathbf x_k).
$$

因此：

$$
\boxed{
p(\mathbf x_{0:K}\mid\mathbf z_{1:K})
\propto
p(\mathbf x_0)
\prod_{k=1}^{K}
p(\mathbf x_k\mid\mathbf x_{k-1})
\prod_{k=1}^{K}
p(\mathbf z_k\mid\mathbf x_k).
}
$$

这条公式可以看成现代概率 SLAM 的“母公式”。

---

# 27. 上面的轨迹后验是怎么推出来的？

首先根据贝叶斯：

$$
p(\mathbf x_{0:K}\mid\mathbf z_{1:K})
=
\frac{
p(\mathbf z_{1:K}\mid\mathbf x_{0:K})
p(\mathbf x_{0:K})
}{
p(\mathbf z_{1:K})
}.
$$

由于：

$$
p(\mathbf z_{1:K})
$$

与待估状态无关：

$$
p(\mathbf x_{0:K}\mid\mathbf z_{1:K})
\propto
p(\mathbf z_{1:K}\mid\mathbf x_{0:K})
p(\mathbf x_{0:K}).
$$

状态联合概率使用链式法则：

$$
\begin{aligned}
p(\mathbf x_{0:K})
={}&
p(\mathbf x_0)
p(\mathbf x_1\mid\mathbf x_0)\\
&\cdot
p(\mathbf x_2\mid\mathbf x_0,\mathbf x_1)
\cdots.
\end{aligned}
$$

引入一阶 Markov 假设：

$$
p(\mathbf x_k\mid\mathbf x_{0:k-1})
=
p(\mathbf x_k\mid\mathbf x_{k-1}),
$$

得到：

$$
p(\mathbf x_{0:K})
=
p(\mathbf x_0)
\prod_k
p(\mathbf x_k\mid\mathbf x_{k-1}).
$$

同理，对观测假设：

$$
p(\mathbf z_k\mid
\mathbf x_{0:K},
\mathbf z_{1:k-1})
=
p(\mathbf z_k\mid\mathbf x_k),
$$

于是：

$$
p(\mathbf z_{1:K}\mid\mathbf x_{0:K})
=
\prod_k
p(\mathbf z_k\mid\mathbf x_k).
$$

两者代回就得到最终结果。

---

# 28. 为什么这会直接变成 factor graph？

对后验求 MAP：

$$
\mathbf x_{0:K}^*
=
\arg\max_{\mathbf x_{0:K}}
p(\mathbf x_{0:K}\mid\mathbf z_{1:K}).
$$

代入概率分解：

$$
\mathbf x^*
=
\arg\max
p(\mathbf x_0)
\prod_k
p(\mathbf x_k\mid\mathbf x_{k-1})
\prod_k
p(\mathbf z_k\mid\mathbf x_k).
$$

取负对数：

$$
\boxed{
\begin{aligned}
\mathbf x^*
=
\arg\min
\quad&
-\log p(\mathbf x_0)\\
&-
\sum_k
\log p(\mathbf x_k\mid\mathbf x_{k-1})\\
&-
\sum_k
\log p(\mathbf z_k\mid\mathbf x_k).
\end{aligned}
}
$$

如果都是高斯：

$$
p(\mathbf x_k\mid\mathbf x_{k-1})
\propto
\exp
\left[
-\frac12
\|r_k^{\mathrm{motion}}\|_{Q_k^{-1}}^2
\right],
$$

$$
p(\mathbf z_k\mid\mathbf x_k)
\propto
\exp
\left[
-\frac12
\|r_k^{\mathrm{meas}}\|_{R_k^{-1}}^2
\right].
$$

于是：

$$
\boxed{
\min_{\mathbf x_{0:K}}
\|
r_0
\|_{P_0^{-1}}^2
+
\sum_k
\|
r_k^{\mathrm{motion}}
\|_{Q_k^{-1}}^2
+
\sum_k
\|
r_k^{\mathrm{meas}}
\|_{R_k^{-1}}^2.
}
$$

这就是 SLAM 中熟悉的非线性最小二乘。

所以：

$$
\boxed{
\text{贝叶斯概率分解}
\xrightarrow{-\log}
\text{因子图 / 最小二乘优化}.
}
$$

---

# 29. 滤波与优化不是两个完全不同的世界

对于同一个概率模型：

$$
p(x_k\mid x_{k-1}),
\qquad
p(z_k\mid x_k),
$$

可以选择不同推断方法。

| 方法 | 核心思想 |
|---|---|
| EKF | 递归维护当前近似高斯后验 |
| Particle Filter | 用样本表示后验 |
| Sliding-window MAP | 显式保留一段状态并重复优化 |
| Batch MAP | 一次优化完整轨迹 |
| Factor Graph / Smoothing | 利用概率因子的稀疏结构高效求解 |
| Variational Inference | 用受限分布族近似复杂后验 |

因此：

$$
\boxed{
\text{滤波 vs 优化}
\text{通常是推断策略的区别，}
\text{而不一定是概率建模本身的区别。}
}
$$

---

# 30. 论文里的 $M$ 如果放进优化框架会怎样？

完全可以。

事件因子可以写成：

$$
\phi_k^{\mathrm{event}}
=
p(o_k\mid X).
$$

如果仍使用论文的高斯-均匀混合模型：

$$
\phi_k^{\mathrm{event}}
=
\pi_m
\mathcal N(M_k(X);0,\sigma_m^2)
+
(1-\pi_m)\mathcal U.
$$

那么优化目标就是：

$$
\boxed{
\mathcal L_{\mathrm{event}}
=
-\sum_k
\log
\left[
\pi_m
\mathcal N(M_k(X);0,\sigma_m^2)
+
(1-\pi_m)\mathcal U
\right].
}
$$

它可以和其他因子共同进入：

$$
\mathcal L
=
\mathcal L_{\mathrm{prior}}
+
\mathcal L_{\mathrm{IMU}}
+
\mathcal L_{\mathrm{event}}
+
\mathcal L_{\mathrm{stereo}}.
$$

所以这篇论文的事件 residual 并不天然属于 EKF。

它本质上定义了一个：

$$
\boxed{
\text{event measurement factor}.
}
$$

后面可以接 EKF，也可以接 sliding-window optimization、factor graph 或其他 MAP estimator。

---

# 31. 滤波中的协方差传播，在优化里去了哪里？

滤波中：

$$
P_k^-
=
F_kP_{k-1}^+F_k^\top
+
Q_k.
$$

显式传播不确定性。

优化中，状态演化写成运动残差：

$$
r_k^{\mathrm{motion}}
=
x_k
-
f(x_{k-1},u_k),
$$

并由：

$$
Q_k^{-1}
$$

加权：

$$
\|
r_k^{\mathrm{motion}}
\|_{Q_k^{-1}}^2.
$$

所以：

$$
\boxed{
Q
\text{ 在滤波中控制 covariance propagation，}
}
$$

$$
\boxed{
Q^{-1}
\text{ 在 MAP 优化中控制 motion factor 的信息权重。}
}
$$

两者来自完全相同的：

$$
p(x_k\mid x_{k-1}).
$$

---

# 32. 几组最容易混淆的概念

| 概念 A | 概念 B | 区别 |
|---|---|---|
| residual $M$ | loss | residual 描述“不一致是什么”；loss 描述“如何惩罚不一致” |
| $\pi_m$ | $w_k$ | $\pi_m$ 是总体内点先验；$w_k$ 是当前事件的后验内点概率 |
| $Q$ | $\sigma_m^2$ | $Q$ 是过程噪声；$\sigma_m^2$ 是观测残差噪声 |
| prediction | correction | prediction 使用运动模型；correction 使用新观测似然 |
| prior | posterior | prior 是观测到来前的 belief；posterior 是融合观测后的 belief |
| map frame rate | event update rate | 参考图是静态地图，不限制逐事件更新频率 |
| approximate posterior | approximate likelihood | 论文主要近似的是复杂后验，而不是把似然直接丢掉 |
| filter | optimization | 通常是不同推断策略，不必对应不同概率模型 |

---


# 33. 对事件相机研究的进一步启发

从这篇论文往更加 general 的方向推，可以看到事件 SLAM 中一个很自然的研究空间：

$$
\boxed{
\text{事件观测}
\rightarrow
\text{物理/几何 residual}
\rightarrow
\text{probabilistic likelihood}
\rightarrow
\text{状态估计}
}
$$

这里可以改变的并不只是求解器。

可以研究：

- 更合理的事件生成噪声模型；
- polarity-dependent likelihood；
- pixel-dependent contrast threshold；
- 异步 stereo correspondence likelihood；
- history-aware association probability；
- depth uncertainty；
- event representation uncertainty；
- temporal support $\tau$ 的概率建模；
- measurement covariance 的在线估计；
- state estimator 与 representation parameter 的联合优化。

真正值得思考的并不是：

> “把一个新的 loss 塞进 SLAM 后端。”

而是：

> **这个量究竟描述了哪个随机变量？它对应什么条件概率？正确状态下为什么应该服从这个分布？**

一旦这个问题回答清楚，滤波、优化、因子图、可微状态估计都只是后续求解方式。

---

# 34. 总结

这篇论文表面上是一篇事件相机 6-DoF tracking 工作，但从状态估计角度看，它最有价值的地方是完整展示了一条概率建模链路：

$$
\boxed{
\text{事件生成物理模型}
\rightarrow
M
\rightarrow
p(o_k\mid s_k)
\rightarrow
p(s_k\mid o_{1:k})
\rightarrow
\text{KL projection}
\rightarrow
\text{weighted EKF}.
}
$$

而继续抽象以后，又可以得到整个机器人概率状态估计的统一框架：

$$
\boxed{
\text{State}
+
\text{Transition Probability}
+
\text{Measurement Likelihood}
=
\text{Posterior}.
}
$$

对于完整轨迹：

$$
\boxed{
p(x_{0:K}\mid z_{1:K})
\propto
p(x_0)
\prod_{k=1}^{K}
p(x_k\mid x_{k-1})
\prod_{k=1}^{K}
p(z_k\mid x_k).
}
$$

滤波是在递归近似这个 posterior；  
MAP / factor graph 是对同一个 posterior 取最大值；  
最小二乘则是在高斯假设下对负对数后验进行优化。

最终我认为最值得记住的是三句话：

$$
\boxed{
\text{Residual 决定“什么叫不一致”。}
}
$$

$$
\boxed{
\text{Likelihood 决定“不一致应该被怎样概率化”。}
}
$$

$$
\boxed{
\text{Filter / Optimizer 决定“怎样从这些概率约束中得到状态”。}
}
$$

---

# Reference

Guillermo Gallego, Jon E. A. Lund, Elias Mueggler, Henri Rebecq, Tobi Delbruck, and Davide Scaramuzza, **“Event-Based, 6-DOF Camera Tracking from Photometric Depth Maps,”** *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 40, no. 10, pp. 2402–2412, 2018. DOI: 10.1109/TPAMI.2017.2769655.
