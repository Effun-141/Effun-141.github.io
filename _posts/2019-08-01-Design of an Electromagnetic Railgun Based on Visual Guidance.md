---
layout: post
title:  "Design of an Electromagnetic Railgun Based on Visual Guidance"
info: "This is the control category problem of the 2019 National Undergraduate Electronic Design Competition."
tech: "Outcomes: First Prize of National Student Electronic Design Competition (Jiangsu Division) "
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

The positions of the railgun and the circular target are shown in the diagram. The railgun is placed at the calibration point, with the initial horizontal direction of the barrel aligned with the central axis at an angle of 0° and the vertical elevation angle also at 0°. The circular target is placed horizontally on the ground, with its bullseye located at a distance of 200 cm < d < 300 cm from the calibration point and at an angle of -30° < α < 30° relative to the central axis.

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20190801/curved-fire-gun-1.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

1. 
2. 
3. 

研究内容：电磁炮炮身由一个两自由度云台控制，使用OpenMV作为视觉传感器采集目标信息设计了一个DC-AC逆变电路模块和整流模块为电磁炮充能，可控硅控制充电时间与充电量。电磁炮具有手动和自动两种工作模式，手动模式下，根据输入的靶心位置坐标击发弹丸；自动模式下，OpenMV识别红色标志确定靶心位置坐标，自动击发弹丸。

研究方法：主要负责系统硬件搭建和算法实现，在OpenMV IDE中使用Python语言编写了红色标靶的识别程序、测距程序以及单片机与OpenMV的通信程序。利用MATLAB和EXCEL拟合出弹丸出射距离与充电电压之间的关系，得到弹道射表关系曲线，确保命中靶心。


#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Aug. 2019 - Oct. 2019

## Contributions

1. 
2. 
3. 
4. 

标靶识别算法实现。基于OpenMV，使用Python语言编写了红色标靶的识别程序、测距程序以及单片机与OpenMV的通信程序。
利用MATLAB和EXCEL拟合出弹丸出射距离与充电电压之间的关系，得到弹道射表关系曲线，确保命中靶心。

## Outcomes
 
 First Prize of National Student Electronic Design Competition (Jiangsu Division)

## Project Showcase

### The electromagnetic railgun and competition site:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20190801/微信图片_20240906152047.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20190801/微信图片_20240906152055.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

We refined our electromagnetic railgun in 2023.

<video width="640" height="480" controls>
    
    <source src="https://effun.xyz/assets/img/20190801/curved-fire-gun-1.mp4" type="video/mp4">

</video>
