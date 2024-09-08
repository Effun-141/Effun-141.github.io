---
layout: post
title:  "Design of a Vehicle-Type Exploration and Cruising Robot"
info: "The 14th NXP Cup Intelligent Vehicle Competition for National University Students (Visual Four-wheel Racing)"
tech: "Outcomes: Second Prize in the Four-wheel Vehicle Category of the 14th NXP Cup Intelligent Vehicle Competition for National Students (East China Region) "
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

This is a competition project in the National University Student Intelligent Car Contest. The competition requires the intelligent car to be able to operate on any designed track and increase its speed as much as possible. The intelligent car uses rear-wheel drive, with front-wheel servo control for steering. It uses a camera to collect real-time road image information and send it back to the MCU for image processing. The MCU then finds the road centerline based on the processed image and controls the intelligent car to drive on the track.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Oct. 2023 - Nov. 2023

## Contributions

1. Image preprocessing, including image binarization and mean filtering.
2. Based on the principle of inverse perspective transformation, establish the correspondence between the world coordinate system and the pixel coordinate system to achieve better control effect.
3. Use PI (Proportional-Integral) algorithm based on integral separation for motor closed-loop control.
4. Apply PD (Proportional-Derivative) algorithm based on incomplete differentiation for servo control.



## Conclusion

The designed visually guided intelligent vehicle is able to run well on the track with a maximum speed of 2m/s.

## Outcomes
 
Second Prize in the Four-wheel Vehicle Category of the 14th NXP Cup Intelligent Vehicle Competition for National Students (East China Region)

## Project Showcase

### The process of robot making:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151349.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151348.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151346.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Display of our robot:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151343.jpg" width="100%" />
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
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151342.jpg" width="100%" />
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
				<img src="https://effun.xyz/assets/img/20191020/微信图片_20240906151345.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Competition record:

<video width="480" height="270" controls>
    
    <source src="https://effun.xyz/assets/img/20191020/1.mp4" type="video/mp4">

</video>