---
layout: post
title:  "Design of an Intelligent Car Based on Visual Guidance"
info: "The 14th NXP Cup Intelligent Vehicle Competition for National University Students (Visual Four-wheel Racing)"
tech: "Outcomes: Second Prize in the Four-wheel Vehicle Category of the 14th NXP Cup Intelligent Vehicle Competition for National Students (East China Region) "
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

This is a competition project in the National University Student Intelligent Car Contest. The competition requires the intelligent car to be able to operate on any designed track and increase its speed as much as possible. The intelligent car uses rear-wheel drive, with front-wheel servo control for steering. It uses a camera to collect real-time road image information and send it back to the MCU for image processing. The MCU then finds the road centerline based on the processed image and controls the intelligent car to drive on the track.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Mar. 2019 - Jul. 2019

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

### Competitiion record:

<video width="640" height="480" controls>
    
    <source src="https://effun.xyz/assets/img/20190308/比赛-恩智浦华东四轮.mp4" type="video/mp4">

</video>

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/gallery/微信图片_20240906151113.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>