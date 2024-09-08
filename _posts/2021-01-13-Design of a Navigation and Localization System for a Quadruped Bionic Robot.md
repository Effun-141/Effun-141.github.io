---
layout: post
title:  "Design of a Navigation and Localization System for a Quadruped Bionic Robot"
info: "Researched Visual Simultaneous Localization and Mapping (VSLAM) techniques for undergraduate thesis project."
tech: "Outcomes: An undergraduate thesis that won the second prize for excellent graduation design in Jiangsu Province."
type: Brief introduction
---

## Introduction

This is the title of my undergraduate thesis. The main research focus is on vision-based Simultaneous Localization and Mapping (SLAM) technology. This project was my first SLAM topic and marked the beginning of my strong interest in SLAM. In this project, I engaged in academic writing for the first time and encountered a series of new concepts, including the Ubuntu operating system, Robot Operating System (ROS), PCL, G2O, and more (prior to this, I primarily used microcontrollers for bare-metal embedded development). 

It is no exaggeration to say that this project significantly enhanced my overall capabilities. This was not only because I learned a new algorithm (ORB-SLAM2), but also because I ventured into a new field of robotics development, learned how to conduct academic writing, and updated my "knowledge base" in engineering implementation. Most importantly, this project taught me how to analyze and solve problems, providing me with a reasonable approach to tackle various future projects. By the way, my thesis based on this project received the Second Prize for Excellent Graduation Design in Jiangsu Province (ranking first among over 400 students in my college), marking a perfect conclusion to my undergraduate studies.

#### &#127774; Aims: 

1. Familiarize yourself with the principles and code of an open-source visual SLAM algorithm and replicate it in the quadruped robot (Yushu Technology A1).
2. Achieve indoor localization accuracy of 5 cm and outdoor localization accuracy of 0.5 m for the robot.
3. Creat the map ofexperimental scenes.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Jan. 2021 - Jun. 2021

## Contributions

1. Learned visual SLAM fundamental theories and ORB-SLAM2 source code.
2. Reproduced ORB-SLAM2 for indoor/outdoor high-precision localization using TUM dataset and D435i camera.
3. Developed dense point cloud and octree mapping programs using ROS, OpenCV, and PCL for environment perception and navigation.

## Conclusion

1. Concluded that the designed navigation and localization system can achieve indoor localization accuracy of less than 5cm and outdoor accuracy of less than 0.3m.
2. Produced the point cloud map and the octree map.


## Outcomes
 
An undergraduate thesis that won the second prize for excellent graduation design in Jiangsu Province.

## Project Showcase

### Robot used for experiments:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/微信图片_20240908173151.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Experimental processing and results:

#### ORB feature points extraction and matching:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/筛后.png" width="100%" />
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
				<img src="https://effun.xyz/assets/img/20210113/2021-03-21 12-04-06 的屏幕截图.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

#### Experiment under TUM dataset:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/2021-05-07 10-48-21 的屏幕截图.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

#### Experiment in real-world environment:

#### Indoor scene:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/10C0802F3F087825431C7B6603F1505A.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/7E14CF3A52AD30588098B3D2C882BFE1.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20210113/DBBEB6C8D782568ED1B022148B9EE106.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>