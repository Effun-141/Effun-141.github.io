---
layout: post
title:  "Design of a Laser SLAM System Based on LeGO-LOAM."
info: "Learned the principles of Lego-LOAM and replicated it, completing the related course report."
tech: "Outcomes: A course report that received a score of 98, ranking 1st out of 90 in the class."
type: Brief introduction
---

## Introduction

This is a course I took during my graduate studies, titled Robot Control Theory and Technology. The final assessment involved the teacher presenting multiple robotics-related topics, from which students could choose one to conduct experiments and write a course paper. I selected a topic related to laser SLAM, and the specific requirements are as follows:

![fig]( https://effun.xyz/assets/img/20230501/1.png )

#### &#127774; Aims: 

1. Complete a LiDAR-based SLAM program under the ROS system
2. Discuss and compared current centralized LiDAR SLAM methods
3. Write a technical report

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: May. 2023 - Jun. 2023

## Contributions

1. Designed a system based on the mainstream open-source LiDAR SLAM framework LeGO-LOAM and ROS, becoming familiar with both the system principles and code.
2. Added a pose-saving function to LeGO-LOAM for comparison with ground truth, based on an in-depth understanding of the code.
3. Set up the required software environment for experiments, including installing Ubuntu 18.04, ROS-melodic, and dependencies like Gtsam, PCL, and OpenCV.
4. Tested the system using the KITTI dataset, viewing the generated 3D point cloud map in real-time in RViz.
5. Compared ground truth poses with estimated poses using the evo tool.

## Outcomes
 
A course report that received a score of 98, ranking 1/90 in the class, which can be access at the following link: 

<a href='https://effun.xyz/assets/img/20230501/1.pdf?spm=1001.2014.3001.5502'><img src="https://img.shields.io/badge/-course report-blue?logo=Git&logoColor=white"></a>



## Project Showcase

### Robot used for experiments:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20240318/1 (1).jpg" width="90%" />
				<br/>
				<font color="AAAAAA">Front</font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20240318/1 (2).jpg" width="90%" />
				<br/>
				<font color="AAAAAA">Back</font>
			</center>
		</td>
	</tr>
</table>


### Experimental processing and results:
