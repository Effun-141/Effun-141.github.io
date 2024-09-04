---
layout: post
title:  "Design of a Navigation and Localization System for an Autonomous Harvesting Robot in a Simulated Farmland Environment"
info: "Design a mobile robot with mapping and autonomous waypoint navigation capabilities."
tech : "Outcomes:"
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

1. Solve the problem of 2D map construction for mobile robots.
2. Implement autonomous navigation using prior maps.
3. Address waypoint data interaction issues.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Oct. 2023 - Nov. 2023

## Contributions

1. Deployed a 2D LiDAR SLAM algorithm based on Cartographer in the mobile robot, adapted to the robot's LiDAR for 2D map creataion.
2. Designed autonomous navigation module using AMCL and move_base, established and managed TF transformations. Implemented A* and DWA algorithms for path planning, and tuned parameters for the robot and test environment.
3. Designed a ROS topic-based data interaction program to subscribe odometry data, publish target linear/angular velocities to mobile base, and waypoints to navigation module.


## Outcomes
 
Developed a complete navigation and localization system for a mobile robot, enabling it to create 2D maps and autonomously cruise along designated waypoints 

## Project Showcase

### Robot used for experiments:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="" width="90%" />
				<br/>
				<font color="AAAAAA">Front</font>
			</center>
		</td>
		<td>
			<center>
				<img src="" width="90%" />
				<br/>
				<font color="AAAAAA">Back</font>
			</center>
		</td>
	</tr>
</table>


### Partial core code

`
1
`

### Experimental processing and results:

