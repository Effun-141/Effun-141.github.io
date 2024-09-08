---
layout: post
title:  "Design and Implementation of a Multi-AGV Warehouse Management System"
info: "Design of three optically guided model AGVs to simulate the warehouse management scheduling process."
tech: "Outcomes: Two software copyright; A complete multi-AGV system."
type: Brief introduction
---

## Introduction

This was a provincial-level research training project and also one of the required courses during the undergraduate stage. The project addressed issues in traditional warehouse management, such as high human resource investment, low efficiency in goods transportation, and high costs for inbound and outbound logistics, by developing a multi-AGV-based warehouse management system. The final outcomes of the project included a complete simulated AGV warehouse management system and two software copyrights. I ultimately scored 90 in the course.

#### &#127774; Aims: 

1. Design three vision-based model AGVs to operate on a grid map of a simulated logistics warehouse.
2. Develop the autonomous line following, motion control, path planning, task scheduling, and communication functions for the model AGVs.


#### &#128221; Advisor: Prof. Xin He 

#### &#128197; Duration: Oct. 2019 - May. 2020

## Contributions

1. Designed precise wheeled odometry control for accurate robot localization.
2. Implemented A* algorithm for path planning; used PID for motor closed-loop and robot line-following control.
3. Independently completed hardware design and assembly of 3 model AGVs; developed and integrated embedded software for all modules.


## Conclusion

The designed multi-AGV system operates efficiently on the simulated map. Each AGV can plan a path to the target point, follow the planned path to the designated location, and dynamically avoid other AGVs in real-time based on priority during operation. Additionally, each AGV can communicate with the central control system.

## Outcomes
 
1. Two software copyright; 
2. A complete multi-AGV system

## Project Showcase

### Designed AGV:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/微信图片_202409061520531.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/微信图片_20240906152128.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV2</font>
			</center>
		</td>
	</tr>
</table>

### Experimental site:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/多AGV.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
	</tr>
</table>

### Simulation of A* algorithem:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/多AGV.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/多AGV.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
	</tr>
</table>

### Designed PCB:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/多AGV.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20191001/多AGV.jpg" width="90%" />
				<br/>
				<font color="AAAAAA">AGV1</font>
			</center>
		</td>
	</tr>
</table>

### Experimental results:

<video width="640" height="480" controls>
    
    <source src="https://effun.xyz/assets/img/20191001/多AGV.mp4" type="video/mp4">

</video>



