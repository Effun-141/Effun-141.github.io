---
layout: post
title:  "Design of an Autonomous Vehicle System Based on ROS for Multi-Vehicle Interaction Scenarios"
info: "Simulate the process of an autonomous vehicle driving on the road."
tech: "Outcomes: A complete and versatile development platform for unmanned vehicles."
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

1. The main goal of this project is to design a versatile unmanned vehicle experimental platform that can be used for the development and validation of algorithms such as SLAM, path planning, and object detection.
2. We used the designed unmanned vehicle platform to participate in the *13th Jiangsu Provincial Robotics Competition's multi-drone interaction project*. The competition simulates autonomous vehicles driving on roads, involving tasks such as straight-line driving, overtaking, lane changing, and parking.
3. It was also the final project for a course called 'Comprehensive Embedded Systems Experiment' during my master's studies. 
   

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Oct. 2023 - Nov. 2023

## Contributions

1. Proposed a dual-loop localization and obstacle avoidance algorithm based on LiDAR data for angle and position control, enabling the autonomous vehicle to drive in the center of a simulated road and interact with multiple vehicles.
2. Designed a communication program between the NVIDIA Jetson Nano and the lower-level control board based on ROS; integrated all algorithms into ROS packages to improve development efficiency.
3. Independently completed the full assembly of the mobile robot.


## Outcomes
 
A complete and versatile development development platform for unmanned vehicle.

A course report that received a score of 93, ranking 1/114 in the class.

## Project Showcase

### Designed unmanned vehicle development platform:

It comprises of a NVIDIA Jetson Nano, a 2D Lidar, a gimbal camera, a Wifi module, a remoting module and a power module.

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/2.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
        		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/1.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
        </td>
	</tr>
</table>

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/微信图片_20240907205620.jpg" width="80%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>
        		
<video width="480" height="272" controls>
    
    <source src="https://effun.xyz/assets/img/20220928/ackerman-2.mp4" type="video/mp4">

</video>

The designed platform can also used to deploy SLAM algorithm:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/6.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
        		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/5.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Dual-loop localization algorithm based on LiDAR data:

####  Lidar centre position solution:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/Quicker_20240907_211427.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

By using the distance information at 90° and 270°, the distance $x$ from the LiDAR center to the right-side barrier can be calculated.

<br>

\[
\frac{x}{90-x} = \frac{range\_30}{range\_9}
\]


<br>

$$
x = \frac{90 * range\_30}{range\_9 + range\_30}
$$

 
$$
x = \frac{90 * range\_30}{range\_9 + range\_30}
$$
 
$$
\frac{x}{90-x} = \frac{a}{b}
$$




$$
^B_T T =  
\left [
    \begin{array}{}
        I & p_{TB} \\\\
        0   & 1   \\\\
    \end{array}
\right ] = 
\left [
    \begin{array}{}
        1 & 0 & 0 & -l \\\\
        0 & 1 & 0 & H_1 + H_2 - h_1 \\\\
        0 & 0 & 1 & -d \\\\
        0 & 0 & 0 & 1   \\\\     
    \end{array}
\right ]
$$


For heading control, it is necessary to determine the front-facing direction and the change in orientation of the mobile robot. Since the Ackermann steering mechanism was used in this design, the servo's steering angle can not directly represent the vehicle's heading. To address this issue, the design used a trigonometric method to determine the heading.

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/Quicker_20240907_211522.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

$\alpha$ represents the offset angle of the car head. In heading control, the goal is to maintain the angle $\alpha$ at 0°, meaning the vehicle's front is pointing forward.

The sine of the angle $\alpha$ can be obtained from the following equation:

$$
sin \alpha = \sqrt{ 1 - (\frac{x}{range\_30})^2}
$$

The block diagram of the heading-position double closed-loop control is shown below:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/Quicker_20240907_211552.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### System test:

Test the Lidar data:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/3.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

Test the proposed double-loop localization algorithm:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20220928/4.jpg" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Final effect:

<video width="310" height="640" controls>
    
    <source src="https://effun.xyz/assets/img/20220928/ackerman-1.mp4" type="video/mp4">

</video>

