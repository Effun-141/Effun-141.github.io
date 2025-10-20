---
layout: post
title:  "Wheeled-Legged Amphibious Robot Project"
info: "Designed and developed a novel wheeled-legged amphibious robot, focusing on its balance control issues."
tech: "Outcomes: A patent."
type: Brief introduction
---

## Introduction

#### &#127774; Aims: 

1. Design and make the physical prototype of wheeled-legged robot.
2. Design a effective balance control algorithm based for the designed wheeled-legged robot.


#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Feb. 2023 - Apr. 2023

## Contributions

1. Conducted forward and inverse kinematics analysis of the robot using the D-H method, and established kinematic and dynamic models considering joint center of mass constraints.
2. Designed a super-helical sliding mode balance controller based on the robot modeling results, and verified the balance algorithm through simulations in Simulink.
3. Designed and developed the robot's hardware circuitry. Built an air-ground integrated experimental prototype of the robot.


## Conclusion

Concluded that the designed balance controller quickly converges to the desired position or speed, exhibits strong disturbance rejection during air-to-ground transitions, and maintains robustness when the robot's height changes.


## Outcomes
 
1. A patent (A Wheel-Legged Air-Ground Integrated Robot Based On Super-Spiral Sliding Form.)
2. An experimental prototype of the robot

## Project Showcase

### Conceptual diagram of the robot design:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi.png" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi1.png" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi2.png" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>


### Hardware circuitry for the robot's wheel leg section:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi (5).jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi (6).jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### The designed amphibious air-ground robot:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi (2).jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi (3).jpg" width="90%" />
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
				<img src="https://effun.xyz/assets/img/20230207/3.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/4.jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Mathematical model of the robot

The balancing of the robot involves only position and inclination control.

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/kongdi3.png" width="60%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

Using $x$ and $\phi$ represent the position and inclination of the robot, $C_l$ and $C_r$ represent the left and right motor torque, we can get the mathematical model of the robot as following

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/12.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

(More details are coming soon ...)

### Simulink-based simulation model of super-helical sliding mode control algorithm:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/11.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Partial simulation results:

The targst position is 1m and the initial inclination is $\frac{\pi}{4}$:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/位移.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

The targst velocity is 1m/s and the initial inclination is $\frac{\pi}{4}$:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/速度.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

The immunity is tested by equating the instant of air-ground mode switching to a perturbation signal:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20230207/抗扰.png" width="100%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>
