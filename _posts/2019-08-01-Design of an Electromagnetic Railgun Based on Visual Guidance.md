---
layout: post
title:  "Design of an Electromagnetic Railgun Based on Visual Guidance"
info: "This is the control category problem of the 2019 National Undergraduate Electronic Design Competition."
tech: "Outcomes: First Prize of National Student Electronic Design Competition (Jiangsu Division) "
type: Brief introduction
---

## Introduction


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

#### &#127774; Aims: 

1. Electromagnetic guns have both manual and automatic operating modes
2. In manual mode, the projectile is fired based on the input of the target center position coordinates. 
3. In automatic mode, the target center position coordinates are determined by recognizing the red marker using a visual sensor, and the projectile is fired automatically.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Aug. 2019 - Oct. 2019

## Contributions

1. Target Recognition Algorithm Implementation. Based on OpenMV sensor, a red target recognition program, distance measurement program, and communication program between the microcontroller and OpenMV were written in Python.
2. Using MATLAB and Excel, the relationship between the projectile launch distance and the charging voltage was fitted to obtain a ballistic table relationship curve, ensuring that the projectile hits the target center.


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



#### Accuracy display

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20190801/curved-fire-gun-1.gif" width="100%" />
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
