---
layout: post
title:  "Research on Visual SLAM Algorithms for Mobile Robots in Complex Dynamic Environments"
info: "Mitigating the impact of various dynamic objects on SLAM system performance to achieve high-precision localization and static dense point cloud map construction."
tech: "Outcomes: Two papers; A complete novel RGB-D visual SLAM system"
type: Brief introduction 
---

## Introduction

#### &#127774; Aims: 

1. Addressing the impact of highly dynamic objects on SLAM system performance.
2. Solving the problem of unknown dynamic object recognition when semantic segmentation network fails.
3. Overcoming the limitation of SLAM systems in generating dense point cloud maps under dynamic scenes.

#### &#128221; Advisor: Prof. Yifei Wu 

#### &#128197; Duration: Feb. 2024 - present

## Contributions

1. Embedded the ANN semantic segmentation network into ORB-SLAM2 by leveraging ROS framework to identify common dynamic objects.
2. Proposed an unknown dynamic object recognition algorithm combining depth map clustering and multi-view geometry, enabling accurate dynamic object recognition when the semantic segmentation network fails.
3. Designed a strategy to remove dynamic features using semantic information and dynamic depth clusters, improving localization accuracy and map quality.
4. Developed a static point cloud map creating thread to construct high-quality maps in dynamic environments.

## Outcomes
 
1. A complete novel RGB-D visual SLAM system with improved accuracy, robustness, and environmental awareness in highly dynamic scenarios.
2. A journal-type paper. (IEEE Transactions on Instrumentation & Measurement, JCR-Q1, ***Under review***)
3. A conference-type paper. (***Accepted*** at the 22nd IEEE International Conference on Industrial Informatics (IEEE-INDIN 2024))

## Project Showcase

### Robot used for experiments

### Experimental processing and results