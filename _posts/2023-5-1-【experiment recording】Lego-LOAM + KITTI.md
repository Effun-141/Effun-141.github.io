---
layout: post
title: 【experiment recording】基于KITTI数据集测试Lego-LOAM并使用evo工具对生成的轨迹进行评估
date: 2023-5-1
tags: Lego-LOAM
---

*本文记录如何运行开源激光SLAM框架[Lego-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)。基于KITTI数据集测试算法效果，并生成轨迹以使用evo工具进行评估。*

** **

**实验环境：Ubuntu18.04 + ROS-melodic**

# I 编译Lego-LOAM

# II 使用kitti2bag工具将KITTI数据段转化为bag文件

# III 相关更改

## 1.更改Lego-LOAM的点云topic
&emsp;&emsp;用`rosbag info` 命令查看KITTI数据集bag包的点云topic为`/kitti/velo/pointcloud`，而Lego-LOAM的点云topic为`/velodyne_points`。因此要运行该bag需要将Lego-LOAM的点云topic作如下更改：
```
extern const string pointCloudTopic = "/kitti/velo/pointcloud";
```

## 2.更改utility.h参数
```
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2083;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 26.8/float(N_SCAN-1);
extern const float ang_bottom = 24.8;
extern const int groundScanInd = 55;
```
## 3.更改useCloudRing模式为false

&emsp;&emsp;出现以下报错：
```
Point cloud is not in dense format, please remove NaN points first!
```
&emsp;&emsp;解决方法为更改useCloudRing模式为false：
```
extern const bool useCloudRing = false;
```

## 4.启用回环检测
```
extern const bool loopClosureEnableFlag = true;
```

# IV 运行命令

&emsp;&emsp;运行Lego_LOAM
```
roslaunch lego_loam run.launch
```

&emsp;&emsp;进入KITTI数据段所在的文件夹
```
cd /home/effun/KITTI_Dataset/2011_09_26_drive_0035_sync
```

```
rosbag play kitti_2011_09_26_drive_0035_synced.bag --clock --topic /kitti/velo/pointcloud
```

# V evo轨迹评估

&emsp;&emsp;轨迹绘制
```
evo_traj kitti 00res.txt --ref=00.txt -p --plot_mode=xz
```

&emsp;&emsp;绝对位姿误差ape
```
evo_ape kitti 00.txt 00res.txt -r full -va --plot --plot_mode xyz --save_plot /home/effun --save_results /home/effun
```

&emsp;&emsp;相对位姿误差rpe
```
evo_rpe kitti 00.txt 00res.txt -r full -va --plot --plot_mode xyz
```

# VI 报错记录

&emsp;&emsp;运行Lego-LOAM并播放转化好的KITTI数据集bag时出现一个warning：
```
Detected jump back in time of 4.01076s. Clearing TF buffer
```
&emsp;&emsp;关掉系统时间通过网络同步即可解决。





