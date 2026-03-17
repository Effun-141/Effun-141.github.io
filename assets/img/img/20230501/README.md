# 课程论文《基于LeGO-LOAM的激光SLAM系统设计》程序运行说明

*本文档对如何运行论文代码进行说明，欢迎老师访问我的个人网站查看更加详细的[实验记录](https://effun.xyz/2023/05/experiment-recording-Lego-LOAM-+-KITTI/)*


## 1.软件环境

&emsp;&emsp;本文软件运行环境为Ubuntu18.04

&emsp;&emsp;ROS版本：melodic

&emsp;&emsp;所需要安装的依赖库及其对应版本：

|库名称|版本|
|---|---|
|OpenCV|3.4.3|
|Gtsam|4.0.0-alpha2|
|Python |2.7.17|
|ceres |2.0.0|
|PCL |1.8.1|
|Eigen |3.3.4|
|CMake |3.10.2|

## 2.运行说明

### （1）程序编译

&emsp;&emsp;legoloam_ws是一个工作空间，首次运行需对其进行编译，使用的命令如下：
```
cd legoloam_ws
catkin_make -j1
```

* 说明：只有首次编译需加-j1，后续重新编译只需要`catkin_make`即可。

### （2）使用KITTI数据集07数据段对系统进行测试

&emsp;&emsp;由于制作好的rosbag有接近6G，因此需要通过网盘下载，下面是网盘链接：
```
链接：https://pan.baidu.com/s/1bV0UL7SDPYPMveUxfzyRRw?pwd=uqrh 
提取码：uqrh
```

&emsp;&emsp;修改transformFusion.cpp文件中的位姿文件保存路径：
```
const string RESULT_PATH = "[自己的路径]/07res.txt";
```

&emsp;&emsp;打开两个终端，其中一个输入：
```
roslaunch lego_loam run.launch
```
&emsp;&emsp;在另一个终端中调用命令播放rosbag之前要先cd到bag文件所在文件夹，即：
```
cd ~[bag文件路径]
rosbag play kitti_2011_09_30_drive_0027_synced.bag --clock --topic /kitti/velo/pointcloud
```

### (3) evo轨迹评估

&emsp;&emsp;基于文件夹中的07.txt真实值文件和程序所生成的07res.txt文件，使用evo工具进行对比。相关命令如下：

&emsp;&emsp;轨迹绘制
```
evo_traj kitti 07res.txt --ref=07.txt -p --plot_mode=xz
```

&emsp;&emsp;相对位姿误差rpe
```
evo_rpe kitti 07.txt 07res.txt -r full -va --plot --plot_mode xyz
```
