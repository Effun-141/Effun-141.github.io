---
layout: post
title:  "Event Camera DAVIS346 使用记录"
info: "记录DAVIS346事件相机的使用方法以及一些对驱动程序的修改."
tech: " "
type: Brief introduction 
---

**写在前面的：感觉事件相机虽然已经兴起有一段时间了，但是还处在发展阶段，相关的资料其实并不完善，使用过程中会遇到大大小小各种问题，因此欢迎使用事件相机（特别是DAVIS346）的朋友一起交流学习，可以通过我[主页](effun.xyz)的联系方式与我联系！**

**要记录的东西有好多，来不及排版了，先想到什么记下什么吧~~~**


使用的相机型号为DAVIS346 stereo kit，由两个DAVIS346 mono组成，可输出事件流以及灰度图像，目前已踩了好多的坑，下面仅对双目davis ROS模式下的使用(**davis_ros_driver**)进行记录。这里是[ros driver的github仓库](https://github.com/uzh-rpg/rpg_dvs_ros)。

## 已解决问题

#### &#127774; 事件发布频率无法调整问题: 

通过rqt的参数调整界面无法设置streaming rate，上限始终为30hz，远远小于期望的1000hz。

在github仓库中有对应的[issue #94](https://github.com/uzh-rpg/rpg_dvs_ros/issues/94)以及解决办法。

解决方法：修改davis_ros_driver下的`driver.cpp`：在**633行下面添加如下代码：**

```
 caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, 1000);
```

该处代码块变成：

```
caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
//添加的一行：
caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, 1000);
caerDeviceDataStart(davis_handle_, NULL, NULL, NULL, &DavisRosDriver::onDisconnectUSB, this);
```

这里的`1000`对应最大事件频率1000hz，如果需要更高的事件频率，就要减小这个数值，如`800`对应`1250hz`。

#### &#127774; 灰度图像时不时闪烁且分辨率错误问题：

测试过程中发现，相机左右目的灰度图像会是不是发生“闪烁”，并且频率不确定，每次闪烁的结果就是图像的分辨率会改变，比如正常是346\*260，闪烁后变成259\*260,(这是我用一个ros脚本将图像从bag中提取出来发现的。。。)

**这个问题实在是太难绷了，我试图找到原因和规律，但是这个现象似乎并没有什么特别的触发规律**

对于这个问题暂时我还没有很好的解决，只能通过一个笨方法去除所有错误分辨率的图像。

在`driver.cpp`中添加对图像尺寸的验证，如果不符合标准的346\*260就不发布图像，在795行后添加，代码块变成：

```
msg.width = frame_width;
msg.height = frame_height;
msg.step = frame_width * frame_channels;

// 下面的if判断是要添加的代码，添加尺寸验证
if (msg.width != 346 || msg.height != 260)
{
	ROS_WARN("Incorrect image dimensions: %dx%d", msg.width, msg.height);
	continue;  // 跳过错误帧
}
```

#### &#127774; IMU数据问题：

在`rqt_reconfig`中可以调节IMU相关参数：

**1.imu_acc_scale（加速度计量程）：**

定义加速度传感器的测量范围
常见值：±2g, ±4g, ±8g, ±16g
较小的量程提供更高的精度，但测量范围小
较大的量程可以测量更大的加速度，但精度较低
例：如果设置为±4g，意味着可以测量-4g到+4g范围内的加速度

默认16，我修改为4

**2.imu_gyro_scale（陀螺仪量程）：**

定义陀螺仪的角速度测量范围
常见值：±250°/s, ±500°/s, ±1000°/s, ±2000°/s
较小的量程提供更精确的角速度测量
较大的量程可以测量更快的旋转运动
例：设置为±500°/s意味着可以测量每秒-500°到+500°的旋转速度

默认2000我修改为1000

**3.imu_low_pass_filter（低通滤波器）：**

用于过滤高频噪声
较低的截止频率可以更好地抑制噪声，但会增加延迟
较高的截止频率响应更快，但可能包含更多噪声
通常需要在响应速度和平滑度之间找到平衡
常见范围：5Hz-260Hz

默认260我修改为184

修改后的`config`文件夹中的`DAVIS346.yaml`配置文件如下：

```
{ADC_RefHigh_volt: 29, ADC_RefHigh_curr: 7, ADC_RefLow_volt: 1, ADC_RefLow_curr: 7, DiffBn_coarse: 4, DiffBn_fine: 39, OFFBn_coarse: 4, OFFBn_fine: 0, ONBn_coarse: 6,
  ONBn_fine: 200, PrBp_coarse: 2, PrBp_fine: 58, PrSFBp_coarse: 1, PrSFBp_fine: 33,
  RefrBp_coarse: 4, RefrBp_fine: 25, aps_enabled: true, dvs_enabled: true, exposure: 4000, autoexposure_enabled: true,
  frame_delay: 0, imu_acc_scale: 1, imu_enabled: true, imu_gyro_scale: 2, imu_low_pass_filter: 2, max_events: 0,
  streaming_rate: 999}

```

#### &#127774; 找不到相机配置文件问题：

又是一个难绷的问题：

```
Camera calibration file /home/njust/.ros/camera_info/DAVIS-00001099.yaml not found

Camera calibration file /home/njust/.ros/camera_info/DAVIS-00001091.yaml not found
```

左右目的相机配置文件找不到，这样的话在roslaunch的时候就没有`camerainfo`这个topic的信息

其实这个影响大不大我暂时还不太确定，但是有的人在遇到这个问题的时候相机是无法工作的。

解决的办法已经在这个[issue](https://giters.com/uzh-rpg/rpg_dvs_ros/issues/117)中给出了，似乎就是`boost-inivation`和ROS build系统发生了冲突，官方说法：

*A quick fix is to uninstall the 'boost-inivation' package, then clean and rebuild the ROS packages, then they should work again.*

而有[CSDN博客](https://blog.csdn.net/gwplovekimi/article/details/120458248?spm=1001.2014.3001.5502)也提到了同样的问题。解决方法相同，都是移除`boost-inivation`：

```
sudo apt-get remove boost-inivation
```

然后应该是要重新编译ros工程的，但是我这样做了好像没有什么用，大家可以试一试，幸运的是我在另一台电脑上面的编译过程中并没有报这个问题，在`./ros/`目录下有camera_info这一文件夹，里面有相关yaml文件，我就直接复制过来了，反正都是同一台相机。


