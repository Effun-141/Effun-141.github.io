---
layout: post
title:  "Design of a Navigation and Localization System for an Autonomous Harvesting Robot in a Simulated Farmland Environment"
info: "Designed a mobile robot with mapping and autonomous waypoint navigation capabilities."
tech : "Outcomes: A simulated unmanned agricultural robot"
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
				<img src="https://effun.xyz/assets/img/20231001/1 (2).jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20231001/1 (5).jpg" width="90%" />
				<br/>
				<font color="AAAAAA"></font>
			</center>
		</td>
	</tr>
</table>

### Experimental results:

<table rules="none" align="center">
	<tr>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20231001/1 (6).jpg" width="90%" />
				<br/>
				<font color="AAAAAA">Creating 2D map based on Cartograpgher</font>
			</center>
		</td>
		<td>
			<center>
				<img src="https://effun.xyz/assets/img/20231001/1 (4).jpg" width="90%" />
				<br/>
				<font color="AAAAAA">Autonomous localization and navigation based on AMCL and move_base</font>
			</center>
		</td>
	</tr>
</table>

<video width="480" height="272" controls>
    
    <source src="https://effun.xyz/assets/img/20231001/2 (2).mp4" type="video/mp4">

</video>

<video width="480" height="272" controls>

    <source src="https://effun.xyz/assets/img/20231001/2 (3).mp4" type="video/mp4">

</video>


### Partial core code

#### Linear velocity, angular velocity data encapsulation function:
```
void data_pack(const geometry_msgs::Twist& cmd_vel){
	//unsigned char i;
	float_union Vx,Vy,Ang_v;
	Vx.fvalue = cmd_vel.linear.x;
	Vy.fvalue = cmd_vel.linear.y;
	Ang_v.fvalue = cmd_vel.angular.z;
	
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cvalue[0];
	s_buffer[3] = Vx.cvalue[1];
	s_buffer[4] = Vx.cvalue[2];
	s_buffer[5] = Vx.cvalue[3];
	//Vy
	s_buffer[6] = Vy.cvalue[0];
	s_buffer[7] = Vy.cvalue[1];
	s_buffer[8] = Vy.cvalue[2];
	s_buffer[9] = Vy.cvalue[3];
	//Ang_v
	s_buffer[10] = Ang_v.cvalue[0];
	s_buffer[11] = Ang_v.cvalue[1];
	s_buffer[12] = Ang_v.cvalue[2];
	s_buffer[13] = Ang_v.cvalue[3];
	//CRC
	s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^
					s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];
	/*
	for(i=0;i<15;i++){
		ROS_INFO("0x%02x",s_buffer[i]);
	}
	*/
	ser.write(s_buffer,sBUFFERSIZE);
	
}
```
#### Subscribe to the callback function for the topic /cmd_vel for displaying velocity as well as angular velocity :
```
void DataCallback(const geometry_msgs::Twist& vel) {
    ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",vel.linear.x,vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(vel);
};
```

#### Description of the data format in the header file:
```
#define	sBUFFERSIZE	15//send buffer size 串口发送缓存长度
#define	rBUFFERSIZE	27//receive buffer size 串口接收缓存长度

unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

/************************************
 * 串口数据发送格式共15字节
 * head head linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff float       float      float      u8
 * **********************************/
/**********************************************************
 * 串口接收数据格式共27字节
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xff 0xff float      float      float   float   float         float(yaw)   u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union;

```

#### Initializes a ROS node to publish odometry data and broadcast transforms in main.c:
```
  int main(int argc, char** argv){

  ros::init(argc, argv, "odom_pub");

  ros::NodeHandle nh;
  ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 50, DataCallback); //订阅/cmd_vel数据
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 30); //发布/odom话题
  
  //定义tf对象
  tf::TransformBroadcaster odom_broadcaster;

  float_union posx, posy, vx, vy, va, yaw;
  //定义tf发布时需要的类型消息
  geometry_msgs::TransformStamped odom_trans;

  //定义里程计消息对象
  nav_msgs::Odometry odom;

  //定义四元数变量
  geometry_msgs::Quaternion odom_quat;


  //开启串口
  try
    {
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

  //定义时间
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(10); //10hz执行
  while(ros::ok())
  {

    ros::spinOnce();
     current_time = ros::Time::now();
    if(ser.available())
    {
        ROS_INFO_STREAM("Reading from serial port");
			  ser.read(r_buffer,rBUFFERSIZE);

        if(data_analysis(r_buffer) != 0)
        {
          int i;
		  ROS_INFO_STREAM("odom received");
				  for(i=0;i<4;i++){
					posx.cvalue[i] = r_buffer[2+i];//x 坐标
					posy.cvalue[i] = r_buffer[6+i];//y 坐标
					vx.cvalue[i] = r_buffer[10+i];// x方向速度
					vy.cvalue[i] = r_buffer[14+i];//y方向速度
					va.cvalue[i] = r_buffer[18+i];//角速度
					yaw.cvalue[i] = r_buffer[22+i];	//yaw 偏航角
					
				  }	
				  ROS_INFO("[%f], [%f], [%f], [%f], [%f], [%f]",posx.fvalue, posy.fvalue, vx.fvalue, vy.fvalue, va.fvalue, yaw.fvalue);		
				  //将偏航角转换成四元数才能发布
				  odom_quat = tf::createQuaternionMsgFromYaw(yaw.fvalue);

				odom_trans.header.stamp = current_time;
				//发布坐标变换父子坐标系
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_link";
				//填充获取的数据
				odom_trans.transform.translation.x = posx.fvalue;//x坐标
				odom_trans.transform.translation.y = posy.fvalue;//y坐标
				odom_trans.transform.translation.z = 0;//z坐标				
				odom_trans.transform.rotation = odom_quat;//偏航角
				//发布tf坐标变换
				odom_broadcaster.sendTransform(odom_trans);
				//获取当前时间
				//current_time = ros::Time::now();
				//载入里程计时间戳
				
				odom.header.stamp = current_time;
				//里程计父子坐标系
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_link";
				//里程计位置数据
				odom.pose.pose.position.x = posx.fvalue;
				odom.pose.pose.position.y = posy.fvalue;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = odom_quat;
				//载入线速度和角速度
				odom.twist.twist.linear.x = vx.fvalue;
				odom.twist.twist.linear.y = vy.fvalue;
				odom.twist.twist.angular.z = va.fvalue;
				//发布里程计消息
				odom_pub.publish(odom);
				ROS_INFO("publish odometry");
				last_time = current_time;
        }
        memset(r_buffer,0,rBUFFERSIZE);
    }              
    r.sleep();

  }
  return 0;
}
```

