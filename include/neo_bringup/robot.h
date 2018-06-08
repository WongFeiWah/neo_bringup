//
// Created by huang on 18-6-7.
//

#ifndef NEO_BRINGUP_ROBOT_H
#define NEO_BRINGUP_ROBOT_H

#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <neo_bringup/CThreadPool.h>
#include <neo_bringup/serial.h>
#include <neo_bringup/sonar.h>






class neo_bringup{
protected:
private:
	ros::NodeHandle nh;
    ros::Publisher odom_publisher;
	ros::Publisher imu_publisher;
	ros::Subscriber sub;
    ros::Publisher sound_wave_publisher0;
    ros::Publisher sound_wave_publisher1;
    ros::Publisher sound_wave_publisher2;
    ros::Publisher sound_wave_publisher3;
	// tf::TransformBroadcaster tfbroadcaster;

    sensor_msgs::Range sonar0;
    sensor_msgs::Range sonar1;
    sensor_msgs::Range sonar2;
    sensor_msgs::Range sonar3;
    sensor_msgs::Range sonar4;
    sensor_msgs::Range sonar5;
    sensor_msgs::Range sonar6;
    sensor_msgs::Range sonar7;


public:
	neo_bringup();

	~neo_bringup();
	void PublishOdom(nav_msgs::Odometry &odom);
	void PublishImu(sensor_msgs::Imu &Imu);
	void callbackCmdvel(const geometry_msgs::Twist& cmdvel_);
    void PublishSound_wave(float *distance_);
};

neo_bringup *GetHandleNode();


#endif //NEO_BRINGUP_ROBOT_H
