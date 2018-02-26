/*
---------+---+----+----------------------------------+--------+---+---+----+
|  Header 				|Type	|   Len  |      Payload 12 bytes        		 	|Checksum	|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55			|0xAA	|0x03|    short    | 2 Int 2 short            	| 0xZZ   		|ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+

*/


#include <math.h>
#include <stdio.h>
#include <sys/time.h>    //struct itimerval, setitimer()
#include <signal.h>


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <serial/serial.h>
#include <pthread.h>
#include <micvsion_bringup/serial.h>


// Create serial port
myserial serial;
//serial::Serial ser; 


typedef struct {
	float value1;
	float value2;
	float value3;
}send_data;

typedef struct {
	int value1;
	int value2;
	int value3;

	float speed1;
	float speed2;
	float speed3;
}odom_data;


typedef struct{
  short acc[3];
  short gyro[3];

  float w;
  float x;
  float y;
  float z;
}IMU_MSG;

typedef struct {
    unsigned char head;
    unsigned char len;
    unsigned char mode;
}ST_PACKAGE_HEAD;

typedef struct {
    unsigned char end;
}ST_PACKAGE_END;

typedef struct {
    unsigned char mode;
    unsigned char len;
    unsigned char data[256];
}ST_PACKAGE_DATA;

int old_encoder[3]={0,0,0};
bool old_encoder_init = false;
double odometry_x=0;
double odometry_y=0;
double odometry_th=0;
nav_msgs::Odometry odom;
#define VEL_TRANS 329.4531
//circle/min trans to m/s,formula: 1circle/min = 1*(2*pi*0.029)/60 m/s.
#define INVERSE_VEL_TRANS 0.00304
//#define L 0.11
#define PI 3.1415926
double L=0;



geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;

sensor_msgs::Imu imu_msg;
//tf::TransformBroadcaster tfbroadcaster;

static int resetCount = 0;
static bool sendOK = true;

double q[4] = {0};


void odom_handle(void* read_data)
{
	static double last_th = 0.0;
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	odom_data recvData = {0};
	memcpy(&recvData,read_data,24);
	/*****************************************************************************

			Get Encoder 
	
	*****************************************************************************/
	int new_encoder[3];
	if (!old_encoder_init) {
		old_encoder[0] = recvData.value1;
		old_encoder[1] = recvData.value2;
		old_encoder[2] = recvData.value3;
		old_encoder_init = true;
	}

	int diff_encoder[3];
#if 0
		printf("------\r\n");
	
		printf("odom: %f\r\n");
		printf("count: \n%d  \n%d  \n%d\r\n",recvData.value1,recvData.value2,recvData.value3);
		printf("speed: \n%f  \n%f  \n%f\r\n",recvData.speed1,recvData.speed2,recvData.speed3);
#endif

	
	
	new_encoder[0] = recvData.value1;
	new_encoder[1] = recvData.value2;
	new_encoder[2] = recvData.value3;

	diff_encoder[0] = new_encoder[0] - old_encoder[0];
	diff_encoder[1] = new_encoder[1] - old_encoder[1];
	diff_encoder[2] = new_encoder[2] - old_encoder[2];

	old_encoder[0] = new_encoder[0];
	old_encoder[1] = new_encoder[1];
	old_encoder[2] = new_encoder[2];
	/*****************************************************************************

			calc Odometry
	
	*****************************************************************************/
	float wheel_dis[3];
	float x,y,th;
	wheel_dis[0] = diff_encoder[2]*PI*58*0.001/330;
	wheel_dis[1] = diff_encoder[1]*PI*58*0.001/330;
	wheel_dis[2] = diff_encoder[0]*PI*58*0.001/330;
	x = (wheel_dis[0]-wheel_dis[1])/1.732;
	y = (2*wheel_dis[2]-wheel_dis[0]-wheel_dis[1])/3.0;
	//th = -(wheel_dis[0]+wheel_dis[1]+wheel_dis[2])/(3*L);
	th = atan2f(q[1]*q[2] + q[0]*q[3],
                     0.5f - q[2]*q[2] - q[3]*q[3]) - last_th;
	last_th = atan2f(q[1]*q[2] + q[0]*q[3],
                     0.5f - q[2]*q[2] - q[3]*q[3]);

	odometry_th += th;
	odometry_x += (x * cos(odometry_th) - y * sin(odometry_th));
	odometry_y += (y * cos(odometry_th) + x * sin(odometry_th));

	if(odometry_th > 2*PI)
		odometry_th -= 2*PI;
	if(odometry_th < -2*PI)
		odometry_th += 2*PI;
	if(odometry_x > 100.0 || odometry_x < -100.0)
	{
		ROS_INFO("odometry_x:%d  %d  %d ",new_encoder[0],new_encoder[1],new_encoder[2]);		
	}

	geometry_msgs::Quaternion odom_quat;
	//printf("theta : %f\n",odometry_th);
	odom_quat = tf::createQuaternionMsgFromYaw(odometry_th);
	// position
	odom.pose.pose.position.x = odometry_x;
	odom.pose.pose.position.y = odometry_y;
	odom.pose.pose.position.z = th;
	odom.pose.pose.orientation = odom_quat;

	odom.pose.covariance[0] = 0.1;
	odom.pose.covariance[7] = 0.1;
	odom.pose.covariance[14] = 999999;
	odom.pose.covariance[21] = 999999;
	odom.pose.covariance[28] = 999999;
	odom.pose.covariance[35] = 0.05;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	double v_motor[3];
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	v_motor[0] = recvData.speed1*INVERSE_VEL_TRANS;
	v_motor[1] = recvData.speed2*INVERSE_VEL_TRANS;
	v_motor[2] = recvData.speed3*INVERSE_VEL_TRANS;
	//double dt = (current_time - last_time).toSec();
	vx = (v_motor[0]-v_motor[1])/1.732;
	vy = (2*v_motor[2]-v_motor[0]-v_motor[1])/3.0;
	vth = -(v_motor[0]+v_motor[1]+v_motor[2])/(3*L);

	//velocity
	odom.twist.twist.linear.x = vx; 
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.linear.z = recvData.speed1;
	odom.twist.twist.angular.x = recvData.speed2;
	odom.twist.twist.angular.y = recvData.speed3;
	odom.twist.twist.angular.z = vth;

	odom_pub.publish(odom);
//	ROS_INFO("odom");
	static tf::TransformBroadcaster tfbroadcaster;
    //tfbroadcaster.init(nh);
	odom.header.frame_id = "odom";
	odom_tf.header = odom.header;
	odom_tf.child_frame_id = "base_footprint";
	odom_tf.transform.translation.x = odom.pose.pose.position.x;
	odom_tf.transform.translation.y = odom.pose.pose.position.y;
	odom_tf.transform.translation.z = odom.pose.pose.position.z;
	odom_tf.transform.rotation = odom.pose.pose.orientation;

	tfbroadcaster.sendTransform(odom_tf);
}

void sensor_handle(void*  read_data)
{
}

void imu_handle(void*  read_data)
{
	double th = 0.0;
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
	IMU_MSG imu_info = {0};
	memcpy(&imu_info, read_data, sizeof(IMU_MSG));
#if 1
	printf("------\r\n");
	th = atan2f(q[1]*q[2] + q[0]*q[3],
											 0.5f - q[2]*q[2] - q[3]*q[3])*180/3.141592653;

	printf("imu: %f\r\n",th);
	printf("acc: \n%f  \n%f  \n%f\r\n",(double)imu_info.acc[0],(double)imu_info.acc[1],(double)imu_info.acc[2]);
	printf("gyro: \n%f  \n%f  \n%f\r\n",(double)imu_info.gyro[0],(double)imu_info.gyro[1],(double)imu_info.gyro[2]);
	printf("q: \n%f  \n%f  \n%f \n%f\r\n",(double)imu_info.w,(double)imu_info.x,(double)imu_info.y,(double)imu_info.z);
#endif
	if(imu_info.gyro[0] == 0.0 
		&& imu_info.gyro[1] == 0.0 
		&& imu_info.gyro[2] == 0.0 
		&& imu_info.acc[0] == 0.0)
	{
		return;
	}

	q[0] = imu_info.w;
	q[1] = imu_info.x;
	q[2] = imu_info.y;
	q[3] = imu_info.z;

    imu_msg.header.stamp    = ros::Time::now();
	imu_msg.header.frame_id = "imu_link";

	imu_msg.angular_velocity.x = imu_info.gyro[0];
	imu_msg.angular_velocity.y = imu_info.gyro[1];
	imu_msg.angular_velocity.z = imu_info.gyro[2];
	imu_msg.angular_velocity_covariance[0] = 0.02;
	imu_msg.angular_velocity_covariance[1] = th;
	imu_msg.angular_velocity_covariance[2] = 0;
	imu_msg.angular_velocity_covariance[3] = 0;
	imu_msg.angular_velocity_covariance[4] = 0.02;
	imu_msg.angular_velocity_covariance[5] = 0;
	imu_msg.angular_velocity_covariance[6] = 0;
	imu_msg.angular_velocity_covariance[7] = 0;
	imu_msg.angular_velocity_covariance[8] = 0.02;

	imu_msg.linear_acceleration.x = imu_info.acc[0];///1638.4;
	imu_msg.linear_acceleration.y = imu_info.acc[1];///1638.4;
	imu_msg.linear_acceleration.z = imu_info.acc[2];///1638.4;
	imu_msg.linear_acceleration_covariance[0] = 0.04;
	imu_msg.linear_acceleration_covariance[1] = 0;
	imu_msg.linear_acceleration_covariance[2] = 0;
	imu_msg.linear_acceleration_covariance[3] = 0;
	imu_msg.linear_acceleration_covariance[4] = 0.04;
	imu_msg.linear_acceleration_covariance[5] = 0;
	imu_msg.linear_acceleration_covariance[6] = 0;
	imu_msg.linear_acceleration_covariance[7] = 0;
	imu_msg.linear_acceleration_covariance[8] = 0.04;

	imu_msg.orientation.w = imu_info.w;
	imu_msg.orientation.x = imu_info.x;
	imu_msg.orientation.y = imu_info.y;
	imu_msg.orientation.z = imu_info.z;

	imu_msg.orientation_covariance[0] = 0.0025;
	imu_msg.orientation_covariance[1] = 0;
	imu_msg.orientation_covariance[2] = 0;
	imu_msg.orientation_covariance[3] = 0;
	imu_msg.orientation_covariance[4] = 0.0025;
	imu_msg.orientation_covariance[5] = 0;
	imu_msg.orientation_covariance[6] = 0;
	imu_msg.orientation_covariance[7] = 0;
	imu_msg.orientation_covariance[8] = 0.0025;

	imu_pub.publish(imu_msg);
	

	static tf::TransformBroadcaster tfbroadcaster;
	//tfbroadcaster.init(nh);
	tfs_msg.header.stamp    = ros::Time::now();
	tfs_msg.header.frame_id = "base_link";
	tfs_msg.child_frame_id  = "imu_link";
	tfs_msg.transform.rotation.w = imu_info.w;
	tfs_msg.transform.rotation.x = imu_info.x;
	tfs_msg.transform.rotation.y = imu_info.y;
	tfs_msg.transform.rotation.z = imu_info.z;

	tfs_msg.transform.translation.x = 0.0;
	tfs_msg.transform.translation.y = 0.0;
	tfs_msg.transform.translation.z = 0.15;

	tfbroadcaster.sendTransform(tfs_msg);
	
}




bool checkSum(unsigned char* buf)
{
	unsigned char sum = 0;
	unsigned short len = 0;
	len = buf[3]*256 + buf[4];
	
	for(int i = 5; i< len+5; i++)
	{
		sum+=buf[i];
	}
	
	if(sum != buf[len+5])
	{
		return false;
	}
	//ROS_INFO("%02X",buf[2]);
	return true;
}

void LOG_Handle(void* buf,unsigned short len)
{
	//char buffer[516] = {0};
	
	//printf((char*)buf);
}
static int tmp = 0;
void* package_thread(void* buf)
{
    pthread_detach(pthread_self());
    ST_PACKAGE_DATA readData;
    memcpy(&readData,buf,readData.len+2);
	unsigned short len = 0;
	//ROS_INFO("PACKAGE  %2X",buf[2]);
	switch(readData.mode)
	{
		case 0:
			break;
		case 1:
			odom_handle(readData.data);
			break;
		case 2:
			break;
		case 3:
			sensor_handle(readData.data);
			break;
		case 'I':
			printf("len = %d, %c\n",readData.len,readData.mode);
			imu_handle(readData.data);
			break;
		case 0x97:
			//ROS_INFO("LOG");
			LOG_Handle(readData.data,readData.len);
			break;
		case 0x98:
			ROS_INFO("OK	 %d",tmp);
			tmp++;
			sendOK = true;
			break;
		case 'T':
			//if(readData.data[0] != 0xDD)return NULL;

            printf("TEST===================RECV \n");

			resetCount++;
			break;
		default:
			break;
	}

	return NULL;
}


static void callback(const geometry_msgs::Twist& cmdvel_)
{
	static double pre_secs = 0.0;
	double secs =ros::Time::now().toSec();
	send_data sdata = {0};
	unsigned char a = 0;
	unsigned char sendBuffer[25] = {0};
	float wheel_left,wheel_back,wheel_right;
	float linear_x = cmdvel_.linear.x;
	float linear_y = cmdvel_.linear.y;
	float angular_z = cmdvel_.angular.z;
	sdata.value1 = wheel_left = ( linear_x * 0.8660254 - linear_y * 0.5 - angular_z * L ) * VEL_TRANS/2.5;
	sdata.value2 = wheel_back =( linear_y - angular_z *  L) * VEL_TRANS/2.5;
	sdata.value3 = wheel_right=(-linear_x * 0.8660254 -linear_y * 0.5 -angular_z*L)*VEL_TRANS/2.5;

	
	sdata.value1 = -wheel_back;
	sdata.value2 = -wheel_left ;
	sdata.value3 = -wheel_right;

//	sdata.value1 = 0.0;
//	sdata.value2 = 0.0;
//	sdata.value3 = 0.0;
   

	 
	sendBuffer[0] = 0x55;
	sendBuffer[1] = 0xAA;
	sendBuffer[2] = 0x81;
	sendBuffer[3] = 0;
    sendBuffer[4] = 0x0C;
	memcpy(&sendBuffer[5], &sdata, 12);
	for (int i = 5; i < (12+5); i++) {
	  a += sendBuffer[i];
	}
	sendBuffer[12+5] = a;
	a = 0;
	sendBuffer[12+6] = 0;
	sendBuffer[12+7] = 0;
	sendBuffer[12+8] = 0x0A;
	if(pre_secs == 0.0)
	{
		pre_secs = ros::Time::now().toSec();
	}

	if(sendOK == true || abs(pre_secs-secs) > 0.2)
	{
		serial.Write(sendBuffer, 21);
		//ROS_INFO("CMD:%f  %f  %f ",sdata.value1,sdata.value2,sdata.value3);		
		pre_secs = secs;
		sendOK = false;
	}

}

void timercallback(const ros::TimerEvent& event)
{
	static int count = 0;
	printf("timer %d   reset:%d \n",count,resetCount);


	send_data sdata = {0};
	unsigned char a = 0;
	unsigned char sendBuffer[25] = {0};

	sdata.value1 = 0.0;
	sdata.value2 = 0.0;
	sdata.value3 = 0.0;



	sendBuffer[0] = 0x55;
	sendBuffer[1] = 0xAA;
	sendBuffer[2] = 0x81;
	sendBuffer[3] = 0;
	sendBuffer[4] = 0x0C;
	memcpy(&sendBuffer[5], &sdata, 12);
	for (int i = 5; i < (12+5); i++) {
	a += sendBuffer[i];
	}
	sendBuffer[12+5] = a;
	a = 0;
	sendBuffer[12+6] = 0;
	sendBuffer[12+7] = 0;
	sendBuffer[12+8] = 0x0A;
	serial.Write(sendBuffer, 21);
	count++;

    return ;
}

void* robot_serial_thread(void*)
{
    pthread_detach(pthread_self());
    ST_PACKAGE_HEAD head = {0};
    int length = 0;
    int ret = 0;
    unsigned char ch;
    ST_PACKAGE_DATA readData = {0};
    printf("serial thread.\n");
    //unsigned char readData[256] = {0};
    while(!serial.stop_thread)
    {
        // read head handle
        ret = serial.Read(&ch,1,10);
        if(ret == 1){
            if(head.head == 0){
                if(ch != 0xAA)continue;
                head.head = ch;
            }else if(head.len == 0){
                if(ch == 0){
                    memset(&head,0,sizeof(ST_PACKAGE_HEAD));
                    continue;
                }
                head.len = ch;
            }else if(head.mode == 0){
                head.mode = ch;
                // read data handle
                if( serial.Read(readData.data,head.len,10) == head.len ){
                    readData.mode = head.mode;
					readData.len = head.len;
                    /*printf("%02X ",head.head);
					printf("%02X ",head.len);
					printf("%02X ",head.mode);
					printf("%02X \n",readData.data[0]);*/

                    pthread_t id;
                    bool ret;
                    ret = pthread_create(&id,NULL,package_thread,(void*)&readData);
                    if(!ret)
                    {
                        //printf("succeed!\n");

                    }
                    else
                    {
                        printf("Fail to Create Thread.\n");
                    }
                }
                memset(&head,0,sizeof(ST_PACKAGE_HEAD));
            }
        }
    }
    printf("serial end.\n");
}


int main(int argc, char** argv)
{
	unsigned char buffer[512] = {0};
	int i, length = 0;
	int readflag = 0;
	int match_flag = 0;
	char ch = 0;
	ros::init(argc,argv,"neo_bringup_node");
	ros::NodeHandle nh;
	ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 100);
	ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 100, callback);


	nh.param<double>("odom_pub/robot_radius", L, 0.105);

	// Open serial port ("COM3", "/dev/ttyUSB0")
	serial.Open((char*)"/dev/ttyACM0", 115200, 8, NO, 1);
	ROS_INFO("/dev/ttyUSB0  start");


	//ros::Timer timer = nh.createTimer(ros::Duration(0.02), timercallback);
    ros::Rate loop_rate(100);

    pthread_t id;
    bool ret;
    ret = pthread_create(&id,NULL,robot_serial_thread,NULL);
    if(!ret)
    {
        printf("succeed!\n");
    }
    else
    {
        printf("Fail to Create Thread.\n");
        return 0;
    }

	while(ros::ok())
	{
		ros::spinOnce();
	    loop_rate.sleep();

	}
	
	// Close serial port
	serial.Close();
	
	return 0;
}
