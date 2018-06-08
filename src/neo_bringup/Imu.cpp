//
// Created by huang on 18-6-8.
//
#include <neo_bringup/Imu.h>
#include <neo_bringup/robot.h>

IMU_MSG   imu_info = {0};
double q[4] = {0};
sensor_msgs::Imu imu_msg;


void imu_handle(void*  read_data)
{
    //pthread_mutex_lock(&Device_mutex);
    double th = 0.0;
    neo_bringup *ctl = GetHandleNode();

    memcpy(&imu_info, read_data, sizeof(IMU_MSG));
#if 0
    printf("------\r\n");
	th = atan2f(q[1]*q[2] + q[0]*q[3],0.5f - q[2]*q[2] - q[3]*q[3])*180/3.141592653;

	printf("imu: %f\r\n",th);
	printf("acc: \n%f  \n%f  \n%f\r\n  ",
		(double)imu_info.acc[0],(double)imu_info.acc[1],(double)imu_info.acc[2]);

	printf("gyro: \n%f  \n%f  \n%f\r\n",(double)imu_info.gyro[0],(double)imu_info.gyro[1],(double)imu_info.gyro[2]);

	printf("q: \n%f  \n%f  \n%f \n%f\r\n",(double)imu_info.w,(double)imu_info.x,(double)imu_info.y,(double)imu_info.z);
#endif
    if( imu_info.gyro[0] == 0.0
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

    //imu_pub.publish(imu_msg);
    ctl->PublishImu(imu_msg);
/*
	static tf::TransformBroadcaster tfbroadcaster;
	tfs_msg.header.stamp    = ros::Time::now();
	tfs_msg.header.frame_id = "base_link";
	tfs_msg.child_frame_id  = "imu_link";
	tfs_msg.transform.rotation.w = imu_info.w;
	tfs_msg.transform.rotation.x = imu_info.x;
	tfs_msg.transform.rotation.y = imu_info.y;
	tfs_msg.transform.rotation.z = imu_info.z;

	tfs_msg.transform.translation.x = 0.0;
	tfs_msg.transform.translation.y = 0.0;
	tfs_msg.transform.translation.z = 0.15;*/

//	tfbroadcaster.sendTransform(tfs_msg);
}


Imu::Imu(void *data) {

}

Imu::~Imu() {

}

void Imu::doAction() {

}
