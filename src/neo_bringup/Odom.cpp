//
// Created by huang on 18-6-8.
//
#include <neo_bringup/Odom.h>
#include <neo_bringup/robot.h>

#define TICK2RAD                         0.013
#define LEFT                             0
#define RIGHT                            1
#define WHEEL_RADIUS                     0.033
#define WHEEL_SEPARATION                 0.185           // meter (BURGER : 0.160, WAFFLE : 0.287)
//#define TURNING_RADIUS                   0.1435          // meter (BURGER : 0.080, WAFFLE : 0.1435)
//#define ROBOT_RADIUS                     0.87           // meter (BURGER : 0.105, WAFFLE : 0.220)
//#define ENCODER_MAX                      2147483648      // raw
//#define MOTOR_PULSE                      480

bool init_encoder_[2]  = {false, false};
int  last_diff_tick_[2];
int  last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double odom_pose[3];
float  odom_vel[3];
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;


int    old_encoder[3]={0,0,0};
bool   old_encoder_init = false;
double odometry_x=0;
double odometry_y=0;
double odometry_th=0;
#define VEL_TRANS 329.4531
//circle/min trans to m/s,formula: 1circle/min = 1*(2*pi*0.029)/60 m/s.
#define INVERSE_VEL_TRANS 0.00304
//#define L 0.11
#define PI 3.1415926
double L=0;

ros::Time time_now, prev_update_time;
int current_left_encoder,last_left_encoder;
int current_right_encoder, last_right_encoder;
//geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
nav_msgs::Odometry odom;

static double last_theta = 0.0;

bool updateOdometry(double diff_time )
{
    // ROS_INFO("if you");
    double odom_vel[3];

    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double delta_s, delta_theta,delta_theta1;

    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;
    double temp;
    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
        return false;
    //printf("step_time-------%f\r\n",step_time);
    wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];
    // printf("wheel_l%f\r\n",wheel_l);
    // printf("wheel_r%f\r\n",wheel_r);
    if (isnan(wheel_l))
        wheel_l = 0.0;

    if (isnan(wheel_r))
        wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    // printf("delta_s-------%f\r\n",delta_s);
    //delta_theta = WHEEL_RADIUS * (wheel_l - wheel_r) / WHEEL_SEPARATION;
    //delta_theta1  = atan2f(imu_info.x*imu_info.y + imu_info.w*imu_info.z,
    //                       0.5f - imu_info.y*imu_info.y - imu_info.z*imu_info.z)  -last_theta;
    //last_theta = atan2f(imu_info.x*imu_info.y + imu_info.w*imu_info.z,  0.5f - imu_info.y*imu_info.y - imu_info.z*imu_info.z);
    v = delta_s / step_time;
    w = delta_theta1 / step_time;
//printf("delta_theta%f\r\n",delta_theta);
    // printf("delta_theta1%f\r\n",delta_theta1);
    last_velocity_[LEFT]  = wheel_l / step_time;
    last_velocity_[RIGHT] = wheel_r / step_time;

    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta1 / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta1 / 2.0));
    odom_pose[2] += delta_theta1;
    // temp = odom_pose[2];
    // compute odometric instantaneouse velocity
    odom_vel[0] = v;
    // printf("temp-------%f\r\n",temp);
    odom_vel[1] = 0.0;
    odom_vel[2] = w;
    //printf("w-------%f\r\n",w);
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    // odom.pose.pose.position.x = 0;
    //odom.pose.pose.position.y = 0;
    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_pose[2]);
    odom.pose.pose.orientation = odom_quat;

    // We should update the twist of the odometry
    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];

    //last_theta = atan2f(imu_info.x*imu_info.y + imu_info.w*imu_info.z, 0.5f - imu_info.y*imu_info.y - imu_info.z*imu_info.z);

    //odom.header.stamp = stamp_now;

    tf::TransformBroadcaster tfbroadcaster;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    /* odom_tf.transform.rotation.w = 1.0;
     odom_tf.transform.rotation.x = 0.0;
     odom_tf.transform.rotation.y = 0.0;
     odom_tf.transform.rotation.z = 0.0;

     odom_tf.transform.translation.x = odom_pose[0];
     odom_tf.transform.translation.y = odom_pose[1];
     odom_tf.transform.translation.z = 0.0;*/

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;

    odom_tf.transform.rotation = odom.pose.pose.orientation;
    // tfbroadcaster.sendTransform(odom_tf);
    /*
     tfs_msg.header.stamp = ros::Time::now();
     tfs_msg.header.frame_id = "base_footprint";
     tfs_msg.child_frame_id = "base_link";
     tfs_msg.transform.rotation.w = 1.0;
     tfs_msg.transform.rotation.x = 0.0;
     tfs_msg.transform.rotation.y = 0.0;
     tfs_msg.transform.rotation.z = 0.0;

     tfs_msg.transform.translation.x = 0.0;
     tfs_msg.transform.translation.y = 0.0;
     tfs_msg.transform.translation.z = 0.0;
     tfbroadcaster.sendTransform(tfs_msg);
*/
}

void publishDriveInformation(void)
{
    //ros::NodeHandle n;
    neo_bringup *ctl = GetHandleNode();
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    time_now = ros::Time::now();
    //printf("time_now-----%f\r\n",time_now);
    double step_time = ( time_now - prev_update_time).toSec();
    prev_update_time = time_now;
    //printf("step_time-----%f\r\n",step_time);
    updateOdometry((double)(step_time));
    odom.header.stamp = ros::Time::now();
    //odom_pub.publish(odom);
    ctl->PublishOdom(odom);
}

int  odom_handle(void* read_data)
{

    tf::TransformBroadcaster  odom_broadcaster;
    static geometry_msgs::TransformStamped   odom_tf;
    static geometry_msgs::TransformStamped   tfs_msg;
    odom_data recvData = {0};
    int current_tick;
    double wheel_l, wheel_r;      //       value of wheel [rad]
    double delta_s, delta_theta;

    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]


    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = 0.0;
    v = w = 0.0;
    memcpy(&recvData,read_data,sizeof(odom_data));
    //printf("dsadsa\n");
    if(recvData.left_encoder == 0){
        //printf("sdasdas\n");
    }
    current_tick = recvData.left_encoder;
    // printf("lef::%d\r\n",recvData.left_encoder);
    // printf("right::%d \r\n",  recvData.right_encoder);
    if (!init_encoder_[LEFT])
    {
        last_tick_[LEFT] = current_tick;
        init_encoder_[LEFT] = true;
    }

    last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
    // printf("current_tick::%d\r\n",current_tick);
    // printf("last_diff_tick_[LEFT] :: %d\r\n",last_diff_tick_[LEFT] );
    last_tick_[LEFT] = current_tick;
    last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

    current_tick = recvData.right_encoder;

    if (!init_encoder_[RIGHT])
    {
        last_tick_[RIGHT] = current_tick;
        init_encoder_[RIGHT] = true;
    }

    last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
    // printf("last_diff_tick_[RIGHT]--------%d\r\n",last_diff_tick_[RIGHT]);
    last_tick_[RIGHT] = current_tick;
    last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
    publishDriveInformation();
}


Odom::Odom(void *data) {

}

Odom::~Odom() {

}

void Odom::doAction() {

}
