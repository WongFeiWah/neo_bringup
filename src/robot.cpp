/*
---------+---+----+----------------------------------+--------+---+---+----+
|  Header 				|Type	|   Len  |      Payload 12 bytes        		 	|Checksum	|Res|Res|End |
+---------+----+---+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+-_-+----+
|0x55			|0xAA	|0x03|    short    | 2 Int 2 short            	| 0xZZ   		|ZZ |ZZ |0x0A|
+----+----+---+----+-+--+--+--+--+--+--+--+--+--+--+--+--------+---+---+----+

*/
#include <neo_bringup/robot.h>

myserial serial;

typedef struct {
	float value1;
	float value2;
	float value3;
}send_data;

unsigned char cmd_package[18];

typedef struct {
	double goal_linear_velocity;
	double goal_angular_velocity;
}CMD_MSG;
CMD_MSG cmd_msg;

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

//static int resetCount = 0;
//static bool sendOK = true;
//static bool lockSerialData = false;
//float distance;
//std_msgs::Float32MultiArray  n;

neo_bringup::neo_bringup():
	nh()
	// tfbroadcaster()
{
	
	odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 100);
	imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1000);
    sound_wave_publisher0 = nh.advertise<sensor_msgs::Range>("sound_wave0", 100);
    sound_wave_publisher1 = nh.advertise<sensor_msgs::Range>("sound_wave1", 100);
    sound_wave_publisher2 = nh.advertise<sensor_msgs::Range>("sound_wave2", 100);
    sound_wave_publisher3 = nh.advertise<sensor_msgs::Range>("sound_wave3", 100);
	sub = nh.subscribe("cmd_vel", 1000, &neo_bringup::callbackCmdvel,this);

}

neo_bringup::~neo_bringup(){


}
void neo_bringup::PublishOdom(nav_msgs::Odometry &odom)
{
	odom_publisher.publish(odom);

	//static tf::TransformBroadcaster tfbroadcaster;
	geometry_msgs::TransformStamped odom_tf_;
	geometry_msgs::TransformStamped BaseLink_tf_;
	odom_tf_.header.stamp = ros::Time::now();
	odom_tf_.header.frame_id = "odom";
	odom_tf_.child_frame_id = "base_footprint";

	odom_tf_.transform.translation.x = odom.pose.pose.position.x;
	odom_tf_.transform.translation.y = odom.pose.pose.position.y;
	odom_tf_.transform.translation.z = odom.pose.pose.position.z;

	odom_tf_.transform.rotation = odom.pose.pose.orientation;
	// tfbroadcaster.sendTransform(odom_tf_);

	BaseLink_tf_.header.stamp = ros::Time::now();

	BaseLink_tf_.header.frame_id = "base_footprint";
	BaseLink_tf_.child_frame_id = "base_link";
	BaseLink_tf_.transform.rotation.w = 1.0;
	BaseLink_tf_.transform.rotation.x = 0.0;
	BaseLink_tf_.transform.rotation.y = 0.0;
	BaseLink_tf_.transform.rotation.z = 0.0;

	BaseLink_tf_.transform.translation.x = 0.0;
	BaseLink_tf_.transform.translation.y = 0.0;
	BaseLink_tf_.transform.translation.z = 0.0;
	// tfbroadcaster.sendTransform(BaseLink_tf_);
}
void neo_bringup::PublishImu(sensor_msgs::Imu &Imu)
{
	imu_publisher.publish(Imu);

	geometry_msgs::TransformStamped tfs_msg_;
	//static tf::TransformBroadcaster tfbroadcaster;
	tfs_msg_.header.stamp	= ros::Time::now();
	tfs_msg_.header.frame_id = "base_link";
	tfs_msg_.child_frame_id	= "imu_link";
	tfs_msg_.transform.rotation.w = Imu.orientation.w;
	tfs_msg_.transform.rotation.x = Imu.orientation.x;
	tfs_msg_.transform.rotation.y = Imu.orientation.y;
	tfs_msg_.transform.rotation.z = Imu.orientation.z;

	tfs_msg_.transform.translation.x = 0.0;
	tfs_msg_.transform.translation.y = 0.0;
	tfs_msg_.transform.translation.z = 0.15;

	// tfbroadcaster.sendTransform(tfs_msg_);

	
}
void neo_bringup::callbackCmdvel(const geometry_msgs::Twist& cmdvel_)
{
	cmd_package[0] = 0x38;
	cmd_package[1] = sizeof(CMD_MSG);	
	cmd_msg.goal_angular_velocity = cmdvel_.angular.z;
	cmd_msg.goal_linear_velocity = cmdvel_.linear.x; 
	memcpy(&cmd_package[2],&cmd_msg,sizeof(CMD_MSG));
	serial.Write(cmd_package, sizeof(cmd_package));
  //ROS_INFO("hahhahahahh.");
}

void neo_bringup::PublishSound_wave(float *distance_){
  sonar0.header.stamp = ros::Time::now();
  sonar0.header.frame_id = "sonar0";
  sonar0.radiation_type = 0; 
  sonar0.field_of_view = 15 * M_PI / 180;
  sonar0.min_range = 0.02;
  sonar0.max_range = 4.0;
  sonar0.range = distance_[0];
  
  sonar1.header.stamp = ros::Time::now();
  sonar1.header.frame_id = "sonar1";
  sonar1.radiation_type = 0;
  sonar1.field_of_view = 15 * M_PI / 180;
  sonar1.min_range = 0.02;
  sonar1.max_range = 4.0;
  sonar1.range = distance_[1];
 
  sonar2.header.stamp =ros::Time::now();
  sonar2.header.frame_id = "sonar2";
  sonar2.radiation_type = 0;
  sonar2.field_of_view = 15 * M_PI / 180;
  sonar2.min_range = 0.02;
  sonar2.max_range = 4.0;
  sonar2.range = distance_[2];
 
  sonar3.header.stamp = ros::Time::now();
  sonar3.header.frame_id ="sonar3";
  sonar3.radiation_type = 0;
  sonar3.field_of_view = 15 * M_PI / 180;
  sonar3.min_range = 0.02;
  sonar3.max_range = 4.0;
  sonar3.range = distance_[3];
  sound_wave_publisher0.publish(sonar0); 
  sound_wave_publisher1.publish(sonar1);
  sound_wave_publisher2.publish(sonar2);
  sound_wave_publisher3.publish(sonar3);
}
	  
static neo_bringup *neo_bringup_node = NULL;
static CThreadPool *serialHandlePool = NULL;




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

void* package_thread(void* buf)
{
    pthread_detach(pthread_self());
    
    ST_PACKAGE_DATA readData;
    ST_PACKAGE_DATA *p_buf = (ST_PACKAGE_DATA *)buf;
    memcpy(&readData,buf,p_buf->len+2);
    //lock = 1;
    //pthread_mutex_lock(&Device_mutex);
	unsigned short len = 0;
	//ROS_INFO("PACKAGE  %2X",buf[2])
	switch(readData.mode)
	{
		case 0:
			break;
		case 1:
//			printf("len = %d, %c\n",readData.len,readData.mode);
			//odom_handle(readData.data);
			
			break;
		case 2:
			break;
		case 3:
			break;
		case 'I':
			//printf("len = %d, %c\n",readData.len,readData.mode);
			//imu_handle(readData.data);
			break;
        case 0x50:
    	    //sound_handle(readData.data);
            break;
		case 0x97:
			//ROS_INFO("LOG");
			//LOG_Handle(readData.data,readData.len);
			break;
		case 0x98:
			//sendOK = true;
			break;
		case 'T':
			//if(readData.data[0] != 0xDD)return NULL;

            //printf("TEST===================RECV \n");

			//resetCount++;
			break;
		default:
			break;
	}
	//pthread_mutex_unlock(&Device_mutex);

	return NULL;
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
        ret = serial.Read(&ch,1,10);
		
        if(ret == 1)
  	    {
            if(head.head == 0)
	        {
                if(ch != 0x40)continue;
                head.head = ch;
            }else if(head.len == 0){
                if(ch == 0)
	              	{
                    memset(&head,0,sizeof(ST_PACKAGE_HEAD));			
                    continue;
                  }
                head.len = ch;
            }else if(head.mode == 0){
                head.mode = ch;
                if( serial.Read(readData.data,head.len,10) == head.len ){
                    readData.mode = head.mode;
                    readData.len = head.len;

                    if(serialHandlePool != NULL){
                        switch(readData.mode)
                        {
                            case 0x50:
                                serialHandlePool->addTask(new Sonar(readData.data));
                                break;
                            default:
                                break;
                        }
                    }
                }
                memset(&head,0,sizeof(ST_PACKAGE_HEAD));
            }
        }
    }
    printf("serial end.\n");
}

neo_bringup *GetHandleNode()
{
    return neo_bringup_node;
}

int  main(int argc, char** argv)
{   
	unsigned char buffer[512] = {0};
	int i, length = 0;
	int readflag = 0;
	int match_flag = 0;
	char ch = 0;
	unsigned char  send_test = 10;
	ros::init(argc,argv,"neo_bringup_node");

	neo_bringup node;
	neo_bringup_node = &node;

    serialHandlePool = new CThreadPool(20);

  	if(argc > 1){
		if(serial.Open((char*)argv[1], 115200, 8, NO, 1) == 0){
            printf("not found serial.\n");
			return 0;
		}
	}else{
        if(serial.Open((char*)"/dev/ttyUSB0", 115200, 8, NO, 1) == 0){
          printf("no serial com\n");
          return 0;
        }
	}
	ros::Rate loop_rate(100);

	pthread_t id;
	bool ret;
	ret = pthread_create(&id,NULL,robot_serial_thread,NULL);
	if(!ret)
	{
		printf("succeed to create robot_serial_thread!\n");
	}
	else
	{
		printf("Fail to Create Thread.\n");
		return 0;
	}
	ros::spin();
	serial.Close();
	return 0;
}
