//
// Created by huang on 18-6-8.
//
#include <neo_bringup/sonar_node.h>

myserial serial;

float readDistance(myserial *serial_,unsigned char num)
{
    float result = 0.0f;
    unsigned char buf[3] = {0xe8,0x02,0x30};
    unsigned char readBuf[2] = {0};


    buf[2] = num;
    serial_->Write(buf,3);
    serial_->Read(readBuf,2,10);
    result = (readBuf[0]*256+readBuf[1])/1000.0;
    printf("%02X:%.3fm  \n",num,result);

    return result;
}

int  main(int argc, char** argv)
{
    unsigned char buffer[512] = {0};
    int i, length = 0;
    int readflag = 0;
    int match_flag = 0;
    char ch = 0;
    unsigned char  send_test = 10;
    ros::init(argc,argv,"neo_sonar_node");
    ros::NodeHandle nh;

    printf("sonar_node\n");
    if(argc > 1){
        if(serial.Open((char*)argv[1], 9600, 8, NO, 1) == 0){
            printf("not found serial.\n");
            return 0;
        }
    }else{
        if(serial.Open((char*)"/dev/ttyUSB0", 9600, 8, NO, 1) == 0){
            printf("no serial com\n");
            return 0;
        }
    }
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        readDistance(&serial,0x3e);
        ros::spinOnce();
        loop_rate.sleep();
    }
    serial.Close();
    return 0;
}
