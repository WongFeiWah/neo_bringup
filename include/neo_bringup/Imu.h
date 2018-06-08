//
// Created by huang on 18-6-8.
//

#ifndef NEO_BRINGUP_IMU_H
#define NEO_BRINGUP_IMU_H

#include <neo_bringup/CThreadPool.h>

typedef struct{
    short acc[3];
    short gyro[3];

    float w;
    float x;
    float y;
    float z;
}IMU_MSG;

class Imu:public CTask
{
public:
    Imu(void *data);
    ~Imu();
    void doAction();
private:
};

#endif //NEO_BRINGUP_IMU_H
