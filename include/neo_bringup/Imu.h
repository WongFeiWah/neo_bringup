//
// Created by huang on 18-6-8.
//

#ifndef NEO_BRINGUP_IMU_H
#define NEO_BRINGUP_IMU_H

typedef struct{
    short acc[3];
    short gyro[3];

    float w;
    float x;
    float y;
    float z;
}IMU_MSG;

#endif //NEO_BRINGUP_IMU_H
