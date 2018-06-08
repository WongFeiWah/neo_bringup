//
// Created by huang on 18-6-8.
//

#ifndef NEO_BRINGUP_ODOM_H
#define NEO_BRINGUP_ODOM_H

#include <neo_bringup/CThreadPool.h>

typedef struct {
    int left_encoder;
    int right_encoder;
}odom_data;

class Odom:public CTask
{
public:
    Odom(void *data);
    ~Odom();
    void doAction();
private:
};

#endif //NEO_BRINGUP_ODOM_H
