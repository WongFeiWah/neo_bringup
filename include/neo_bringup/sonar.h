//
// Created by huang on 18-6-7.
//

#ifndef NEO_BRINGUP_SONAR_H
#define NEO_BRINGUP_SONAR_H

#include <neo_bringup/CThreadPool.h>

typedef struct{
    float sonar0_distance;
    float sonar1_distance;
    float sonar2_distance;
    float sonar3_distance;
}SOUND_TIME;

class Sonar:public CTask
{
public:
    Sonar(void *data);
    ~Sonar();
    void doAction();
private:
    SOUND_TIME sonar_data;
};

#endif //NEO_BRINGUP_SONAR_H
