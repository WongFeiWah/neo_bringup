//
// Created by huang on 18-6-7.
//
#include <neo_bringup/sonar.h>
#include <neo_bringup/robot.h>
#include <math.h>
#include <cstdlib>
#include <cstring>

SOUND_TIME sound_time;

const float T = 0.1;
const int F = 2;
const int  alpha = 20;
static float x1_old[8] = {0};
static float x2_old[8] = {0};

int sgn(float num)
{
    if(num > 0)
        return 1;
    else if (num == 0)
        return 0;
    else
        return -1;

}

float gsat(float a, float z, float b)
{
    if(z > b)
        return b;
    else if(z >= a && z <= b)
        return z;
    else
        return a;

}

float SlidingFilter(float distance, float &x1_old, float &x2_old){
    float result = 0.0f;
    float x2 = 0.0f;
    float x2_ = 0.0f;
    if( distance > 0.5f ){
        distance = 0.5f;
    }
    x2_= sgn(x1_old - distance) * (F*T - sqrt(F*F*T*F + 2*F*abs(x1_old -distance)));
    x2 = x2_old -gsat(gsat(-T*F*alpha, x2_old,-T*F), x2_old - x2_, gsat(T*F, x2_old, alpha*T*F));
    result = T*x2 + x1_old;
    x2_old = x2;
    x1_old = result;

    return result;
}

Sonar::Sonar(void *data) {
    memcpy(&this->sonar_data, data, sizeof(this->sonar_data));
}

Sonar::~Sonar() {

}

void Sonar::doAction() {
    neo_bringup *ctl = GetHandleNode();

    if(ctl == NULL) return;

    float distance[8] = {0};

    distance[0] = SlidingFilter(sonar_data.sonar0_distance, x1_old[0], x2_old[0]);
    distance[1] = SlidingFilter(sonar_data.sonar1_distance, x1_old[1], x2_old[1]);
    distance[2] = SlidingFilter(sonar_data.sonar2_distance, x1_old[2], x2_old[2]);
    distance[3] = SlidingFilter(sonar_data.sonar3_distance, x1_old[3], x2_old[3]);
    //printf("Sonar::doAction()\n");
    ctl->PublishSound_wave(distance);
}




