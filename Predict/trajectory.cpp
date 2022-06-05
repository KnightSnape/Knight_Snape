//
// Created by knight on 22-5-27.
//


#include "trajectory.h"
#include "global.h"

float Trajectory::getDis(float a,float b)
{
    return sqrt(a*a+b*b);
}

float Trajectory::getTheta(float a,float b)
{
    return atan2(b,a);
}

float Trajectory::CalT(float dis,float v0)
{
    return (expf(k*dis)-1)/(k*v0);
}
void Trajectory::initConst()
{
    this->v_2 = v * v;
    this->g_k = g / k;
    this->gk = g * k;
    this->k_g = k / g;
    this->g2_k = sqrt(g_k);
    this->g2k = sqrt(gk);
    this->k2_g = sqrt(k_g);
}

void Trajectory::trajectory_cal(float dis,float theta,float &y,float &T)
{
    float vx = v * cos(theta);
    float vy = v * sin(theta);
    T = this->CalT(dis,v);
    float vy2 = vy * vy;
    if(theta < 0)
    {
        y = ((-0.5f) * logf(1 - (vy2 * k_g))) - (1.0 / k) * logf(coshf(T*g2k-atanhf(vy*k2_g)));
    }
    else
    {
        float C =(atanf(vy * k2_g)/g2k);
        if(C>T)
        {
            y = (1.0 / k) * logf(cosf(g2k * (C - T)) / cosf(g2k * C));
        }
        else
        {
            y = (1.0 / (2 * k) * logf(k_g * vy2 + 1)) - (1.0 / k * logf(coshf(g2k * (C - T))));
        }
    }
}

void Trajectory::trajectory_predict(Matrix31d &pw,float &finalpitch,float &getV)
{

    distance = getDis(pw(0,0),pw(2,0));
    height = pw(1,0);
    //distance = 7.5;
    //height = 1.0;
    //v = 30;
    v = getV;
    //auto Time_begin = chrono::high_resolution_clock::now(); //测时间用的
    float ytemp = height;
    float Y,T;//actural Y,time
    this->initConst();
    pitch = getTheta(distance,ytemp);
    while(1)
    {
        trajectory_cal(distance,pitch,Y,T);
        float dy = height - Y;

        if(DEBUG)
            cout << "Y="<<Y <<",dy="<<dy << endl;
        if (fabsf(dy)<0.0008)
        {
            pw<<(pw(0,0),Y,pw(2,0));

            //cout << "done"<<endl;
            //auto Time_end = chrono::high_resolution_clock::now(); //计算函数所花的时间
            //mid += (static_cast<chrono::duration<double, milli>>(Time_end - Time_begin)).count();
            //cout << "计算用时:" << (static_cast<chrono::duration<double, milli>>(Time_end - Time_begin)).count() << " ms" << endl;
            pitch = getTheta(distance,ytemp);
            finalpitch = pitch;
            return;
        }
        ytemp+= dy;
        pitch = getTheta(distance,ytemp);
        if(DEBUG)
        cout << ytemp << endl;

    }
}