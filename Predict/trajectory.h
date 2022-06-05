//
// Created by knight on 22-5-27.
//

#ifndef GUARD_TRAJECTORY_H
#define GUARD_TRAJECTORY_H
#include "global.h"
class Trajectory
{
    public:
        Trajectory() = default;
        ~Trajectory() = default;

        float getDis(float a,float b);

        float getTheta(float a,float b);

        float CalT(float dis,float v0);

        void initConst();

        void trajectory_cal(float dis,float theta,float &y,float &T);

        void trajectory_predict(Matrix31d &pw,float &finalpitch,float &getV);

    private:

        float g = 9.8;//gravity
        float k = 0.020;//k/m
        float pitch;
        float yaw;
        float distance;
        float height;
        float v;

        float v_2;
        float g_k;
        float gk;
        float k_g;
        float g2_k;
        float g2k;
        float k2_g;

        Matrix33d transitional_matrix;



};

#endif //GUARD_TRAJECTORY_H

