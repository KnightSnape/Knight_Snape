//
// Created by knight on 22-5-29.
//

#include "PredictEKF.h"



void EKF_CTRV::predict()
{
    this->xpos = this->X(0,0);
    this->ypos = this->X(1,0);
    this->Vnow = this->X(2,0);
    this->alpha = this->X(3,0);
    this->Omiga = this->X(4,0);

    double T_2 = this->T * this->T;
    double T_3 = this->T * T_2;
    double T_4 = this->T * T_3;

    std::default_random_engine e;
    std::normal_distribution<> J(0,0.1);

    a = J(e);
    yaw = J(e);

    double A_x_k,A_y_k;
}