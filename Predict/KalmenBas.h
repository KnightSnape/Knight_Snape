//
// Created by knight on 22-5-29.
//

#ifndef GUARD_KALMENBAS_H
#define GUARD_KALMENBAS_H
#include"global.h"

class KalmanBas
{
    public:
        KalmanBas() = default;

        KalmanBas(Matrix22d A,Matrix12d H,Matrix22d R,Matrix11d Q,Matrix21d init,double t)
        {
            reset(A,H,R,Q,init,t);
        }

        void reset(Matrix22d A,Matrix12d H,Matrix22d R,Matrix11d Q,Matrix21d init,double t0)
        {
            this->A = A;
            this->H = H;
            this->P = Matrix22d::Zero();
            this->R = R;
            this->Q = Q;
            x_k1 = init;
            last_t = t0;
        }

        void reset(double x,double t0)
        {
            x_k1(0,0) = x;
            last_t = t0;
        }

        Matrix21d update(Matrix11d z_k,double t0)
        {
            for(int i=1;i<2;i++)
            {
                A(i-1,i) = t0 - last_t;
            }
            last_t = t0;
            Matrix21d p_x_k = A*x_k1;
            P = A*P*A.transpose()+R;

            K=P*H.transpose()*(H*P*H.transpose()+Q).inverse();
            x_k1 = p_x_k+K*(z_k - H*p_x_k);
            P = (Matrix22d::Identity()-K*H)*P;
            return x_k1;
        }
    private:
        Matrix21d x_k1;
        Matrix22d A;
        Matrix21d K;
        Matrix12d H;
        Matrix22d R;
        Matrix11d Q;
        Matrix22d P;
        double last_t{0};

};

class KalmanForWind
{
public:
    KalmanForWind() = default;

    KalmanForWind(Matrix33d A,Matrix33d H,Matrix33d R,Matrix33d Q,Matrix31d init,double t)
    {
        reset(A,H,R,Q,init,t);
    }

    void reset(Matrix33d A,Matrix33d H,Matrix33d R,Matrix33d Q,Matrix31d init,double t0)
    {
        this->A = A;
        this->H = H;
        this->P = Matrix33d::Zero();
        this->R = R;
        this->Q = Q;
        x_k1 = init;
        last_t = t0;
    }

    void reset(Matrix31d x,double t0)
    {
        x_k1= x;
        last_t = t0;
    }

    Matrix31d update(Matrix31d z_k,double t0)
    {
        for(int i=1;i<3;i++)
        {
            A(i-1,i) = t0 - last_t;
        }
        last_t = t0;
        Matrix31d p_x_k = A*x_k1;
        P = A*P*A.transpose()+R;

        K=P*H.transpose()*(H*P*H.transpose()+Q).inverse();
        x_k1 = p_x_k+K*(z_k - H*p_x_k);
        P = (Matrix33d::Identity()-K*H)*P;
        return x_k1;
    }
private:
    Matrix31d x_k1;
    Matrix33d A;
    Matrix33d K;
    Matrix33d H;
    Matrix33d R;
    Matrix33d Q;
    Matrix33d P;
    double last_t{0};

};



#endif //GUARD_KALMENBAS_H
