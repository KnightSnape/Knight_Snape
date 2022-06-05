//
// Created by knight on 22-5-29.
//

#ifndef GUARD_KALMENFOROTHERS_H
#define GUARD_KALMENFOROTHERS_H

#include"trajectory.h"
#include"global.h"
#include"autoaim.h"
#include"KalmenBas.h"

class KalmenForOthers
{
    public:

        bool  predict(Detection_package &data,RobotCMD &send ,cv::Mat &im_show,Robotstatus &getdata);
        Eigen::Vector3d PnP_get_pc(const cv::Point2f p[4],int armor_number);
        struct ROI
        {
            bool ROI_selected = false;
            cv::Rect2f ROI_bbox;
            int last_class = -1;
            ROI() = default;
            ROI(cv::Rect2f &&bbox,int &last):ROI_selected(true),ROI_bbox(bbox),last_class(last){}
            inline void clear()
            {
                ROI_selected = false;
                last_class = -1;
            }
            ~ROI() = default;
        };

    private:

    ROI roi;
    KalmanBas kalmen;


    cv::Mat F_Mat;//
    cv::Mat C_Mat;//
    Matrix33d F;
    Eigen::Matrix<double,5,1> C;

    cv::Mat R_CI_Mat;
    Matrix33d R_CI;
    inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc, const Eigen::Matrix3d &R_IW)
    {
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }

    inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_IW)
    {
        auto R_WC = (R_CI * R_IW);
        return R_WC * pw;
    }

    inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc)
    {
        return F*pc/pc(2,0);
    }

    inline void re_project_point(cv::Mat &image,const Eigen::Vector3d &pw,const Matrix33d &R_IW,const cv::Scalar &color)
    {
        Eigen::Vector3d pc = pw_to_pc(pw,R_IW);
        Eigen::Vector3d pu = pc_to_pu(pc);
        cv::circle(image,{(int)(pu(0,0)),(int)(pu(1,0))},3,color,2);
    }

    inline double getDis(double A,double B)
    {
        return sqrt(A*A+B*B);
    }


};
#endif //GUARD_KALMENFOROTHERS_H
