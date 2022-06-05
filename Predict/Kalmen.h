//
// Created by knight on 22-5-27.
//

#ifndef GUARD_KALMEN_H
#define GUARD_KALMEN_H
#include "global.h"
#include "trajectory.h"
#include "TRTModule.h"
#include "autoaim.h"
#include "KalmenBas.h"
#include "PredictEKF.h"
//test
#define MAXN 500
class KalmenWorking
{

    public:
        //main predict operation(choose,pnp,KF OR EKF and so on)
        bool predict(Detection_package &,RobotCMD &,cv::Mat &,bool,Robotstatus &);


        //test
        //void make3Ddata();

        Eigen::Vector3d PnP_get_pc(const cv::Point2f p[4],int armor_number);

        void match_armors(bbox_t &armor,bool &selected,const std::vector<bbox_t> &detections,const Matrix33d &R_IW,const bool right);

        float bbOverlap(const cv::Rect2f &box1,const cv::Rect2f &box2);

        double get_quadrangle_size(const bbox_t &bx);

        inline void load_param(bool update_all = true)
        {
            if(!antitop)
            {

            }
            else
            {

            }
        }

    private:

        EKF_CV ekf;
        int DEAD_BUFFER = 20;

        bool distance_check = false;  // far
        bool antitop = false;
        bool exist_hero = false; //have hero

        bool last_is_one_armor = false;
        bool last_is_two_armors = false;
        bool right = false;
        bool anticlockwise =true;

        double last_time;

        //the last
        std::vector<bbox_t> last_boxes;
        bbox_t last_sbbox;    
        Matrix33d last_R_IW;
        Matrix31d last_m_pw;
        bool last_shoot = false;
        double last_pitch=0,last_yaw=0;

        int dead_buffer = 0; //grey one


        cv::Mat F_Mat;//
        cv::Mat C_Mat;//
        Matrix33d F;
        Eigen::Matrix<double,5,1> C;

        cv::Mat R_CI_Mat;
        Matrix33d R_CI;

        //test
        Matrix31d place3d[MAXN];
        double a;
        double theta;

        struct PredictRecord
        {
            EKF_CV ekf;
            bbox_t bbox;
            Matrix31d last_m_pw;
            bool updated = false, init = false;
            double last_yaw = 0, last_pitch = 0;
            float yaw_angle = 0.,pitch_angle = 0., yaw_speed = 0., pitch_speed = 0.;
            int distance;
            bool distant = false;
            int age = 0;
            PredictRecord(bbox_t &armor):bbox(armor),updated(false),
            init(true),last_yaw(0),last_pitch(0),yaw_angle(0.),pitch_angle(0.),
            yaw_speed(0.),pitch_speed(0.),distant(false),age(0){}
        };

        std::vector<PredictRecord> antitop_candidates;

        Matrix31d NowPlace3d;
        Matrix31d NowV3d;

        Matrix99d A3d;
        Matrix99d B3d;
        Matrix91d x3d;
        Matrix31d z3d;
        Matrix91d u3d;
        Matrix39d H3d;
        Matrix99d P3d;
        Matrix99d Q3d;
        Matrix33d R3d;
        Matrix99d I3d;

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

    inline cv::Point getCenter(cv::Rect rect)
    {
        cv::Point cpt;
        cpt.x = rect.x + cvRound(rect.width/2.0);
        cpt.y = rect.y + cvRound(rect.height/2.0);
        return cpt;
    }

    inline bool is_same_armor_by_distance(const Eigen::Vector3d old_m_pw, const bbox_t &new_armor, const Matrix33d &R_IW,const double distance_threshold = 0.15)
    {
        Matrix31d new_m_pc = PnP_get_pc(new_armor.pts, new_armor.ID);
        Matrix31d new_m_pw = pc_to_pw(new_m_pc, R_IW);
        Matrix31d m_pw_delta = new_m_pw - old_m_pw;
        double Distance = m_pw_delta.norm();
        if(Distance < distance_threshold)
            return true;
        else
            return false;
    }

    inline cv::Point2f points_center(cv::Point2f pts[4])
    {
        cv::Point2f center;
        center.x = (pts[0].x+pts[1].x+pts[2].x+pts[3].x)/4;
        center.y = (pts[0].y+pts[1].y+pts[2].y+pts[3].y)/4;
        return center;
    }

    //get ROI
    inline cv::Rect2f get_ROI(bbox_t &armor, float coefficient = 1.0f)
    {
        auto center = points_center(armor.pts);
        auto w = std::max({armor.pts[0].x,armor.pts[1].x,armor.pts[2].x,armor.pts[3].x})
                -std::min({armor.pts[0].x,armor.pts[1].x,armor.pts[2].x,armor.pts[3].x});
        auto h = std::max({armor.pts[0].y,armor.pts[1].y,armor.pts[2].y,armor.pts[3].y})
                 -std::min({armor.pts[0].y,armor.pts[1].y,armor.pts[2].y,armor.pts[3].y});

        return cv::Rect2f(center.x - w/2,center.y-h/2 , w*coefficient, h*coefficient);
    }



    inline void clear()
    {
        last_boxes.clear();
        last_shoot = false;
        antitop = false;
        dead_buffer = 0;
        antitop_candidates.clear();
    }

    inline double getTheta(double a,double b)
    {
        return atan2(b,a);
    }

    inline double getDis(double a,double b)
    {
        return sqrt(a*a+b*b);
    }

    inline double getDis2(double a,double b,double c)
    {
        return sqrt(a*a+b*b+c*c);
    }

    inline void makeAngle(Matrix31d pw,float &pitch,float &yaw,float &Distance)
    {
        yaw = getTheta(pw(0,0),pw(2,0));
        pitch = getTheta(getDis(pw(0,0),pw(1,0)),pw(2,0));
        Distance = getDis2(pw(0,0),pw(1,0),pw(2,0));
    }


};

#endif //GUARD_KALMEN_H
