//
// Created by knight on 22-5-29.
//

#include "KalmenForOthers.h"


cv::Point2f points_center(cv::Point2f pts[4])
{
    for(int i=0;i<4;i++)
    {
        for(int j=i+1;j<4;j++)
        {
            if(pts[i] == pts[j])
            {
                cout<<"[Error] Unable to calculate center point"<<endl;
                return cv::Point2f{0,0};
            }
        }
    }
    cv::Point2f center(0,0);
    if(pts[0].x == pts[2].x && pts[1].x == pts[3].x)
    {
        cout<<"[Error] Unalbe to calculate center point"<<endl;
    }
    else if(pts[0].x == pts[2].x && pts[1].x != pts[3].x)
    {
        center.x = pts[0].x;
        center.y = (pts[3].y - pts[1].y)/(pts[3].x - pts[1].x)*(pts[0].x - pts[3].x)+pts[3].y;
    }
    else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x)
    {
        center.x = pts[1].x;
        center.y = (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * (pts[1].x - pts[0].x) + pts[0].y;
    }
    else
    {
        center.x = (((pts[3].y - pts[1].y) / (pts[3].x - pts[1].x) * pts[3].x - pts[3].y + \
            pts[0].y - (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * pts[0].x)) / \
            ((pts[3].y - pts[1].y) / (pts[3].x - pts[1].x) - (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x));
        center.y = (pts[2].y - pts[0].y) / (pts[2].x - pts[0].x) * (center.x - pts[0].x) + pts[0].y;
    }

    return center;

}

bool KalmenForOthers::predict(Detection_package &data,RobotCMD &send ,cv::Mat &im_show,Robotstatus &getdata)
{
    auto &[detections,img,q_,t0] = data;
    im_show = img.clone();

    Eigen::Quaternionf q_raw(q_[0],q_[1],q_[2],q_[3]);
    Eigen::Quaternionf q(q_raw.matrix().transpose());

    Matrix33d R_IW = q.matrix().cast<double>();

    auto robot_status = getdata;

    if(detections.empty())
    {
        roi.clear();
        return false;
    }

    std::vector<bbox_t> new_detections;

    for(auto &d: detections)
    {
        if((int)robot_status.color == d.color&&d.confident >= 0.5f && d.ID!=0 &&d.ID!=2 && d.ID!=6)
            new_detections.push_back(d);
    }

    if (new_detections.empty())
    {
        roi.clear();
        return false;
    }



    std::sort(detections.begin(),detections.end(),[](const auto &bx,const auto &by)
    {
        auto K1_x = bx.pts[0].x*bx.pts[1].y - bx.pts[1].x*bx.pts[0].y;
        auto K2_x = bx.pts[1].x*bx.pts[2].y - bx.pts[2].x*bx.pts[1].y;
        auto K3_x = bx.pts[2].x*bx.pts[3].y - bx.pts[3].x*bx.pts[2].y;
        auto K4_x = bx.pts[3].x*bx.pts[0].y - bx.pts[0].x*bx.pts[3].y;
        auto S_1 = 0.5 *fabs(K1_x+K2_x+K3_x+K4_x);
        auto K1_y = by.pts[0].x*bx.pts[1].y - by.pts[1].x*bx.pts[0].y;
        auto K2_y = by.pts[1].x*bx.pts[2].y - by.pts[2].x*bx.pts[1].y;
        auto K3_y = by.pts[2].x*bx.pts[3].y - by.pts[3].x*bx.pts[2].y;
        auto K4_y = by.pts[3].x*bx.pts[0].y - by.pts[0].x*bx.pts[3].y;
        auto S_2 = 0.5 *fabs(K1_y+K2_y+K3_y+K4_y);


        return S_1 > S_2;
    });

    auto &armor = new_detections.front();

    if(roi.ROI_selected)
    {
        bool flag = false;
        for(auto &d: new_detections)
        {
            auto center = points_center(d.pts);
            if(d.confident >= 0.2f &&(center.inside(roi.ROI_bbox)))
            {
                armor = d;
                flag = true;
                break;
            }
        }
        if(!flag)
        {
            for(auto &d:new_detections)
            {
                if(d.ID == roi.last_class)
                    armor = d;
            }
        }
    }
    auto center = points_center(armor.pts);
    if(center.x!=0||center.y!=0)
    {
        roi.ROI_selected = true;
        roi.last_class = armor.ID;
        roi.ROI_bbox = cv::Rect2f(center.x,center.y,(armor.pts[2].x+armor.pts[3].x-armor.pts[0].x-armor.pts[1].x)*0.8, \
                                  (armor.pts[1].y + armor.pts[2].y - armor.pts[0].y - armor.pts[3].y)*0.8);
    }
    else
    {
        roi.ROI_selected = false;
        cout<<"[Warning] ROI is illegal"<<endl;
    }

    Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
    Matrix31d m_pw = pc_to_pw(m_pc,R_IW);

    static double last_yaw = 0,last_speed = 0;
    //如果在这个地方数据偏邪门就改成1
    double mc_yaw = std::atan2(m_pc(2,0),m_pc(0,0));
    double m_yaw = std::atan2(m_pw(2,0),m_pw(0,0));

    if(DEBUG)
        cout<<"m_yaw"<<m_yaw * 180 / PI<<endl;
    if(std::fabs(last_yaw - m_yaw) > 5. / 180. *PI)
    {
        kalmen.reset(m_yaw,t0);
        last_yaw = m_yaw;

        cout<<"reset"<<endl;
        return false;
    }

    last_yaw = m_yaw;
    Matrix11d z_k{m_yaw};
    Matrix21d state = kalmen.update(z_k,t0);
    last_speed = state(1,0);
    double c_yaw = state(1,0);
    double c_speed = state(1,0) * m_pw.norm();

    double predict_time = m_pw.norm() / robot_status.robot_speed + shoot_delay;
    double p_yaw = c_yaw + atan2(predict_time + c_speed, m_pw.norm());

    double length = getDis(m_pw(0,0),m_pw(1,0));

    Matrix31d p_pw;
    p_pw << length*cos(p_yaw),length*sin(p_yaw),m_pw(2,0);
    Trajectory trajectory;
    float get_pitch;
    Matrix31d s_pw = p_pw;
    float v0 = robot_status.robot_speed;
    trajectory.trajectory_predict(s_pw,get_pitch,v0);
    Matrix31d s_pc = pw_to_pc(s_pw,R_IW);

    re_project_point(im_show,m_pw,R_IW,{0,255,0});
    re_project_point(im_show,p_pw,R_IW,{0,0,255});
    re_project_point(im_show,s_pw,R_IW,{255,0,0});
    for(int i=0;i<4;i++)
    {
        cv::circle(im_show,armor.pts[i],3,{255,255,0});
    }
    cv::circle(im_show,{im_show.cols/2,im_show.rows/2},3,{0,255,255});

    double s_yaw = atan(s_pc(0,0)/s_pc(2,0)) / PI * 180;
    double s_pitch = atan(s_pc(1,0)/s_pc(2,0)) / PI * 180;

    send.yaw_angle = (float) s_yaw;
    send.pitch_angle = (float) s_pitch;

    //send

    if(std::abs(send.yaw_angle)<90&&std::abs(send.pitch_angle) <90)
        send.shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
    else
        send.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);

    return true;

}
//complete
Eigen::Vector3d KalmenForOthers::PnP_get_pc(const cv::Point2f p[4],int armor_number)
{

    static const std::vector<cv::Point3d> pw_small=
            {
                    {-armor_small_l , armor_small_w, 0},//- +
                    {armor_small_l , armor_small_w, 0},//+ +
                    {armor_small_l , -armor_small_w, 0},//+ -
                    {-armor_small_l , -armor_small_w, 0} //- -
            };

    static const std::vector<cv::Point3d> pw_big=
            {
                    {-armor_big_l,armor_big_w,0},  //- +
                    {armor_big_l,armor_big_w,0},  // + +
                    {armor_big_l,-armor_big_w,0},  // + -
                    {-armor_big_l,-armor_big_w,0}  // - -
            };

    std::vector<cv::Point2d> pu(p,p+4);

    cv::Mat rvec,tvec;
    //solvePnP
    if(armor_number==0||armor_number==1)
        cv::solvePnP(pw_big,pu,F_Mat,C_Mat,rvec,tvec);
    else
        cv::solvePnP(pw_small,pu,F_Mat,C_Mat,rvec,tvec);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec,pc);

    pc<< pc(0,0) + guard_pc_x,pc(1,0)+guard_pc_y,pc(2,0)+guard_pc_z;

    return pc;
}
