//
// Created by knight on 22-5-27.
//

#include "Kalmen.h"



struct LastTimeHelper//you can update the time
{
    LastTimeHelper(double current_time,double &ref_time):c_time(current_time),r_time(ref_time){};
    ~LastTimeHelper() = default;

    double c_time;
    double &r_time;
};

bool KalmenWorking::predict(Detection_package &data,RobotCMD &send,cv::Mat &im_show,bool isshow,Robotstatus &getdata)
{
    //start debug can make it better to solve the problem
    if(DEBUG)
        cout<<"===============start predict===================="<<endl;
    //get data
    auto &[detections,img,q_,t0] = data;

    LastTimeHelper helper(t0,last_time);

    im_show = img.clone();

    int dead_buffer = DEAD_BUFFER;
    antitop = false;
    bool selected = false;

    distance_check = false,exist_hero = false;

    Eigen::Quaternionf q_raw(q_[0],q_[1],q_[2],q_[3]);
    Eigen::Quaternionf q(q_raw.matrix().transpose());
    if(DEBUG)
        cout<<"q = "<<q_[0]<<" "<<q_[1]<<" "<<q_[2]<<" "<<q_[3]<<endl;
    Matrix33d R_IW = q.matrix().cast<double>();

    double delta_t = t0 - last_time;
    Robotstatus robotsta = getdata;

    bbox_t armor;
    //right
    if(detections.empty())
    {
        clear();
        return false;
    }

    std::vector<bbox_t> new_detections;

    bool same_armor = false;
    bool same_id = false;
    bool switch_armor = false;
    bool need_init = false;
    bool this_is_one_armor = false;
    bool this_is_two_armors = false;


    for(auto &d: detections)
    {
        if((int)(robotsta.color) == d.color&& d.ID!=0&&d.ID!=6&&(d.ID!=2||(d.ID==2&&((last_shoot &&
        last_sbbox.ID!=2)||(robotsta.enemy.at(2)*10<=killer_point_for_2&&robotsta.enemy.at(2)*10>0)))))
        {
            if(last_shoot && d.ID == last_sbbox.ID)
                dead_buffer = 0;

            Matrix31d m_pc = PnP_get_pc(d.pts,d.ID);
            Matrix31d m_pw = pc_to_pw(m_pc, R_IW);
            if(m_pw[2] > height_thres)
                continue;// remove higher one
            if(d.ID == 1)
                exist_hero = true;
            if((int)(robotsta.gamestate)==0) // you cannot shoot to fat
            {
                double Distance = m_pw.norm();
                if(Distance > distance_max)
                continue;
            }

            if(d.confident >= high_thres) // higher than you can push
                new_detections.push_back(d);
            else if(d.confident >= low_thres)
            {
                auto center = points_center(d.pts);
                for(auto &tmp: last_boxes)
                {
                    Matrix31d tmp_last_m_pc = PnP_get_pc(tmp.pts,tmp.ID);
                    Matrix31d tmp_last_m_pw = pc_to_pw(tmp_last_m_pc,last_R_IW);
                    if(center.inside(get_ROI(tmp))||is_same_armor_by_distance(tmp_last_m_pw,d,R_IW))
                    {
                        new_detections.push_back(d);
                        break;
                    }
                }
            }
        }
        else if(d.color == 2&& last_shoot && d.ID == last_sbbox.ID && dead_buffer <= dead_buffer_max_size)
        {
            if(robotsta.enemy.at(d.ID)==0)
            {
                dead_buffer = dead_buffer_max_size + 1;
                continue;
            }
            ++dead_buffer;
            new_detections.push_back(d);
        }
    }

    if(new_detections.empty())//no
    {
        clear();
        return false;
    }

    if(robotsta.target_id !=255&&((last_shoot && last_sbbox.ID!= robotsta.target_id)||!last_shoot))
    {
        for(auto &d:new_detections)
        {
            if(d.ID == robotsta.target_id)
            {
                if(DEBUG)
                    cout<<"The other need tag_id is"<<robotsta.target_id<<". "<<endl;
                //need update
                if ((int)(robotsta.gamestate) == 0)
                {
                    Matrix31d m_pc = PnP_get_pc(d.pts, d.ID);
                    Matrix31d m_pw = pc_to_pw(m_pc, R_IW);
                    double Distance = m_pw.norm();
                    if (Distance > distance_max)
                        continue;
                }
                armor = d;
                selected = true;
                same_armor = false;
                same_id = false;
                switch_armor = false;
                need_init = true;
            }
            

        }
    }

    if(!selected && last_shoot)
    {
        for(auto &d : new_detections)
        {
            auto center = points_center(d.pts);
            if(last_shoot&&(center.inside(get_ROI(last_sbbox))||is_same_armor_by_distance(last_m_pw,d,R_IW)))
            {
                armor = d;
                selected = true;
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
                if(DEBUG)
                    cout<<"selected same armor"<<endl;
                break;
            }
        }
    }
    //complete

    if(!selected && last_shoot)
    {
        unsigned int i=0;
        for(auto &d: detections)
        {
            if(last_sbbox.ID == d.ID)
                i++;
        }
        if(i==1)
        {
            this_is_one_armor = true;
            this_is_two_armors = false;
        }
        else if(i==2)
        {
            this_is_one_armor = false;
            this_is_two_armors = true;
        }
        else
        {
            this_is_one_armor = false;
            this_is_two_armors = false;
            cout<<"[WARNING] multiple armors detected!!"<<endl;
        }
        if(last_is_one_armor && this_is_one_armor)
        {
            unsigned int i=0;
            for(auto &d: new_detections)
            {
                if(d.ID == last_sbbox.ID)
                {
                    armor = d;
                    break;
                }
            }
            selected = true;
            same_armor = true;
            same_id = false;
            switch_armor = false;
            need_init = false;
        }
        else if(last_is_two_armors && this_is_two_armors)
        {
            match_armors(armor,selected,new_detections,R_IW,right);
            if(selected)
            {
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            }
        }
        else if(last_is_two_armors && this_is_one_armor && antitop)
        {
            selected = true;
            if((anticlockwise && !right)|| (!anticlockwise && right))
            {
                for(auto &d: new_detections)
                {
                    if(d.ID == last_sbbox.ID)
                    {
                        armor = d;
                        break;
                    }
                }
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            }
            else if((anticlockwise && right)||(!anticlockwise&& !right))
            {
                same_id = true;
                switch_armor = true;

                for(auto &d: new_detections)
                {
                    if(d.ID == last_sbbox.ID)
                    {
                        armor = d;
                        break;
                    }
                }
                if(antitop_candidates.size() != 0 )
                {
                    Matrix31d m_pc = PnP_get_pc(armor.pts, armor.ID);
                    Matrix31d m_pw = pc_to_pw(m_pc,R_IW);
                    unsigned long int i=-1,j=-1;
                    double min_distance = 100;
                    for(auto &ac: antitop_candidates)
                    {
                        ++j;
                        Matrix31d tmp_m_pc = PnP_get_pc(ac.bbox.pts,ac.bbox.ID);
                        Matrix31d tmp_m_pw = pc_to_pw(tmp_m_pc,R_IW);
                        double Distance = (tmp_m_pw - m_pw).norm();
                        if(Distance < min_distance)
                        {
                            i = j;
                            min_distance = Distance;
                        }
                    }
                    PredictRecord r(armor);
                    r.ekf = ekf;
                    r.updated = false;
                    r.init = need_init;
                    r.last_m_pw = last_m_pw;
                    r.last_yaw = last_yaw;
                    r.last_pitch = last_pitch;
                    antitop_candidates.push_back(r);

                    ekf = antitop_candidates[i].ekf;
                    //load_param(false); //没有写
                    last_m_pw = antitop_candidates[i].last_m_pw;
                    last_pitch = antitop_candidates[i].last_pitch;
                    last_yaw = antitop_candidates[i].last_yaw;

                    antitop_candidates.erase(antitop_candidates.begin() + i);

                    same_armor = true;// lock in
                    need_init = false;//you don't need to update
                }
                else
                {
                    selected = false;
                    same_armor = false;
                    same_id = false;
                    switch_armor = false;
                    need_init = true;
                }
            }
        }
        else if(last_is_one_armor && this_is_two_armors && antitop)
        {
            selected = true;
            same_armor = true;
            same_id = false;
            need_init = false;
            bbox_t t1,t2;
            Matrix31d m_pw1,m_pw2;
            bool flag = false;
            //t0-> k
            for(auto &k : new_detections)
            {
                if(k.ID == last_sbbox.ID)
                {
                    Matrix31d tmp_m_pc = PnP_get_pc(k.pts,k.ID);
                    Matrix31d tmp_m_pw = pc_to_pw(tmp_m_pc,R_IW);
                    if(!flag)
                    {
                        t1 = k;
                        m_pw1 = tmp_m_pw;
                        flag = true;
                    }
                    else
                    {
                        t2 = k;
                        m_pw2 = tmp_m_pw;
                        flag = false;
                        break;
                    }
                }
            }
            if(anticlockwise)
            {
                if(m_pw1[0]>m_pw2[0])
                {
                    armor = t1;
                    right = true;
                }
                else
                {
                    armor = t2;
                    right = false;
                }
            }
            else
            {
                if(m_pw1[0]<m_pw2[0])
                {
                    armor = t1;
                    right = false;
                }
                else
                {
                    armor = t2;
                    right = true;
                }
            }
        }
        else
            selected = false;
    }
    //get the most important one
    if((!selected && last_shoot && last_sbbox.ID != important_kill)||(selected && armor.ID != important_kill && robotsta.enemy.at(armor.ID)*10) <= killer_point)
    {
        for(auto &d: new_detections)
        {
            if(d.ID == important_kill)
            {
                Matrix31d m_pc = PnP_get_pc(d.pts,d.ID);
                Matrix31d m_pw = pc_to_pw(m_pc,R_IW);
                double Distance = m_pw.norm();
                if(Distance > distance_max)
                    break;
                armor = d;
                selected = true;
                same_armor = false;
                same_id = false;
                switch_armor = false;
                need_init = true;
                break; // quit now
            }

        }

    }
    //complete
    if(!selected && last_shoot)
    {
        for(auto &d: new_detections)
        {
            if(d.ID == last_sbbox.ID)
            {
                armor = d;
                selected = true;
                same_armor = false;
                same_id = true;
                switch_armor = true;
                if(DEBUG)
                    cout<<"same id"<<endl;
                break;
            }
        }
    }
    //complete

    if(!selected)
    {
        double max_size = -1;
        for(auto &d:new_detections)
        {
            auto size = get_quadrangle_size(d);
            if(size > max_size)
            {
                armor = d;
                max_size = size;
            }
            if(robotsta.enemy.at(d.ID)*10 <= killer_point)
            {
                armor = d;
                break;
            }
        }
        selected = true;
        same_armor = false;
        same_id = false;
        switch_armor = false;
        need_init = true;
    }

    //record
    unsigned int i = 0;

    for(auto &d: detections)
    {
        if(armor.ID == d.ID)
            i++;
    }
    if(i == 1)
    {
        this_is_one_armor = true;
        this_is_two_armors = false;
    }
    else if(i == 2)
    {
        this_is_one_armor = false;
        this_is_two_armors = true;
    }
    else
    {
        this_is_one_armor = false;
        this_is_two_armors = false;
        cout<<"[WARNING] multiple armors detected !!"<<endl;
    }
    if(DEBUG)
        cout<<"last_is_one_armor = "<<last_is_one_armor<<", last_is_two_armors = "<< last_is_two_armors<<endl;

    if(this_is_two_armors)
    {
        Matrix31d armor_m_pc = PnP_get_pc(armor.pts,armor.ID);
        Matrix31d armor_m_pw = pc_to_pw(armor_m_pc,R_IW);
        for(auto &d: new_detections)
        {
            if(d.ID == armor.ID&& d != armor)
            {
                Matrix31d tmp_m_pc = PnP_get_pc(d.pts, d.ID);
                Matrix31d tmp_m_pw = pc_to_pw(tmp_m_pc,R_IW);
                if(armor_m_pw(0,0) > tmp_m_pw(0,0))
                    right = true;
                else
                    right = false;
                break;
            }
        }
    }

    for(auto &a: antitop_candidates)
    {
        a.updated = false;
        a.init = false;
    }

    if(same_id && !antitop)
    {
        long unsigned int i = 0;
        for(int i = 0;i < antitop_candidates.size();i++)
        {
            auto center = points_center(armor.pts);
            if(center.inside(get_ROI(antitop_candidates[i].bbox))||is_same_armor_by_distance(antitop_candidates[i].last_m_pw,armor,R_IW))
            {
                ekf = antitop_candidates[i].ekf;
                //load_param(false);
                last_m_pw = antitop_candidates[i].last_m_pw;
                last_pitch = antitop_candidates[i].last_pitch;
                last_yaw = antitop_candidates[i].last_yaw;
                if(DEBUG)
                    cout<<"choose candidates"<<endl;
                if(this_is_two_armors)
                    right = !right;
                break;
            }
        }
        if(i < antitop_candidates.size())
        {
            antitop_candidates.erase(antitop_candidates.begin()+i);
            same_armor = true;
            need_init = false;
        }
        else
        {
            need_init = true;
        }
    }
    //complete

    if(same_armor)
    {
        Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
        Matrix31d m_pw = pc_to_pw(m_pc,R_IW);

        //camera and world(如果在这个位置有错误的话，把2换成1(那一部分上交写的很迷))
        float mc_yaw = (float)getTheta(m_pc(0,0),m_pc(2,0));
        float m_yaw = (float)getTheta(m_pw(0,0),m_pw(2,0));
        float mc_pitch = (float)getTheta(getDis(m_pc(0,0),m_pc(2,0)),m_pc(1,0));
        float m_pitch = (float)getTheta(getDis(m_pw(0,0),m_pw(2,0)),m_pw(1,0));

        last_m_pw = m_pw;
        last_yaw = m_yaw;
        last_pitch = m_pitch;

        Matrix31d e_pw = m_pw;
        //ekf work

        PredictFunc predictfunc;
        Measure measure;

        Matrix51d Xr;
        Xr << e_pw(0,0),0,e_pw(1,0),0,e_pw(2,0);
        Matrix31d Yr;
        measure(Xr.data(),Yr.data());
        predictfunc.delta_t = delta_t;
        ekf.predict(predictfunc);
        Matrix51d Xe = ekf.update(measure,Yr);

        double predict_time = e_pw.norm()/getdata.robot_speed + shoot_delay;
        predictfunc.delta_t = predict_time;
        Matrix51d Xp;
        predictfunc(Xe.data(),Xp.data());

        e_pw<<Xp(0,0),Xp(2,0),Xp(4,0);


        float Distance;
        makeAngle(e_pw,m_yaw,m_pitch,Distance);

        Trajectory trajectory;

        Matrix31d s_pw = e_pw;
        float speed = getdata.robot_speed;
        trajectory.trajectory_predict(s_pw,m_pitch,speed);

        Matrix31d s_pc = pw_to_pc(s_pw,R_IW);
        double s_yaw = getTheta(s_pc(0,0),s_pc(2,0)) / PI *180;
        double s_pitch = getTheta(s_pc(1,0),s_pc(2,0))/PI*180;

        send.pitch_angle = s_pitch;
        send.yaw_angle = s_yaw;
        send.distance = Distance;

        if(isshow)
        {
            re_project_point(im_show,m_pw,R_IW,{0,255,0});
            re_project_point(im_show,e_pw,R_IW,{255,0,0});
            re_project_point(im_show,s_pw,R_IW,{0,0,255});
            for(int i=0;i<4;i++)
            {
                cv::circle(im_show,armor.pts[i],3,{255,255,0});
            }
            cv::circle(im_show,{im_show.cols/2,im_show.rows/2},3,{0,255,255});
        }

    }
    if(need_init)
    {
        if(DEBUG)
            cout<<"need init"<<endl;
        Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
        Matrix31d m_pw = pc_to_pw(m_pc,R_IW);

        //camera and world
        float mc_yaw = (float)getTheta(m_pc(0,0),m_pc(2,0));
        float m_yaw = (float)getTheta(m_pw(0,0),m_pw(2,0));
        float mc_pitch = (float)getTheta(getDis(m_pc(0,0),m_pc(2,0)),m_pc(1,0));
        float m_pitch = (float)getTheta(getDis(m_pw(0,0),m_pw(2,0)),m_pw(1,0));

        Matrix31d e_pw = m_pw;
        //ekf init + ekf work

        PredictFunc predictfunc;
        Measure measure;

        Matrix51d Xr;
        Xr << e_pw(0,0),0,e_pw(1,0),0,e_pw(2,0);

        ekf.init(Xr);
        last_m_pw = m_pw;
        last_yaw = m_yaw;
        last_pitch = m_pitch;

        float Distance;
        makeAngle(e_pw,m_yaw,m_pitch,Distance);

        Trajectory trajectory;

        Matrix31d s_pw = e_pw;
        float speed = getdata.robot_speed;
        trajectory.trajectory_predict(s_pw,m_pitch,speed);

        Matrix31d s_pc = pw_to_pc(s_pw,R_IW);
        double s_yaw = getTheta(s_pc(0,0),s_pc(2,0)) /PI*180;
        double s_pitch = getTheta(s_pc(1,0),s_pc(2,0))/PI*180;

        send.pitch_angle = s_pitch;
        send.yaw_angle = s_yaw;
        send.distance = Distance;

        if(isshow)
        {
            re_project_point(im_show,m_pw,R_IW,{0,255,0});
            re_project_point(im_show,e_pw,R_IW,{255,0,0});
            re_project_point(im_show,s_pw,R_IW,{0,0,255});
            for(int i=0;i<4;i++)
            {
                cv::circle(im_show,armor.pts[i],3,{255,255,0});
            }
            cv::circle(im_show,{im_show.cols/2,im_show.rows/2},3,{0,255,255});
        }
    }

    if(DEBUG)
        cout<<"update vector"<<endl;
    if(need_init) antitop_candidates.clear();

    for(auto &d:new_detections)
    {
        if(DEBUG)
            cout<<"=======tag ID ="<<d.ID<<" ============"<<endl;
        if(d.ID!=armor.ID||d == armor)
            continue;
        bool exist = false;
        auto center = points_center(d.pts);
        for(auto &ac: antitop_candidates)
        {
            if(ac.updated)
                continue;
            bool flag = false;
            bbox_t bx = d;
            if(center.inside(get_ROI(ac.bbox))||is_same_armor_by_distance(ac.last_m_pw,d,R_IW))
            {
                flag = true;
            }
            else if(last_is_two_armors && this_is_two_armors)
            {
                match_armors(bx,flag,new_detections,R_IW,!right);
            }
            if(flag)
            {
                exist = true;
                if(DEBUG)
                    cout<<"updated!"<<endl;
                ac.bbox = bx;
                ac.updated = true;

                ++ac.age;
                if(ac.age <= ac_init_min_age)
                {
                    ac.init = true;
                    if(DEBUG)
                        cout<<"need init"<<endl;
                    Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
                    Matrix31d m_pw = pc_to_pw(m_pc,R_IW);

                    //camera and world
                    float mc_yaw = (float)getTheta(m_pc(0,0),m_pc(2,0));
                    float m_yaw = (float)getTheta(m_pw(0,0),m_pw(2,0));
                    float mc_pitch = (float)getTheta(getDis(m_pc(0,0),m_pc(2,0)),m_pc(1,0));
                    float m_pitch = (float)getTheta(getDis(m_pw(0,0),m_pw(2,0)),m_pw(1,0));

                    last_m_pw = m_pw;
                    last_yaw = m_yaw;
                    last_pitch = m_pitch;

                    Matrix31d e_pw = m_pw;

                    //ekf init second
                    Matrix51d Xr;
                    Xr<<e_pw(0,0),-ac_x_v_coefficient * ekf.Xe(1,0),e_pw(1,0),-ac_x_v_coefficient * ekf.Xe(3,0), e_pw(2,0);
                    ac.ekf.init(Xr);
                    ac.last_m_pw = e_pw;
                    ac.last_yaw = m_yaw;
                    ac.last_pitch = m_pitch;
                }
                else
                {
                    ac.init = false;
                    Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
                    Matrix31d m_pw = pc_to_pw(m_pc,R_IW);

                    //camera and world
                    float mc_yaw = (float)getTheta(m_pc(0,0),m_pc(2,0));
                    float m_yaw = (float)getTheta(m_pw(0,0),m_pw(2,0));
                    float mc_pitch = (float)getTheta(getDis(m_pc(0,0),m_pc(2,0)),m_pc(1,0));
                    float m_pitch = (float)getTheta(getDis(m_pw(0,0),m_pw(2,0)),m_pw(1,0));

                    last_m_pw = m_pw;
                    last_yaw = m_yaw;
                    last_pitch = m_pitch;

                    Matrix31d e_pw = m_pw;
                    //ekf work
                    PredictFunc predictfunc;
                    Measure measure;
                    predictfunc.delta_t = delta_t;
                    ac.ekf.predict(predictfunc);

                    Matrix51d Xr;
                    Xr << e_pw(0,0),0,e_pw(2,0),0,e_pw(4,0);
                    Matrix31d Yr;
                    measure(Xr.data(),Yr.data());
                    Matrix51d Xe = ac.ekf.update(measure,Yr);
                    double predict_time = e_pw.norm() / getdata.robot_speed + shoot_delay;

                    predictfunc.delta_t = predict_time;
                    Matrix51d Xp;
                    predictfunc(Xe.data(),Xp.data());

                    e_pw<<Xp(0,0),0,Xp(2,0),0,Xp(4,0);

                    float Distance;
                    makeAngle(e_pw,m_yaw,m_pitch,Distance);

                    Trajectory trajectory;

                    Matrix31d s_pw = e_pw;
                    float speed = getdata.robot_speed;
                    trajectory.trajectory_predict(s_pw,m_pitch,speed);

                    Matrix31d s_pc = pw_to_pc(s_pw,R_IW);
                    double s_yaw = getTheta(s_pc(0,0),s_pc(2,0)) / PI *180;
                    double s_pitch = getTheta(s_pc(1,0),s_pc(2,0))/PI*180;

                    if(isshow)
                    {
                        re_project_point(im_show,m_pw,R_IW,{0,255,0});
                        re_project_point(im_show,e_pw,R_IW,{255,0,0});
                        re_project_point(im_show,s_pw,R_IW,{0,0,255});
                        for(int i=0;i<4;i++)
                        {
                            cv::circle(im_show,armor.pts[i],3,{255,255,0});
                        }
                        cv::circle(im_show,{im_show.cols/2,im_show.rows/2},3,{0,255,255});
                    }

                    ac.distance = (int) (Distance * 10);
                    ac.yaw_angle = (float) s_yaw;
                    ac.pitch_angle = (float) s_pitch;
                }
                break;
            }
        }
        if(!exist)
        {
            cout<<"no suitable,try add new armor"<<endl;
            PredictRecord r(d);
            Matrix31d m_pc = PnP_get_pc(armor.pts,armor.ID);
            Matrix31d m_pw = pc_to_pw(m_pc,R_IW);
            float mc_yaw = (float)getTheta(m_pc(0,0),m_pc(2,0));
            float m_yaw = (float)getTheta(m_pw(0,0),m_pw(2,0));
            float mc_pitch = (float)getTheta(getDis(m_pc(0,0),m_pc(2,0)),m_pc(1,0));
            float m_pitch = (float)getTheta(getDis(m_pw(0,0),m_pw(2,0)),m_pw(1,0));

            Matrix31d e_pw = m_pw;
            Matrix51d Xr;
            Xr << e_pw(0,0),0.5*ekf.Xe(1,0),e_pw(1,0),0.25*ekf.Xe(3,0),e_pw(2,0);
            r.ekf.init(Xr);

            r.last_m_pw = m_pw;
            r.last_yaw = m_yaw;
            r.last_pitch = m_pitch;

            r.updated = true;
            r.init = true;
            antitop_candidates.push_back(r);
        }
    }
    //complete
    int len = antitop_candidates.size();
    for(long unsigned int i=0 ;i<antitop_candidates.size();i++)
    {
        if(!antitop_candidates[i].updated)
        {
            antitop_candidates.erase(antitop_candidates.begin() + i);
            --i;
            --len;
        }
    }

    double size1 = get_quadrangle_size(armor);
    for(long unsigned int i=0 ;i < antitop_candidates.size();i++)
    {
        if(antitop_candidates[i].init)
            continue;
        double size2 = get_quadrangle_size(antitop_candidates[i].bbox);
        if(size2 > switch_armor_size_proportion * size1)
        {
            if(DEBUG)
                cout<<"Switch armor due to size scale"<<endl;
            switch_armor = true;
            PredictRecord r(armor);
            r.ekf = ekf;
            r.updated = true;
            r.init = need_init;
            r.last_m_pw = last_m_pw;
            r.last_yaw = last_yaw;
            r.last_pitch = last_pitch;
            r.yaw_angle = send.yaw_angle;
            r.pitch_angle = send.pitch_angle;
            r.distance = send.distance;
            r.distant = distance_check;
            antitop_candidates.push_back(r);

            armor = antitop_candidates[i].bbox;

            ekf = antitop_candidates[i].ekf;
            last_m_pw = antitop_candidates[i].last_m_pw;
            last_yaw = antitop_candidates[i].last_yaw;
            last_pitch = antitop_candidates[i].last_pitch;
            send.pitch_angle = antitop_candidates[i].pitch_angle;
            send.yaw_angle = antitop_candidates[i].yaw_angle;
            send.distance=r.distance;
            distance_check = antitop_candidates[i].distant;
            antitop_candidates.erase(antitop_candidates.begin()+i);

            if (this_is_two_armors)
                right = !right;
            if (send.yaw_angle > 0)
                anticlockwise = false;
            else
                anticlockwise = true;
            break;
        }
    }

    //strength the armor
    if(isshow)
    {
        for(int i=0;i<4;i++)
        {
            cv::circle(im_show,armor.pts[i],3,{0,255,0},2);
        }

    }

    //list the history
    last_is_one_armor = this_is_one_armor;
    last_is_two_armors = this_is_two_armors;

    last_boxes = std::move(new_detections);
    last_sbbox = armor;
    last_shoot = true;

    //send the command signal
    send.shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
    if(distance_check) send.shoot_mode += static_cast<uint8_t>(ShootMode::DISTANT);
    if(antitop) send.shoot_mode += static_cast<uint8_t>(ShootMode::ANTITOP);
    if(switch_armor) send.shoot_mode += static_cast<uint8_t>(ShootMode::SWITCH);
    if(!(std::abs(send.yaw_angle)<90.)&&std::abs(send.pitch_angle)<90.)
        send.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
    if(exist_hero)
        send.shoot_mode += static_cast<uint8_t>(ShootMode::EXIST_HERO);

    send.target_id = armor.ID;
    send.priority = Priority::DANGER;

    return true;
    //check complete
}




void KalmenWorking::match_armors(bbox_t &armor,bool &selected,const std::vector<bbox_t> &detections,const Matrix33d &R_IW,const bool right)
{
    if(!last_shoot)
    {
        selected = false;
        return;
    }
    std::vector<bbox_t> temp_armors;
    for(auto &d:detections)
    {
        bbox_t tmp = d;
        if(d.ID == last_sbbox.ID)
            temp_armors.push_back(tmp);
    }
    if(temp_armors.size() == 2)
    {
        selected = false;
        Matrix31d m_pw1,m_pw2;
        bool flag = false;
        for(auto &t: temp_armors)
        {
            if(t.ID == last_sbbox.ID)
            {
                Matrix31d tmp_m_pc = PnP_get_pc(t.pts,t.ID);
                Matrix31d tmp_m_pw = pc_to_pw(tmp_m_pc,R_IW);
                if(!flag)
                {
                    m_pw1 = tmp_m_pw;
                    flag = true;
                }
                else
                {
                    m_pw2 = tmp_m_pw;
                    flag = false;
                }
            }
        }
        if(flag)
            cout<<"[ERROR] in match_armors"<<endl;
        if(right)
        {
            if(m_pw1(0,0) > m_pw2(0,0))
                armor = temp_armors.front();
            else
                armor = temp_armors.back();
        }
        selected = true;
    }
    else
    {
        selected = false;
        return;
    }
    //complete
}

float KalmenWorking::bbOverlap(const cv::Rect2f &box1, const cv::Rect2f &box2)
{
    if(box1.x > box2.x+box2.width) {return 0.0;}
    if(box1.y>box2.y+box2.height){return 0.0;}
    if(box1.x+box1.width<box2.x){return 0.0;}
    if(box1.y+box1.height< box2.y) {return 0.0;}

    float colInt = std::min(box1.x + box1.width,box2.x+box2.width) - std::max(box1.x,box2.x);
    float rowInt = std::min(box1.y + box1.height,box2.y+box2.height) -std::max(box1.y,box2.y);
    float intersection = colInt * rowInt;
    float area1 = box1.width * box1.height;
    float area2 = box2.width *box2.height;
    return intersection/(area1+area2-intersection);
}

double KalmenWorking::get_quadrangle_size(const bbox_t &bx)
{
    auto K1 = bx.pts[0].x*bx.pts[1].y - bx.pts[1].x*bx.pts[0].y;
    auto K2 = bx.pts[1].x*bx.pts[2].y - bx.pts[2].x*bx.pts[1].y;
    auto K3 = bx.pts[2].x*bx.pts[3].y - bx.pts[3].x*bx.pts[2].y;
    auto K4 = bx.pts[3].x*bx.pts[0].y - bx.pts[0].x*bx.pts[3].y;
    auto S = 0.5 *fabs(K1+K2+K3+K4);
    return S;
}

Eigen::Vector3d KalmenWorking::PnP_get_pc(const cv::Point2f p[4],int armor_number)
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


/*
void KalmenWorking::make3Ddata()
{
    place3d[0] << 0, 0, 0;
    for (int i = 0; i < MAXN; i++)
    {
        if (i == 0)
        {
            continue;
        }
        if (i < 50)
        {
            double T = t * i;
            this->theta = PI * 0.25;
            this->a = 0.1;
            double V = a * t;
            double X = a * t * t * 0.5;
            double Xcos = X * cos(theta);
            double Xsin = X * sin(theta);
            place3d[i] << (Xcos), 0, (Xsin);
            if (i == 49)
            {
                double Vcos = V * cos(theta);
                double Vsin = V * sin(theta);
                this->NowV3d << (Vcos), 0,(Vsin);
            }
            //cout << place3d[i] << endl;
        }
        else if (i >= 50 && i < 200)
        {
            double V1 = NowV3d(0, 0);
            double V2 = NowV3d(2, 0);
            double T = t * (i - 50.0);
            this->a = 1;
            double V = a * t;
            double X = a * t * t * 0.5;
            double x1 = place3d[i-1](0, 0) + V * T*cos(this->theta);
            double x2 = place3d[i-1](2, 0) + V * T* sin(this->theta);
            place3d[i] << (x1), X, (x2);
            if (i == 199)
            {
                double Vcos = this->NowV3d(0, 0);
                double Vsin = this->NowV3d(2, 0);
                this->NowV3d << (Vcos), V, (Vsin);
            }
            //cout << place3d[i] << endl;
            continue;
        }
        else if (i >= 200 && i <= 300)
        {
            this->place3d[i] = this->place3d[i - 1] + NowV3d * t;
            //cout << place3d[i] << endl;
            continue;
        }
        else
        {
            this->place3d[i] = this->place3d[i - 1] + NowV3d * sin(i)*10*cos(i);
            //cout << place3d[i] << endl;
            continue;
        }
    }
}
*/





