//
// Created by knight on 22-5-27.
//

#ifndef GUARD_GLOBAL_H
#define GUARD_GLOBAL_H
#include<iostream>
#include<cmath>
#include<vector>
#include"eigen3/Eigen/Eigen"
#include"opencv2/core/eigen.hpp"
#include"opencv2/opencv.hpp"
#include<queue>
#include<string>

#include<cstring>
#include<stdint.h>
#include<cstdio>
#include<random>
#include<ceres/ceres.h>
#include<array>
#include<atomic>
#include<thread>


using namespace std;
using namespace std;

using Matrix66d = Eigen::Matrix<double,6,6>;
using Matrix61d = Eigen::Matrix<double,6,1>;
using Matrix16d = Eigen::Matrix<double,1,6>;
using Matrix11d = Eigen::Matrix<double,1,1>;
using Matrix33d = Eigen::Matrix<double,3,3>;
using Matrix31d = Eigen::Matrix<double,3,1>;
using Matrix13d = Eigen::Matrix<double,1,3>;
using Matrix99d = Eigen::Matrix<double,9,9>;
using Matrix91d = Eigen::Matrix<double,9,1>;
using Matrix19d = Eigen::Matrix<double,1,9>;
using Matrix93d = Eigen::Matrix<double,9,3>;
using Matrix39d = Eigen::Matrix<double,3,9>;
using Matrix21d = Eigen::Matrix<double,2,1>;
using Matrix22d = Eigen::Matrix<double,2,2>;
using Matrix12d = Eigen::Matrix<double,1,2>;
using Matrix22f = Eigen::Matrix<float,2,2>;
using Matrix21f = Eigen::Matrix<float,2,1>;
using Matrix12f = Eigen::Matrix<float,1,2>;
using Matrix11f = Eigen::Matrix<float,1,1>;
using Matrix55d = Eigen::Matrix<double,5,5>;
using Matrix51d = Eigen::Matrix<double,5,1>;
using Matrix15d = Eigen::Matrix<double,1,5>;
using Matrix53d = Eigen::Matrix<double,5,3>;
using Matrix35d = Eigen::Matrix<double,3,5>;

//======================here you can change the const ===========================
//solvePnP所需要的常数，由直接测量长度获取
constexpr double armor_big_l = 0.1175;

constexpr double armor_big_w = 0.0290;

constexpr double armor_small_l = 0.066;

constexpr double armor_small_w = 0.027;

constexpr double wind_l = 0.015;

constexpr double wind_w = 0.0635;

//可以手动调偏移量，如无需调，就置为0
constexpr double guard_pc_x = 0.0;

constexpr double guard_pc_y = 0.0;

constexpr double guard_pc_z = 0.0;

constexpr double sol_pc_x = 0.0;

constexpr double sol_pc_y = 0.0;

constexpr double sol_pc_z = 0.0;
//看起来没有价值的东西
constexpr double t = 0.01;
//对于识别到灰色的部分，如果灰色出现超过dead_buffer_max_size次，就会放行
constexpr int dead_buffer_max_size = 20;
//延迟
constexpr double shoot_delay = 0.01;
//高度阈值
constexpr float height_thres = 2;
//置信度阈值
constexpr float high_thres=0.6;

constexpr float low_thres=0.4;

//优先ID选择，可以根据战术调整
constexpr int important_kill = 1;
//斩杀线阈值1号
constexpr int killer_point = 100;
//斩杀线阈值2号
constexpr int killer_point_for_2 = 80;
//目前没有用上，即反陀螺速度的限制幅度
constexpr double antitop_x_v_proportion = 0.;

constexpr double antitop_y_v_proportion = 0.;
//反陀螺速度继承比例
constexpr float ac_x_v_coefficient = 0.5f;

constexpr float ac_y_v_coefficient = 0.5f;
//远距离控制，大于该距离将会被忽略
constexpr double distance_max = 6;

constexpr float switch_armor_size_proportion = 1.1;

constexpr double ac_init_min_age = 1;
//you can use this to cout something to DEBUG
constexpr bool DEBUG = false;

#define PI 3.141592653589793238

//you can change something in there

enum class OURSELVES
{
    GUARD = 0,
    HERO = 1,
    OTHERS = 2,
};


enum class ColorChoose:uint8_t
{
    RED = 0,
    BLUE = 1,
};

enum class GameState:uint8_t
{
    SHOOT_NEAR_ONLY = 0,
    SHOOT_FAR = 1,
    COMMON = 255,
};

enum class Priority:uint8_t
{
    CORE = 0,
    DANGER = 1,
    NONE = 255,
};

enum class ShootMode : uint8_t
{
    COMMON = 0,
    DISTANT = 1,
    ANTITOP = 2,
    SWITCH = 4,
    FOLLOW = 8,
    CRUISE = 16,
    EXIST_HERO = 32,

};

enum class Vision : uint8_t
{
    CLASSIC = 0,
    SMALL = 1,
    BIG = 2,
};

struct Robotstatus
{
    std::array<double,4> q;
    Vision vision = Vision::CLASSIC;
    GameState gamestate = GameState::COMMON;
    ColorChoose color = ColorChoose::BLUE;
    uint8_t target_id = 255;
    float robot_speed = 30.;
    //blood
    std::array<uint8_t ,6> enemy;
    uint8_t lrc = 0;


}__attribute__((packed));

struct RobotCMD
{
    uint8_t start = (unsigned)'s';

    Priority priority; // uint_8
    uint8_t target_id = 255;
    float pitch_angle = 0;
    float yaw_angle = 0;
    //float pitch_speed = 0;
    //float yaw_speed = 0;
    float distance = 0;
    uint8_t shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
    uint8_t lrc;


    uint8_t end = (unsigned)'e';

}__attribute__((packed));

namespace Calculator
{

}




#endif //GUARD_GLOBAL_H
