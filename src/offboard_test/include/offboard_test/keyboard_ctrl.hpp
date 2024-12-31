
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define TRIGGER_KEYS_NUM 8
#define W       0
#define A       1
#define S       2
#define D       3
#define UP      4
#define DOWN    5
#define LEFT    6
#define RIGHT   7

#define LATCH_KEYS_NUM 4
#define NUM1    0
#define NUM2    1
#define NUM3    2
#define NUM4    3

#define M_PI 3.14159265358979323846

extern struct libevdev* dev;

struct Imu_data{
    float imu_linear_accl_x;
    float imu_linear_accl_y;
    float imu_linear_accl_z;

    float imu_angular_vel_x;
    float imu_angular_vel_y;
    float imu_angular_vel_z;
};

class KeyboardCtrl : public ros::NodeHandle{
public:
    tf2::Quaternion q;
    mavros_msgs::AttitudeTarget att_cmd_msg;  // 要发布的AttitudeTarget消息
    Imu_data imu_data;

    KeyboardCtrl(int queue_size, double publish_rate);
    bool* get_key_input();
    void update_latch_key_status(uint8_t key_index);
    void update_trigger_key_status(uint8_t key_index);
    void printKeyStatus();
    void get_namespace (uint8_t);
    void set_mav_cmd();
    int dir_judge(uint8_t dir1, uint8_t dir2);
    
private:
    ros::Timer timer;                       // 定时器对象
    ros::Publisher mav_cmd_publisher;        // 发布者
    ros::Subscriber imu_data_subscriber;
    std::string robot_name;
    struct input_event ev;

    bool latch_key_status[LATCH_KEYS_NUM];
    bool trigger_key_status[TRIGGER_KEYS_NUM];    
    void imu_data_subscriber_callback(const sensor_msgs::Imu & msg);
    void mav_cmd_callback();
    void timer_callback(const ros::TimerEvent& event);  // 定时器回调函数
};
