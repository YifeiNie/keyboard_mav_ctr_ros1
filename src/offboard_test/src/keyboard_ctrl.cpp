#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Vector3.h>
#include <libevdev/libevdev.h>

#include "keyboard_ctrl.hpp"

struct libevdev* dev = nullptr;

// 禁用终端回显
void disableEcho() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~ECHO;  // 关闭回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// 恢复回显
void enableEcho() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag |= ECHO;  // 恢复回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

// 简单的打印
void KeyboardCtrl::printKeyStatus() {
    std::cout << "NUM_1: " << (latch_key_status[NUM1] ? "1" : "0") << ", ";
    std::cout << "NUM_2: " << (latch_key_status[NUM2] ? "1" : "0") << ", ";
    std::cout << "NUM_3: " << (latch_key_status[NUM3] ? "1" : "0") << ", ";
    std::cout << "NUM_4: " << (latch_key_status[NUM4] ? "1" : "0") << ", ";
    std::cout << "W: " << (trigger_key_status[W] ? "1" : "0") << ", ";
    std::cout << "A: " << (trigger_key_status[A] ? "1" : "0") << ", ";
    std::cout << "S: " << (trigger_key_status[S] ? "1" : "0") << ", ";
    std::cout << "D: " << (trigger_key_status[D] ? "1" : "0") << ", ";
    std::cout << "Up: " << (trigger_key_status[UP] ? "1" : "0") << ", ";
    std::cout << "Down: " << (trigger_key_status[DOWN] ? "1" : "0") << ", ";
    std::cout << "Left: " << (trigger_key_status[LEFT] ? "1" : "0") << ", ";
    std::cout << "Right: " << (trigger_key_status[RIGHT] ? "1" : "0") << std::endl;
    usleep(1000);
}

// 节点类的构造函数
KeyboardCtrl::KeyboardCtrl(int queue_size, double publish_rate){

    robot_name = {};
    mav_cmd_publisher = advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    // 如果采用继承ros::NodeHandle的方法订阅，需要添加this关键字如下
    imu_data_subscriber = this->subscribe("/mavros/imu/data_raw", 1000, &KeyboardCtrl::imu_data_subscriber_callback, this); 
    ROS_INFO("mavros initialized");

    timer = createTimer(ros::Duration(1.0 / publish_rate), &KeyboardCtrl::timer_callback, this);
    // 填充
    std::fill(latch_key_status, latch_key_status + LATCH_KEYS_NUM, false);
    std::fill(trigger_key_status, trigger_key_status + TRIGGER_KEYS_NUM, false);
}

void KeyboardCtrl::mav_cmd_callback(){
    set_mav_cmd();
    mav_cmd_publisher.publish(att_msg);
    // ROS_INFO("AttitudeTarget published!");
}

void KeyboardCtrl::imu_data_subscriber_callback(const sensor_msgs::Imu & msg){
    imu_data.imu_angular_vel_x = msg.angular_velocity.x;
    imu_data.imu_angular_vel_y = msg.angular_velocity.y;
    imu_data.imu_angular_vel_x = msg.angular_velocity.z;

    imu_data.imu_linear_accl_x = msg.linear_acceleration.x;
    imu_data.imu_linear_accl_y = msg.linear_acceleration.y;
    imu_data.imu_linear_accl_z = msg.linear_acceleration.z;
}

int KeyboardCtrl::dir_judge(uint8_t dir1, uint8_t dir2){
    if (trigger_key_status[dir1] == true && trigger_key_status[dir2] == false){
        return 1;
    }
    else if (trigger_key_status[dir1] == false && trigger_key_status[dir2] == true){
        return -1;
    }
    return 0;
}

void KeyboardCtrl::set_mav_cmd(){
    att_msg.thrust = 0.5;
    att_msg.type_mask = 7;      // 这里的body_rate表示角速度还是角度，取决于type_mask的值，如果为7表示角度
    att_msg.body_rate.x = 5*dir_judge(LEFT, RIGHT);    // 单位度每秒或度
    att_msg.body_rate.y = 5*dir_judge(UP, DOWN); 
    att_msg.body_rate.z = 5*dir_judge(A, D);
    att_msg.body_rate.z = 21;
    //mav_cmd_publisher.publish(att_msg);
}

// 更新键盘输入状态
void KeyboardCtrl::update_trigger_key_status(uint8_t key_index){
    if (ev.value == 1) {         // 按下
        trigger_key_status[key_index] = true;
    } else if (ev.value == 0) {  // 松开
        trigger_key_status[key_index]= false;
    } else if (ev.value == 2) {  // 按住
        // 持续按住不改变状态
    }
    printKeyStatus();
}

void KeyboardCtrl::update_latch_key_status(uint8_t key_index){
    if (ev.value == 1){
        for (uint8_t n = 0; n < LATCH_KEYS_NUM ; n++){
            if (n == key_index){
                latch_key_status[n] = !latch_key_status[n];
            }
            else{
                latch_key_status[n] = false;
            }
        }
        get_namespace(key_index);
    }
    printKeyStatus();
    
}

// get函数
bool* KeyboardCtrl::get_key_input(){
    return trigger_key_status;
}

void KeyboardCtrl::get_namespace (uint8_t num){
    std::string num_str = std::to_string(num + 1);
    robot_name = "/robot_" + num_str + "/cmd_vel";
}

// 检查按键事件，并更新相应的标志位
void KeyboardCtrl::timer_callback(const ros::TimerEvent& event){
    if (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) == 0) {
        if (ev.type == EV_KEY) {
            switch (ev.code) {
                case KEY_W:
                    update_trigger_key_status(W);
                    break;
                case KEY_A:
                    update_trigger_key_status(A);
                    break;
                case KEY_S:
                    update_trigger_key_status(S);
                    break;
                case KEY_D:
                    update_trigger_key_status(D);
                    break;
                case KEY_UP:
                    update_trigger_key_status(UP);
                    break;
                case KEY_DOWN:
                    update_trigger_key_status(DOWN);
                    break;
                case KEY_LEFT:
                    update_trigger_key_status(LEFT);
                    break;
                case KEY_RIGHT:
                    update_trigger_key_status(RIGHT);
                    break;
                case KEY_1:
                    update_latch_key_status(NUM1);
                    break;
                case KEY_2:
                    update_latch_key_status(NUM2);
                    break;
                case KEY_3:
                    update_latch_key_status(NUM3);
                    break;
                case KEY_4:
                    update_latch_key_status(NUM4);
                    break;
                default:
                    break;
            }
        }      
    }
    mav_cmd_callback();
    // printKeyStatus();
}

int main(int argc, char** argv) {
    //disableEcho();
    const char* dev_path = "/dev/input/event3";  // 输入设备路径，根据实际情况修改
    int fd = open(dev_path, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        perror("Open device error");
        return 0;
    }

    if (libevdev_new_from_fd(fd, &dev) != 0) {
        perror("Libevdev init error");
        close(fd);
        return 0;
    }
    // 初始化 ROS 节点
    ros::init(argc, argv, "keyboard_cmd_publisher");

    // 创建 CmdVelPublisher 类的实例
    KeyboardCtrl cmd_vel_pub = KeyboardCtrl(10, 100.0); // 发布频率为 10 Hz
    // ROS 主循环
    ros::spin();
    enableEcho();
    return 0;
}