#pragma once

#include <ros/ros.h>


#include "car_connect/serial.hpp"
#include "car_connect/msg.hpp"




serial* _serial;
ros::Publisher _imu_pub;
ros::Publisher _odom_pub;
ros::Publisher _vel_cmd_pub;





typedef struct {
    sensor_status_s sensor;
    system_status_s system;
    rc_status_s     rc;
    cnt_status_s    cnt;
}car_state;


car_state _state;





void openSerial(const string& sport) {
    if(sport == "") return;

    if(_serial != nullptr) {
        delete _serial;
        _serial = nullptr;
    }


    string gg = sport;
    _serial = new serial(gg );
    if(_serial->isOpened()) {
        _serial->setup_port(115200, 8, 1, false, false);
        ROS_INFO("Connecting to serial port %s", sport.c_str());
    }else{
        ROS_ERROR("Cannot connect to %s", sport.c_str());
        delete _serial;
    }

}