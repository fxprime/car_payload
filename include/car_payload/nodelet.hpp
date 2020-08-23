#pragma once

#include <ros/ros.h>


#include "car_payload/serial.hpp"
#include "car_payload/msg.hpp"




serial* _serial;
ros::Publisher _payload_pub; 
 



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