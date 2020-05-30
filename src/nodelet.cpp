#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include "car_connect/nodelet.hpp"
#include "car_connect/protocal.hpp"



using namespace std;



void openSerial(const string& sport);

int main(int argc, char *argv[]) {



    //init stuff
    ros::init(argc, argv, "car_connect");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    openSerial("/dev/ttyUSB0");

    _imu_pub          = n.advertise<sensor_msgs::Imu>("car_connect/imu",               1);
    _odom_pub         = n.advertise<nav_msgs::Odometry>("car_connect/odom",            1);
    _vel_cmd_pub      = n.advertise<geometry_msgs::Twist>("car_connect/vel_cmd",       1);

    while(ros::ok()) {

        if (_serial != nullptr)
        {

            if (_serial->isOpened())
            {

                //Read from serial
                if (_serial->BytesAvailable() > 0)
                {
                    uint16_t len = min(_serial->BytesAvailable(), 2048);
                    if (len > 0)
                    {
                        char *buff = (char *)malloc(len);
                        _serial->Read(buff, len);
                        serial_handle(buff, len);
                        delete[] buff;
                    }
                }

            }
        }


        usleep(1);
    }

}
