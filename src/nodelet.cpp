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
const float target_quid = 33;



void openSerial(const string& sport);
void cmdCb(const geometry_msgs::Twist::ConstPtr &msg) 
{
    
    if( std::isnan(msg->linear.x) ||  std::isnan(msg->linear.y) || std::isnan(msg->linear.z))       return;
    if( std::isnan(msg->angular.x) ||  std::isnan(msg->angular.y) || std::isnan(msg->angular.z))    return;
    cnt_status_s pmsg;
    pmsg.vel_cnt.vx = msg->linear.x*100;
    pmsg.vel_cnt.vy = -msg->linear.y*100;
    pmsg.vel_cnt.wz = msg->angular.z*100;
    send_vel_cmd_status(target_quid, pmsg);
    ROS_INFO("Received cmd_vel %.2f %.2f %.2f", msg->linear.x, msg->linear.y, msg->angular.z);
}


int main(int argc, char *argv[]) {



    //init stuff
    ros::init(argc, argv, "car_connect");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    openSerial("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");

    _imu_pub          = n.advertise<sensor_msgs     ::Imu>      ("car_connect/imu",     1);
    _odom_pub         = n.advertise<nav_msgs        ::Odometry> ("car_connect/odom",    1);
    _vel_cmd_pub      = n.advertise<geometry_msgs   ::Twist>    ("car_connect/vel_cmd", 1);
    ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1, &cmdCb);

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
        ros::spinOnce();


        usleep(1);
    }

}
