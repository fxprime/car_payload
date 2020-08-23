#include <std_msgs/UInt8MultiArray.h>



#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include "car_payload/nodelet.hpp"
#include "car_payload/protocal.hpp"



using namespace std;
const float target_quid = 33;



void openSerial(const string& sport);
void cmdCb(const std_msgs::UInt8MultiArray::ConstPtr &msg) 
{
  
    if(msg->data.size() <3) {
        ROS_ERROR("data size mismatch");
        return;
    }

    if(msg->data.size()==3) {
        servos_cmd_s msgOut;
        msgOut.servo_id = msg->data[0];
        msgOut.servo_val = ( toSint(msg->data[1])*100); 
        send_servo_cmd(target_quid, msgOut);
        ROS_INFO("Send servo %d %d", msgOut.servo_id, msgOut.servo_val);
    }else if(msg->data.size()==5) {
        payload_status_s msgOut;
        msgOut.servo1 = msg->data[0];
        msgOut.servo2 = msg->data[1];
        msgOut.servo3 = msg->data[2];
        msgOut.servo4 = msg->data[3];
        msgOut.servo5 = msg->data[4];
        send_allservo_cmd(target_quid, msgOut);
        ROS_INFO("Send all servo = %d %d %d %d %d", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
    }

    
}


int main(int argc, char *argv[]) {



    //init stuff
    ros::init(argc, argv, "car_payload");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    openSerial("/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DM01VISA-if00-port0");
 
    ros::Subscriber sub_payload_cmd = n.subscribe("car_payload/payload_set", 1, &cmdCb);

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
