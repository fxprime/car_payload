#pragma once


#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <car_connect/msg.hpp>

#define toUint32(X)    *(uint32_t*)(&X)
#define toUint8(X)    *(uint8_t*)(&X)
#define toSint8(X)    *(int8_t*)(&X)
#define toUint(X)    *(uint16_t*)(&X)
#define toSint(X)    *(int16_t*)(&X)
#define toFlt(X)    *(float*)(&X)
#define toLong(X)    *(int32_t*)(&X)
#define wrap_pi(x) (x < -M_PI ? x+2*M_PI : (x > M_PI ? x - 2*M_PI: x))


uint8_t st = 0, _msgid, _pay[2048], _c[2],_lenp[2], _uid[2];
uint16_t _lenpsum,_idp, _sum_pay, _csum;
uint16_t _quad_uid = 0;


inline void packet_decode(uint16_t quad_id);

inline void serial_handle(char *buf, uint16_t len)
{

    uint8_t msgReceived = false;

    for (int i =0;i<len;i++)
    {

        // Check if a message could be decoded, return the message in case yes
        unsigned char data = buf[i];

        switch (st) {
            case 0:
                _sum_pay = 0;
                if (data == HEADER) {
                    _sum_pay += data;
                    st++;
                }
                break;


                //Collect payload length(2bytes)
            case 1 :
                _lenp[0] = data;
                _sum_pay += data;
                st++;
                break;
            case 2 :
                _lenp[1] = data;
                _lenpsum = toUint(_lenp);
                _sum_pay += data;
                st++;
                break;

                //collect msgid 1byte
            case 3 :
                _msgid = data;
                _sum_pay += data;
                st++;
                break;

                //collect uid 2 bytes
            case 4 :
                _sum_pay += data;
                _uid[0] = data;
                st++;
                break;
            case 5 :
                _sum_pay += data;
                _uid[1] = data;
                _quad_uid = toUint(_uid);
                _idp = 0;
                st++;
                break;


                //collect payload
            case 6 :
                _pay[_idp++] = data;
                _sum_pay += data;
                if (_idp >= _lenpsum) st++;
                break;

                //collect checksum
            case 7 :
                _c[0] = data;
                st++;
                break;
            case 8 :
                _c[1] = data;
                _csum = toUint(_c);
                st = 0;

                // Serial.printf("_msgid %d _lenp %d vs %d quid %d chksum %d %d", _msgid, _lenpsum, _idp, _quad_uid, _sum_pay, _csum);
                if (_sum_pay == _csum) {

                    packet_decode(_quad_uid);
                    msgReceived = true;
                }
                break;
        }
    }
}



void
calcChecksum(const uint8_t *buffer, const uint16_t length, ulink_checksum_t *checksum)
{
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck = checksum->ck + buffer[i];
    }
}


bool
sendMessage(const uint8_t msg, const uint16_t id, const uint8_t *payload, const uint16_t length)
{ 


    ulink_header_t   header = {0};
    ulink_checksum_t checksum = {0};

    // Populate header
    header.header = HEADER;
    header.msg  = msg;
    header.id.quad_uid = id;
    header.length = length;
 
    // Calculate checksum
    calcChecksum(((uint8_t *)&header) , sizeof(header), &checksum);

    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
    }
    
    
    if (_serial->Write((char *)&header, sizeof(header)) != sizeof(header)) {
        // Serial.println("Write failed closing port..");
        usleep(1000000);

        // Serial.println("Done..Exit");
        usleep(1000000);
        exit(0);
        return false;
    }


    if (payload && _serial->Write((char *)payload, length) != length) {
        // Serial.println("write payload failed");
        return false;
    }

    if (_serial->Write((char *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        // Serial.println("write checksum failed");
        return false;
    }
    

    return true;
}













/* -------------------------------------------------------------------------- */
/*                     Decode the serial comming from car                     */
/* -------------------------------------------------------------------------- */

inline void packet_decode(uint16_t quad_uid )
{

 
    if (_msgid == ACK) {  
    }

    else if (_msgid == SENSOR_STATUS) { 
        // ROS_INFO("Received SENSOR_STATUS");
        memcpy(&_state.sensor, &_pay[0], sizeof(_state.sensor));

        {
            sensor_msgs::Imu msg;
            msg.header.frame_id = "imu";
            msg.header.stamp = ros::Time::now();
            msg.angular_velocity.x = _state.sensor.imu.imu_gyro[0]/100.0;
            msg.angular_velocity.y = _state.sensor.imu.imu_gyro[1]/100.0;
            msg.angular_velocity.z = _state.sensor.imu.imu_gyro[2]/100.0;
            msg.linear_acceleration.x = -_state.sensor.imu.imu_accel[0]/100.0;
            msg.linear_acceleration.y = -_state.sensor.imu.imu_accel[1]/100.0;
            msg.linear_acceleration.z = -_state.sensor.imu.imu_accel[2]/100.0;
            _imu_pub.publish(msg);
        }



        {

            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "base_link";
            msg.twist.twist.linear.x = _state.sensor.est_speed.vx/100.0;
            msg.twist.twist.linear.y = _state.sensor.est_speed.vy/100.0;
            msg.twist.twist.linear.z = 0;
            msg.twist.twist.angular.x = 0;
            msg.twist.twist.angular.y = 0;
            msg.twist.twist.angular.z = _state.sensor.est_speed.wz/100.0;
            msg.pose.pose.position.x = _state.sensor.est_pos.x/1000.0;
            msg.pose.pose.position.y = _state.sensor.est_pos.y/1000.0;
            msg.pose.pose.position.z = 0;

            /* -------------------------------------------------------------------------- */
            /*             convert yaw to quaternion eigen and forward to msg             */
            /* -------------------------------------------------------------------------- */
            float yaw = wrap_pi(_state.sensor.est_pos.thz/100.0/*+M_PI*/);
            Eigen::Quaterniond yawq(cos(yaw / 2), 0, 0, sin(yaw / 2));
            tf::quaternionEigenToMsg(yawq,msg.pose.pose.orientation );
            
            _odom_pub.publish(msg);



            /* -------------------------------------------------------------------------- */
            /*                          Send odom at 20hz for now                         */
            /* -------------------------------------------------------------------------- */
            // static ros::Time last_send_tf = ros::Time::now();
            // if(ros::Time::now()-last_send_tf > ros::Duration(0.05)) {
            //     last_send_tf = ros::Time::now();
                static tf::TransformBroadcaster odom_br;
                tf::Transform transform;
                tf::Quaternion q;
                q.setRPY(0,0,yaw);
                transform.setOrigin(tf::Vector3(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z));
                transform.setRotation(q);

                odom_br.sendTransform(
                    tf::StampedTransform(
                        transform, ros::Time::now(), "odom_wheel", "car"));
            // }
            
      

        }
        

        // ROS_INFO("%.2f,%.2f,%.2f", _state.sensor.imu.imu_gyro[0]/100.0, _state.sensor.imu.imu_gyro[1]/100.0, _state.sensor.imu.imu_gyro[2]/100.0);

    }

    else if (_msgid == SYSTEM_STATUS) {
        ROS_INFO("Received SYSTEM_STATUS");
    }

    else if (_msgid == RC_STATUS) {
        // ROS_INFO("Received RC_STATUS");
        memcpy(&_state.rc, &_pay[0], sizeof(_state.rc));

    }
    else if (_msgid == VEL_CNT) {
        memcpy(&_state.cnt, &_pay[0], sizeof(_state.cnt));
        {
            geometry_msgs::Twist msg;
            memset(&msg, 0, sizeof(msg));
            msg.linear.x = _state.cnt.vel_cnt.vx/100.0;
            msg.linear.y = _state.cnt.vel_cnt.vy/100.0;
            msg.angular.z = _state.cnt.vel_cnt.wz/100.0;
            _vel_cmd_pub.publish(msg);
        }
    }
    else if (_msgid == TEXT_OUT) {
        text_out_s msgOut;
        memcpy(&msgOut, &_pay[0], sizeof(text_out_s));
        ROS_INFO("CAR -> %s", msgOut.text);
    }
}

static inline void send_vel_cmd_status(uint16_t quad_id, cnt_status_s &msg) {
    sendMessage(CMD_NAV_VEL, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}