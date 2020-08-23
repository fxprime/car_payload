#pragma once


#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <car_payload/msg.hpp>

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


    ulink_header_t   header   = {0};
    ulink_checksum_t checksum = {0};

    // Populate header
    header.header      = HEADER;
    header.msg         = msg;
    header.id.quad_uid = id;
    header.length      = length;
 
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
 
    if (_msgid == PAYLOAD_STATUS) {
        payload_status_s msgIn;
        memcpy(&msgIn, &_pay[0], sizeof(msgIn));
        // {
        //     ROS_INFO("Servo %.2f %.2f %.2f %.2f %.2f", msgIn.servo1, msgIn.servo2, msgIn.servo3, msgIn.servo4, msgIn.servo5);
        // }
    } 
}

static inline void send_servo_cmd(uint16_t quad_id, servos_cmd_s &msg) {
    sendMessage(SERVOS_CMD, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}
static inline void send_allservo_cmd(uint16_t quad_id, payload_status_s &msg) {
    sendMessage(SERVO_ALL_CMD, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}