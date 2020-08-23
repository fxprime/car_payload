#ifndef CAR_PAYLOAD_MSG_HPP
#define CAR_PAYLOAD_MSG_HPP

#include <ros/ros.h>
#define HEADER 0xf5

#define SERVOS_CMD           11  
#define SERVO_ALL_CMD        12
#define PAYLOAD_STATUS       77


#pragma pack(push, 1)


typedef struct {
    uint16_t quad_uid:14;
    uint16_t broadcast:1;
    uint16_t forward:1;
} quad_uid_s;

/* General: Header */
typedef struct {
    uint8_t   header;
    uint16_t   length;
    uint8_t   msg;
    quad_uid_s   id;
} ulink_header_t;

/* General: Checksum */
typedef struct {
    uint16_t   ck;
} ulink_checksum_t ;

 



/* PAYLOAD_STATUS */
typedef struct {
    int16_t servo1; //cm-deg
    int16_t servo2; //cm-deg
    int16_t servo3; //cm-deg
    int16_t servo4; //cm-deg
    int16_t servo5; //cm-deg
}payload_status_s;

/* SERVO_CMD */
typedef struct {
    uint8_t servo_id;
    int16_t servo_val; //cm-deg 
}servos_cmd_s;



#pragma pack(pop) 

#endif