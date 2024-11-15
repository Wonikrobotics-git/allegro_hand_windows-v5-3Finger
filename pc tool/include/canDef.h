/*
 *\brief Constants and enums for CAN communication
 *
 *$Author: Sangyup Yi $
 *$Date: 2012/5/11 23:34:00 $
 *$Revision: 1.0 $
 */ 

#ifndef _CANDEF_H
#define _CANDEF_H


#ifdef USING_NAMESPACE_CANAPI
#   define CANAPI_BEGIN namespace CANAPI {
#   define CANAPI_END };
#else
#   define CANAPI_BEGIN
#   define CANAPI_END
#endif


CANAPI_BEGIN


////////////////////////////////////////////////
//  Macros
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

////////////////////////////////////////////////
//  Constants
#define NUM_OF_FINGERS          3 // number of fingers
#define NUM_OF_TEMP_SENSORS     3 // number of temperature sensors

////////////////////////////////////////////////
//  Structures
#pragma pack(push, 1)
typedef struct
{
	unsigned short position;
	unsigned short imu;
	unsigned short temp;
} can_period_msg_t;

typedef struct
{
	unsigned char set;
	unsigned char did;
	unsigned int baudrate;
} can_config_msg_t;
#pragma pack(pop)

////////////////////////////////////////////////
//  Define CAN Command
#define ID_CMD_SYSTEM_ON                0x40
#define ID_CMD_SYSTEM_OFF               0x41
#define ID_CMD_SET_TORQUE               0x60
#define ID_CMD_SET_TORQUE_1             (ID_CMD_SET_TORQUE+0)
#define ID_CMD_SET_TORQUE_2             (ID_CMD_SET_TORQUE+1)
#define ID_CMD_SET_TORQUE_3             (ID_CMD_SET_TORQUE+2)
#define ID_CMD_SET_TORQUE_4             (ID_CMD_SET_TORQUE+3)
#define ID_CMD_SET_POSE_1               0xE0
#define ID_CMD_SET_POSE_2               0xE1
#define ID_CMD_SET_POSE_3               0xE2
#define ID_CMD_SET_POSE_4               0xE3
#define ID_CMD_SET_PERIOD               0x81
#define ID_CMD_CONFIG                   0x68

#define ID_CMD_FINGERTIP				 0xF0
#define ID_CMD_FINGERTIP_1				 (ID_CMD_FINGERTIP+0)
#define ID_CMD_FINGERTIP_2				 (ID_CMD_FINGERTIP+1)
#define ID_CMD_FINGERTIP_3				 (ID_CMD_FINGERTIP+2)
#define ID_CMD_FINGERTIP_4				 (ID_CMD_FINGERTIP+3)

#define ID_CMD_PICK_STATUS				 0x11
#define ID_CMD_PLACE_STATUS				 0x12

////////////////////////////////////////////////
//  Define CAN Data Reqeust (RTR)
#define ID_RTR_HAND_INFO                0x80
#define ID_RTR_SERIAL                   0x88
#define ID_RTR_FINGER_POSE              0x20
#define ID_RTR_FINGER_POSE_1            (ID_RTR_FINGER_POSE+0)
#define ID_RTR_FINGER_POSE_2            (ID_RTR_FINGER_POSE+1)
#define ID_RTR_FINGER_POSE_3            (ID_RTR_FINGER_POSE+2)
#define ID_RTR_STATUS					0x101


CANAPI_END

#endif
