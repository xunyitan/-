#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"
#include "djl_driver.h"
#include "Sys.h"

#define CHASSIS_CAN hcan1
#define SCAP_CAN    hcan1
#define GIMBAL_CAN  hcan2
#define SCAP_Slave  0x051

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
	CAN_PIT_MOTOR_ID = 0x205,
	CAN_YAW_MOTOR_ID = 0x206,
	CAN_TRIGGER_MOTOR_ID = 0x207,
	CAN_GIMBAL_ALL_ID = 0x1FF,//标识符0x1ff与0x1fe要去上位机更改，使用电压环就是0x1ff，使用电流环就是0x1fe
	SCAP_Master=0x061,
	CAN_4310ID=0x01,
	CAN_FINDINGMODE_ID=0xFF,  
	CAN_QUATERNION_ID=0x100,//四元数ID
	CAN_BULLETSPEED_ID=0x101,//子弹速度ID
}can_msg_ide;

//rm_motor_data
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_t;

extern float Motor_Power[4];
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void C3508_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void G6020_cmd(int16_t motor1,int16_t motor2);

#endif
