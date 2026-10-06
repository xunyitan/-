#ifndef JUDGE_UI_TASK_H
#define JUDGE_UI_TASK_H

#include "Sys.h"

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8


/* UI绘制内容cmdID */
#define UI_DataID_Delete   0x100 //客户端删除图形
#define UI_DataID_Draw1    0x101 //客户端绘制1个图形
#define UI_DataID_Draw2    0x102 //客户端绘制2个图形
#define UI_DataID_Draw5    0x103 //客户端绘制5个图形
#define UI_DataID_Draw7    0x104 //客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 //客户端绘制字符图形

/* UI图形操作 */
#define UI_Graph_invalid 0 //空操作
#define UI_Graph_Add     1 //增加图形
#define UI_Graph_Change  2 //修改图形
#define UI_Graph_Delete  3 //删除图形

typedef struct
{
   uint8_t SOF;                    //起始字节,固定0xA5
   uint16_t Data_Length;           //帧数据长度
   uint8_t Seq;                    //包序号
   uint8_t CRC8;                   //CRC8校验值
   uint16_t CMD_ID;                //命令ID
} UI_Packhead;             //帧头

typedef struct
{
   uint16_t Data_ID;               //内容ID
   uint16_t Sender_ID;             //发送者ID
   uint16_t Receiver_ID;           //接收者ID
} UI_Data_Operate;         //操作定义帧

typedef struct
{
   uint8_t Delete_Operate;         //删除操作
   uint8_t Layer;                  //删除图层
} UI_Data_Delete;          //删除图层帧


typedef struct
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   float graph_Float;              //浮点数据
} Float_Data;


typedef struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11;
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11;              //图形数据
} Graph_Data;


typedef struct
{
   Graph_Data Graph_Control;
   uint8_t show_Data[30];
} String_Data;                  //打印字符串数据

/* 自定义绘制UI结构体 -------------------------------------------------------*/
typedef __packed struct  //绘制UI UI图形数据
{
	uint8_t  graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct  //绘制UI UI字符串数据
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data_struct_t;

typedef __packed struct  //绘制UI UI删除图形数据
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header1_struct_t;

typedef __packed struct  //0x0301 机器人间通信 头结构体
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;


typedef __packed struct //绘制UI 绘制1个图形完整结构体
{
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //绘制UI 绘制2个图形完整结构体
{
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //绘制UI 绘制5个图形完整结构体
{
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //绘制UI 绘制7个图形完整结构体
{
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //绘制UI 绘制1字符串完整结构体
{ 
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //绘制UI UI删除图形完整结构体
{
	frame_header1_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;

void UI_Draw_Line(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY);        //截止坐标Y
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY);        //截止坐标Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius);      //半径
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               StartAngle,   //起始角度 [0,360]
								 uint16_t               EndAngle,     //截止角度 [0,360]
								 uint16_t               Width,        //线宽
								 uint16_t               CenterX,      //圆心坐标X
							   uint16_t               CenterY,      //圆心坐标Y
								 uint16_t               XHalfAxis,    //X半轴长
								 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Float(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData);   //数字内容
void UI_Draw_Int(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData);     //数字内容
void UI_Draw_String(string_data_struct_t *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData);   //字符串内容



unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint8_t CRC08_Calculate(uint8_t *Data, uint32_t Lenth /* Without check code lenth 1 */);
uint16_t CRC16_Calculate(uint8_t *Data, uint32_t Lenth /* Without check code lenth 2 */);
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* 与Counter相一致的UI_Graphx结构体头指针 */, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);
char* itoa(int num,char* str,int radix);

void UITask_Init(void);
void DrawUITask(void *argument);

#endif
