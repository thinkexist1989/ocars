#ifndef RM_DEFINE_H
#define RM_DEFINE_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了机械臂接口函数中使用到的结构体和错误代码类型
//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "cJSON.h"

#ifdef _WIN32
#define MSG_DONTWAIT 0
#include <winsock2.h>
#include <windows.h>
#include <process.h>
typedef SOCKET  SOCKHANDLE;
#endif

#ifdef __linux
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef int SOCKHANDLE;
#endif



typedef unsigned char byte;

//位姿结构体
typedef struct
{
    //位置
    float px;
    float py;
    float pz;
    //欧拉角
    float rx;
    float ry;
    float rz;
}POSE;

// 回调函数结构体
typedef struct
{
    int sockhand;       // 返回调用时句柄
    int codeKey;       // 调用接口类型
    int errCode;        // 接口错误码
    POSE pose;          // 位姿信息
    float joint[7];     // 角度信息
    int nforce;        // 力位混合透传 返回力大小
    int sys_err;        // 力位混合透传 返回系统错误


} CallbackData;

//坐标系
typedef struct
{
    char name[10];    //坐标系名称,不超过10个字符
}FRAME_NAME;

//坐标系
typedef struct
{
    FRAME_NAME frame_name;    //坐标系名称
    POSE pose;              //坐标系位姿
    float payload;     //坐标系末端负载重量
    float x;           //坐标系末端负载位置
    float y;           //坐标系末端负载位置
    float z;           //坐标系末端负载位置
}FRAME;

//机械臂控制模式
typedef enum
{
    None_Mode = 0,     //无规划
    Joint_Mode = 1,    //关节空间规划
    Line_Mode = 2,     //笛卡尔空间直线规划
    Circle_Mode = 3,   //笛卡尔空间圆弧规划
}ARM_CTRL_MODES;

//机械臂位置示教模式
typedef enum
{
    X_Dir = 0,       //X轴方向
    Y_Dir = 1,       //Y轴方向
    Z_Dir = 2,       //Z轴方向
}POS_TEACH_MODES;

//机械臂姿态示教模式
typedef enum
{
    RX_Rotate = 0,       //RX轴方向
    RY_Rotate = 1,       //RY轴方向
    RZ_Rotate = 2,       //RZ轴方向
}ORT_TEACH_MODES;

//控制器通讯方式选择
typedef enum
{
    WIFI_AP = 0,       //WIFI AP模式
    WIFI_STA = 1,      //WIFI STA模式
    BlueTeeth = 2,     //蓝牙模式
    USB       = 3,     //通过控制器UART-USB接口通信
    Ethernet  = 4      //以太网口
} ARM_COMM_TYPE;

//机械臂自由度
#define   ARM_DOF               7              //机械臂自由度
//机械臂状态参数
typedef struct
{
    float joint[ARM_DOF];         //关节角度
    float temperature[ARM_DOF];   //关节温度
    float voltage[ARM_DOF];       //关节电压
    float current[ARM_DOF];       //关节电流
    byte en_state[ARM_DOF];       //使能状态
    uint16_t err_flag[ARM_DOF];   //关节错误代码
    uint16_t sys_err;       //机械臂系统错误代码
}JOINT_STATE;

//位置
typedef struct
{
    //position
    float x;
    float y;
    float z;
    //orientation
    float w;
    float x_;
    float y_;
    float z_;
}POSE2;

//姿态
typedef struct
{
    float rx;
    float ry;
    float rz;
}ORT;
//typedef struct
//{
//    POSE2 pose;
//    ORT ort;
//}KINEMATIC;
//旋转矩阵
typedef struct
{
    int irow;
    int iline;
    float data[4][4];
}Matrix;

//位置
typedef struct
{
    float x;
    float y;
    float z;
}pos;

//四元数姿态
typedef struct
{
    float w;
    float x;
    float y;
    float z;
}ort;

//欧拉角姿态
typedef struct
{
    float Phi;
    float Theta;
    float Psi;
}eul;

//机械臂位姿
typedef struct
{
    pos position;    // px
    ort orientation; // rx
    eul euler;
}Pose;

typedef void (*RM_Callback)(CallbackData data);
#define  M_PI_RAD    0.0174533f
#define  MI_PI_ANG   57.2957805f
#define  PI          3.14159f

#define  M_PI		 3.14159265358979323846
#define  DELTA       0.26f   //关节判断角度差
#define  DELTA2      2*PI    //关节运动到该处

// 机械臂型号
#define ARM_65      65
#define ARM_63_1    631
#define ARM_63_2    632
#define ARM_75      75

// 是否打印日志
#define RM_DISPLAY_LOG 0  // 0 不打印, 1打印
#define RM_NONBLOCK 0   // 非阻塞
#define RM_BLOCK 1      // 阻塞

#define RM_INPUT 0      // 输入
#define RM_OUTPUT 1     // 输出

#define RM_LOW 0        // 低
#define RM_TALL 1       // 高

#define PORT_CONTROLLER  0 // 控制器
#define PORT_ENDMODEL    1 // 末端接口板

#define NAVIGATION_MAGNETIC 0 // 磁条导航
#define OPING_CONTROLLER 1    // 开环控制模式

#define TRAJECTORY_FILE_NAME_MAX_LENGTH 300
//系统初始化错误代码
#define SYS_NORMAL                          0x0000          // 系统运行正常
#define CONTROLLER_DATE_RETURN_FALSE        0x0001          // 消息请求返回FALSE
#define INIT_MODE_ERR                       0x0002          // 机械臂未初始化或输入型号非法
#define INIT_TIME_ERR                       0x0003          // 非法超时时间
#define INIT_SOCKET_ERR                     0x0004          // Socket 初始化失败
#define SOCKET_CONNECT_ERR                  0x0005          // Socket 连接失败
#define SOCKET_SEND_ERR                     0x0006          // Socket 发送失败
#define SOCKET_TIME_OUT                     0x0007          // Socket 通讯超时
#define UNKNOWN_ERR                         0x0008          // 未知错误
#define CONTROLLER_DATA_LOSE_ERR            0x0009          // 数据不完整
#define CONTROLLER_DATE_ARR_NUM_ERR         0x000A          // 数组长度错误
#define WRONG_DATA_TYPE                     0x000B          // 数据类型错误
#define MODEL_TYPE_ERR                      0x000C          // 型号错误
#define CALLBACK_NOT_FIND                   0x000D          // 缺少回调函数
#define ARM_ABNORMAL_STOP                   0x000E          // 机械臂异常停止
#define TRAJECTORY_FILE_LENGTH_ERR          0x000F          // 轨迹文件名称过长
#define TRAJECTORY_FILE_CHECK_ERR           0x0010          // 轨迹文件校验失败
#define TRAJECTORY_FILE_READ_ERR            0x0011          // 轨迹文件读取失败
#define CONTROLLER_BUSY                     0x0012          // 控制器忙,请稍后再试
#define ILLEGAL_INPUT                       0x0013          // 非法输入
#define QUEUE_LENGTH_FULL                   0x0014          // 数据队列已满

// 回调函数对应Code
#define MOVEJ_CANFD_CB                      0x0001          // 角度透传非阻塞标识码
#define MOVEP_CANFD_CB                      0x0002          // 位姿透传非阻塞标识码
#define FORCE_POSITION_MOVE_CB              0x0003          // 力位混合透传

#ifdef __cplusplus
}
#endif
#endif // RM_DEFINE_H
