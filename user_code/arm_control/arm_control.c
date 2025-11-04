#include "arm_control.h"
#include "fdcan.h"
#include "tim.h"
#include "CO_app_STM32.h"
#include "OD.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "main.h"
#include "canopen_control.h"


int16_t dianliu_kp[4] = {0}; 
int16_t dianliu_ki[4] = {0};
int16_t dianliu_kd[4] = {0};
int16_t weizhi_kp[4]  = {0}; 
int16_t weizhi_ki[4]  = {0}; 
int16_t weizhi_kd[4]  = {0}; 

/* 存放各电机实际位置，下标=节点ID；motor_angle[0]未用 */
float motor_angle[8];       //存储当前电机角度和长度数据，电机禾川电机，压缩时读数变大，伸展时读数变小
int32_t motor_pos[8];
int32_t set_pos[8];         //pos[1]~pos[7] 设置 1~7号电机编码器目标位置
float set_motor[8];         //set_motor[1]~set_motor[7] 设置 1~3号电机目标长度，4~7号电机目标角度
float set_joint_angle[8];   //设置目标关节角度
float joint_angle[8];       //存储当前关节角度


extern TIM_HandleTypeDef htim17;
// extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern CO_t* CO;
// CO_CANrxMsg_t rcvMsg;

uint8_t g_ucTempBuf[20];
uint8_t g_reTempBuf[20];
uint32_t g_uiReadSize;

uint32_t speed = SPEED;
uint32_t accelerated = ACCELERATED;

int32_t xianwei;

uint32_t flag;

uint8_t data_error[64];  // 存放错误信息并通过串口上报的缓冲区
uint8_t data_jiaodu[64]; // 存放电机角度数据并通过串口上报的缓冲区
uint8_t read = 0;              // 串口接收的当前字节
uint8_t ucStatus = 0;          // 串口协议解析状态机当前状态
uint8_t len = 0;               // 当前接收数据包的长度
uint8_t ucCount = 0;           // 当前已接收数据字节计数
uint8_t motor_init = 0;       // 电机初始化完成标志



void arm_control(void)
{

    for (int i = 0; i < 8; i++) 
    {
        motor_angle[i] = 0;
        set_pos[i] = 0;
        set_motor[i] = 0;
        set_joint_angle[i] = 0;
        joint_angle[i] = 0;
    }


    while (1)
    {
        
        if (motor_init)
        {
            set_all_motor();
            set_all_motor_pos();
            collect_motor_positions();
            collect_joint_angles();
        }

    }
}

/**
 * @brief 设置禾川电机目标位置
 * 
 * @param node_id 禾川电机节点ID
 * @param target 目标位置
 */
void hechuan_motor_setpos(uint8_t node_id, int32_t target)
{
    g_ucTempBuf[0] = 0x00;
    g_ucTempBuf[1] = 0x35;
    g_ucTempBuf[2] = 0x0C;
    write_SDO(CO->SDOclient, node_id, 0x6081, 0x00, g_ucTempBuf, 3); //800000
    if(node_id == 3)
    {
        if (target > 100000) target = 100000;
        if (target < -3500000) target = -3500000;
    }
    else 
    {
        if (target > 900000) target = 900000;
        if (target < -400000) target = -400000;
    }
    g_ucTempBuf[0] = (target & 0xFF);
    g_ucTempBuf[1] = (target & 0xFF00) >> 8;
    g_ucTempBuf[2] = (target & 0XFF0000) >> 16;
    g_ucTempBuf[3] = (target & 0XFF000000) >> 24;
    write_SDO(CO->SDOclient, node_id, 0x607A, 0x00, g_ucTempBuf, 4);
    g_ucTempBuf[0] = 0x1F;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);
    g_ucTempBuf[0] = 0x0F;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);
}

/**
 * @brief 设置钛虎电机目标位置
 * 
 * @param node_id 钛虎电机节点ID
 * @param target 目标位置
 */
void taihu_motor_setpos(uint8_t node_id, int32_t target)
{
    switch (node_id) 
    {
        case 4:
            if (target > 2325000) target = 2325000;
            if (target < -1275000) target = -1275000;
            OD_RAM.x6066__607A.ID4607A = target;
            CO_TPDOsendRequest(&CO->TPDO[0]);
            break;
        case 5:
            if (target > 100000) target = 100000;
            if (target < -2200000) target = -2200000;
            OD_RAM.x6066__607A.ID5607A = target;
            CO_TPDOsendRequest(&CO->TPDO[1]);
            break;
        case 6:
            OD_RAM.x6066__607A.ID6607A = target;
            CO_TPDOsendRequest(&CO->TPDO[2]);
            break;
        case 7:
            OD_RAM.x6066__607A.ID7607A = target;
            CO_TPDOsendRequest(&CO->TPDO[3]);
            break;
        default:
            break;
    }
    g_ucTempBuf[0] = 0x1F;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);
}

/**
 * @brief 收集所有电机位置
 * 
 */
void collect_motor_positions() 
{
    /* 节点1~3：使用 SDO 读取 0x6064 */
    for (uint8_t id = 1; id <= 3; id++) 
    {
        if (read_SDO(CO->SDOclient, id, 0x6064, 0x00,
                     g_reTempBuf, sizeof(g_reTempBuf), &g_uiReadSize) == CO_SDO_AB_NONE
            && g_uiReadSize >= 4) 
        {
            motor_angle[id] = hechuan_pos_to_length(((int32_t)((uint32_t)g_reTempBuf[0] |
                                                    ((uint32_t)g_reTempBuf[1] << 8) |
                                                    ((uint32_t)g_reTempBuf[2] << 16)|
                                                    ((uint32_t)g_reTempBuf[3] << 24))));
            motor_pos[id] =                         ((int32_t)((uint32_t)g_reTempBuf[0] |
                                                    ((uint32_t)g_reTempBuf[1] << 8) |
                                                    ((uint32_t)g_reTempBuf[2] << 16)|
                                                    ((uint32_t)g_reTempBuf[3] << 24)));
        }
        clear_rebuff();
    }

    /* 节点4~7：已通过 TPDO 映射 0x6064，直接取 OD_RAM */
    motor_angle[4] = taihu_pos_to_angle(CANopenSlaveID4, OD_RAM.x6065__6064.ID46064);
    motor_angle[5] = taihu_pos_to_angle(CANopenSlaveID5, OD_RAM.x6065__6064.ID56064);
    motor_angle[6] = taihu_pos_to_angle(CANopenSlaveID6, OD_RAM.x6065__6064.ID66064);
    motor_angle[7] = taihu_pos_to_angle(CANopenSlaveID7, OD_RAM.x6065__6064.ID76064);
    
    
    motor_pos[4] = OD_RAM.x6065__6064.ID46064;
    motor_pos[5] = OD_RAM.x6065__6064.ID56064;
    motor_pos[6] = OD_RAM.x6065__6064.ID66064;
    motor_pos[7] = OD_RAM.x6065__6064.ID76064;
    
}

float taihu_pos_to_angle(uint8_t node_id, int32_t pos)
{
    float angle = 0.0f;
    switch (node_id) 
    {
        case 4:
            angle = (float)pos / 121.0f / 65536.0f * 2 * PI; // 121 减速比
            break;
        case 5:
            angle = (float)pos / 121.0f / 65536.0f * 2 * PI; // 121 减速比
            break;
        case 6:
            angle = (float)pos / 81.0f / 65536.0f * 2 * PI; // 81 减速比
            break;
        case 7:
            angle = (float)pos / 51.0f / 65536.0f * 2 * PI; // 51 减速比
            break;
        default:
            break;
    }
    return angle;
}

/**
 * @brief 将角度转换为钛虎电机位置值
 * 
 * @param node_id 钛虎电机节点ID
 * @param angle 目标角度（单位：rad）
 * @return int32_t 转换后的电机位置值
 */
int32_t taihu_angle_to_pos(uint8_t node_id, float angle)
{
    int32_t pos = 0;
    switch (node_id) 
    {
        case 4:
            pos = (int32_t)(angle / (2 * PI) * 121.0f * 65536.0f); // 121 减速比
            break;
        case 5:
            pos = (int32_t)(angle / (2 * PI) * 121.0f * 65536.0f); // 121 减速比
            break;
        case 6:
            pos = (int32_t)(angle / (2 * PI) * 81.0f * 65536.0f); // 81 减速比
            break;
        case 7:
            pos = (int32_t)(angle / (2 * PI) * 51.0f * 65536.0f); // 51 减速比
            break;
        default:
            break;
    }
    return pos;
}
/**
 * @brief 将禾川电机位置值转换为实际长度（单位：毫米）
 * 
 * @param pos 
 * @return float 
 */
float hechuan_pos_to_length(int32_t pos)
{
    return (float)pos / 65536.0f;
}
/**
 * @brief 将实际长度（单位：毫米）转换为禾川电机位置值
 * 
 * @param length 
 * @return int32_t 
 */
int32_t hechuan_length_to_pos(float length)
{
    return (int32_t)(length * 65536.0f);
}


/****************************************************************

1号电机和2号电机处关节角度分为2个部分：
1.1，2号电机伸长长度差与1，2号电机计算出roll
2.由固定杆长度与1，2号电机中间的长度差算出pitch

3号电机处关节角度
L:直线电机总长度
d_1关节转轴中心（A点）到电机固定端（B点）的固定距离
d_2关节转轴中心（A点）到电机移动端（C点）的固定距离
theta:由AB与AC构成的夹角（初始值不为0），减去初始值后为关节角度

*****************************************************************/


// 直线电机长度转换关节角度
void linear_to_joint_angle()
{
    float L1 = motor_angle[1] + LINEAR_ERROR_1; //1号电机长度
    float L2 = motor_angle[2] + LINEAR_ERROR_2; //2号电机长度
    joint_angle[7] = atan2f((L2 - L1),CENTER_DIS); //roll

    joint_angle[6] = atan2f(((L1 + L2))/2,SHAFT_DIS); //pitch

    float L3 = DEFAULT_LONG - motor_angle[3]; //3号电机长度，电机收缩时编码器为正数，伸长时为负数
    float COS_A = (D_1 * D_1 + D_2 * D_2 - L3 * L3) / (2 * D_1 * D_2);
    joint_angle[4] = acosf(COS_A) - DEFAULT_ANGLE;
}

void collect_joint_angles()
{
    linear_to_joint_angle();
    joint_angle[1] = motor_angle[4]; //4号电机角度
    joint_angle[2] = motor_angle[5]; //5号电机角度
    joint_angle[3] = motor_angle[6]; //6号电机角度
    joint_angle[5] = motor_angle[7]; //7号电机角度
}

//关节角度转换直线电机长度
/****************************************************************
tan(roll) = （L2 - L1）/CENTER_DIS
tan(pitch) = (L1 + L2)/2 / SHAFT_DIS
L1 = SHAFT_DIS * tan(pitch) - CENTER_DIS/2 * tan(roll)
L2 = SHAFT_DIS * tan(pitch) + CENTER_DIS/2 * tan(roll)
set_motor[1] = -L1
set_motor[2] = -L2
***************************************************************/

void joint_angle_to_linear()
{
    float roll = set_joint_angle[7];
    float pitch = set_joint_angle[6];
    set_motor[1] = -(SHAFT_DIS * tan(pitch) - CENTER_DIS/2 * tan(roll)) - LINEAR_ERROR_1;
    set_motor[2] = -(SHAFT_DIS * tan(pitch) + CENTER_DIS/2 * tan(roll)) - LINEAR_ERROR_2;

    float COS_A = cosf(set_joint_angle[4] + DEFAULT_ANGLE);
    float L3 = sqrtf(D_1 * D_1 + D_2 * D_2 - 2 * D_1 * D_2 * COS_A);
    set_motor[3] = DEFAULT_LONG - L3;
}

void set_all_motor()
{
    joint_angle_to_linear();
    set_motor[4] = set_joint_angle[1];
    set_motor[5] = set_joint_angle[2];
    set_motor[6] = set_joint_angle[3];
    set_motor[7] = set_joint_angle[5];
}

void set_all_motor_pos()
{
    set_pos[1] = hechuan_length_to_pos(set_motor[1]);
    set_pos[2] = hechuan_length_to_pos(set_motor[2]);
    set_pos[3] = hechuan_length_to_pos(set_motor[3]);
    set_pos[4] = taihu_angle_to_pos(CANopenSlaveID4, set_motor[4]);
    set_pos[5] = taihu_angle_to_pos(CANopenSlaveID4, set_motor[5]);
    set_pos[6] = taihu_angle_to_pos(CANopenSlaveID4, set_motor[6]);
    set_pos[7] = taihu_angle_to_pos(CANopenSlaveID4, set_motor[7]);
    hechuan_motor_setpos(CANopenSlaveID1, hechuan_length_to_pos(set_motor[1]));
    hechuan_motor_setpos(CANopenSlaveID2, hechuan_length_to_pos(set_motor[2]));
    hechuan_motor_setpos(CANopenSlaveID3, hechuan_length_to_pos(set_motor[3]));
    taihu_motor_setpos(CANopenSlaveID4, taihu_angle_to_pos(CANopenSlaveID4, set_motor[4]));
    taihu_motor_setpos(CANopenSlaveID5, taihu_angle_to_pos(CANopenSlaveID5, set_motor[5]));
    taihu_motor_setpos(CANopenSlaveID6, taihu_angle_to_pos(CANopenSlaveID6, set_motor[6]));
    taihu_motor_setpos(CANopenSlaveID7, taihu_angle_to_pos(CANopenSlaveID7, set_motor[7]));
}



void clear_rebuff(void)
{
    for (int i = 0; i < 20; i++)
    {
        g_reTempBuf[i] = 0;
    }
}