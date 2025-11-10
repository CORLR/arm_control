#include "protocol.h"
#include "stdbool.h" 
#include "stdio.h"   
#include "stdint.h"
#include "arm_control.h"

// -----------------------------------------------------------------
// 1. 协议定义 (根据您的图片)
// -----------------------------------------------------------------

// 帧头
#define FRAME_HEADER 0x55 

// 功能码 (目前固定为 0x01)
#define FUNCTION_CODE 0x01

// 数据长度 (7 个编码器 * 2 字节/编码器)
#define FRAME_DATA_LENGTH 14 


typedef enum {
    STATE_WAIT_HEADER,      // 状态1: 等待 0x55
    STATE_READ_FUNCTION,    // 状态2: 等待 0x01
    STATE_READ_DATA         // 状态3: 接收 14 字节数据
} ParseState_t;


// 当前状态
static ParseState_t g_parse_state = STATE_WAIT_HEADER;

// 临时缓冲区，用于拼装 14 字节的数据
static uint8_t g_frame_buffer[FRAME_DATA_LENGTH]; 

// 当前已接收的数据字节计数器
static uint8_t g_byte_counter = 0;



/**
 * @brief 协议解析，逐字节处理数据
 * 
 * @param byte 
 */
static void Protocol_Parse_Byte(uint8_t byte)
{
    switch (g_parse_state)
    {
        case STATE_WAIT_HEADER:
            if (byte == FRAME_HEADER)
            {
                g_parse_state = STATE_READ_FUNCTION;
            }
            break;

        case STATE_READ_FUNCTION:
            if (byte == FUNCTION_CODE)
            {
                g_byte_counter = 0; // 清零数据计数器
                g_parse_state = STATE_READ_DATA;
            }
            else
            {
                if (byte == FRAME_HEADER)
                {
                }
                else
                {
                    g_parse_state = STATE_WAIT_HEADER;
                }
            }
            break;

        case STATE_READ_DATA:
            g_frame_buffer[g_byte_counter] = byte;
            g_byte_counter++;

            if (g_byte_counter >= FRAME_DATA_LENGTH)
            {
                process_encoder_data(g_frame_buffer, FRAME_DATA_LENGTH);
                g_parse_state = STATE_WAIT_HEADER;
            }
            break;
            
        default:
            g_parse_state = STATE_WAIT_HEADER;
            break;
    }
}


/**
 * @brief 协议解析入口函数
 * 
 * @param chunk 
 * @param len 
 */
void Protocol_Parse_Chunk(uint8_t* chunk, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        Protocol_Parse_Byte(chunk[i]);
    }
}


/**
 * @brief 处理编码器数据
 * 
 * @param pData 
 * @param len 
 */
void process_encoder_data(uint8_t* pData, uint32_t len)
{

    if (len != FRAME_DATA_LENGTH)
    {
        // 理论上不会发生
        return; 
    }

    
    // set_joint_angle[1] = -((float)((pData[0] << 8 | pData[1]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_1);

    // set_joint_angle[2] = (float)((pData[2] << 8 | pData[3]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_2;
    // // set_joint_angle[3] = (float)((pData[4] << 8 | pData[5]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_3;
    // set_joint_angle[3] = 0.0f; //3号编码器出错不发数据
    // set_joint_angle[4] = (float)((pData[6] << 8 | pData[7]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_4;
    // set_joint_angle[5] = -((float)((pData[8] << 8 | pData[9]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_5);
    // // set_joint_angle[6] = -((float)((pData[10] << 8 | pData[11]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_6);
    // set_joint_angle[6] = 0.0f;
    // // set_joint_angle[7] = (float)((pData[12] << 8 | pData[13]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_7;
    // set_joint_angle[7] = 0.0f; //7号编码器出错不发数据
    set_joint_angle[1] = 0.0f;
    set_joint_angle[2] = 0.0f;
    set_joint_angle[3] = 0.2f;
    set_joint_angle[4] = 0.0f;
    set_joint_angle[5] = 0.0f;
    set_joint_angle[6] = 0.0f;
    set_joint_angle[7] = 0.0f;
    for(int i = 1; i <= 7; i++)
    {
        // if(set_joint_angle[i] > PI) set_joint_angle[i] -= 2 * PI;
        // if(set_joint_angle[i] < -PI) set_joint_angle[i] += 2 * PI;
    }
}