#include "canopen_control.h"
#include "main.h"
#include "cmsis_os2.h"
#include "canopen.h"
#include "OD.h"
#include "CO_app_STM32.h"
#include "stdint.h"
#include "fdcan.h"
#include "tim.h"
#include "arm_control.h"



CANopenNodeSTM32 canOpenNodeSTM32;
uint8_t canopen_init_flag = 0;         // CANopen初始化完成标志


void CANopen()
{

    canOpenNodeSTM32.CANHandle = &hfdcan2;              /* 使用CANFD2接口 */
	canOpenNodeSTM32.HWInitFunction = MX_FDCAN2_Init;     /* 初始化CANFD2 */  /*波特率修改NominalPrescaler，NominalPrescaler = 2 ，波特率1M.NominalPrescaler = 4，波特率0.5M*/
	canOpenNodeSTM32.timerHandle = &htim17;             /* 使用的定时器句柄 */
	canOpenNodeSTM32.desiredNodeID = 26;                /* Node-ID */
	canOpenNodeSTM32.baudrate = 1000;                   /* 波特率，单位KHz */
	canopen_app_init(&canOpenNodeSTM32);

    OD_RAM.x607A_target_position = 0;
    while(1)
    {
        /* CAN处理 */
        canopen_app_process();
        if (canopen_init_flag == 0)
        {
            CO_EM_initCallbackRx(CO->em,callback_error);
            canopen_init_flag = 1;
        }
        if (!motor_init)
        {
            canopen_init();
            motor_init = 1;
        }
    }
}

/**
 * @brief 错误回调函数
 * 
 * @param ident 
 * @param errorCode 
 * @param errorRegister 
 * @param errorBit 
 * @param infoCode 
 */
void callback_error(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit, const uint32_t infoCode)
{
    data_error[0] = 0xFF;
    data_error[1] = 0x05;
    data_error[2] = 0x03;
    data_error[3] = ident & 0xFF;
    data_error[4] = errorCode & 0xFF;
    data_error[5] = (errorCode & 0xFF00) >> 8;
    data_error[6] = errorRegister & 0xFF;
    data_error[7] = errorBit & 0xFF;
    // data_error[8] = infoCode & 0xFF;
    // data_error[9] = (infoCode & 0xFF00) >> 8;
    // data_error[10] = (infoCode & 0xFF0000) >> 16;
    // data_error[11] = (infoCode & 0xFF000000) >> 24;
}
/**
 * @brief 电机CANopen初始化
 * 
 */
void canopen_init()
{
    canopen_init_hechuan(CANopenSlaveID1);
    canopen_init_hechuan(CANopenSlaveID2);
    canopen_init_hechuan(CANopenSlaveID3);
    canopen_init_taihu(CANopenSlaveID4);
    canopen_init_taihu(CANopenSlaveID5);
    canopen_init_taihu(CANopenSlaveID6);
    canopen_init_taihu(CANopenSlaveID7);
}

/**
 * @brief 初始化钛虎电机的CANopen参数
 * 
 * @param node_id 
 */
void canopen_init_taihu(uint8_t node_id) //PDO 钛虎
{
    // 1. 配置RPDO1的COB-ID（0x1400:01），使能RPDO1，设置为标准帧，指定主站发送给该节点的PDO报文ID
    g_ucTempBuf[0] = node_id;
    g_ucTempBuf[1] = 0x02;
    g_ucTempBuf[2] = 0x00;
    g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1400, 0x01, g_ucTempBuf, 4);

    // 2. 配置RPDO1映射（0x1600:01），映射目标位置（0x607A，32位）
    g_ucTempBuf[0] = 0x20;
    g_ucTempBuf[1] = 0x00;
    g_ucTempBuf[2] = 0x7A;
    g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x01, g_ucTempBuf, 4);
    
    // g_ucTempBuf[0] = 0x10;
    // g_ucTempBuf[1] = 0x00;
    // g_ucTempBuf[2] = 0x40;
    // g_ucTempBuf[3] = 0x60;
    // write_SDO(CO->SDOclient, node_id, 0x1600, 0x02, g_ucTempBuf, 4);

    // 3. 设置RPDO1映射对象数量（0x1600:00）为1
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x00, g_ucTempBuf, 1);

    // 4. 配置TPDO1的COB-ID（0x1800:01），使能TPDO1，设置为标准帧，指定该节点上报PDO的报文ID
    g_ucTempBuf[0] = 0x80 + node_id;
    g_ucTempBuf[1] = 0x01;
    g_ucTempBuf[2] = 0x00;
    g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x01, g_ucTempBuf, 4);

    // 5. 设置TPDO1传输类型（0x1800:02）为1（同步传输）
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x02, g_ucTempBuf, 1);

    // 6. 设置TPDO1事件计时器（0x1800:06）为1ms，TPDO定时发送
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x06, g_ucTempBuf, 1);

    // 7. 配置TPDO1映射（0x1A00:01），映射实际位置（0x6064，32位）
    g_ucTempBuf[0] = 0x20;
    g_ucTempBuf[1] = 0x00;
    g_ucTempBuf[2] = 0x64;
    g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x01, g_ucTempBuf, 4);

    // g_ucTempBuf[0] = 0x10;
    // g_ucTempBuf[1] = 0x00;
    // g_ucTempBuf[2] = 0x41;
    // g_ucTempBuf[3] = 0x60;
    // write_SDO(CO->SDOclient, node_id, 0x1A00, 0x02, g_ucTempBuf, 4);

    // 8. 设置TPDO1映射对象数量（0x1A00:00）为1
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x00, g_ucTempBuf, 1);

    // 9. 设置软限位最小值（0x607D:01）为0x80000002
    g_ucTempBuf[0] = 0x02;
    g_ucTempBuf[1] = 0x00;
    g_ucTempBuf[2] = 0x00;
    g_ucTempBuf[3] = 0x80;
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x01, g_ucTempBuf, 4);

    // 10. 设置软限位最大值（0x607D:02）为0x7FFFFFFE
    g_ucTempBuf[0] = 0xFE;
    g_ucTempBuf[1] = 0xFF;
    g_ucTempBuf[2] = 0xFF;
    g_ucTempBuf[3] = 0x7F;
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x02, g_ucTempBuf, 4);

    // 11. 设置轮廓速度（0x6081:00)
    g_ucTempBuf[0] = speed & 0xFF; //0x88;
    g_ucTempBuf[1] = (speed >> 8) & 0xFF;//0x13;//5000
    write_SDO(CO->SDOclient, node_id, 0x6081, 0x00, g_ucTempBuf, 2);
    
    //12. 设置加速度
    g_ucTempBuf[0] = accelerated & 0xFF;//0x20;
    g_ucTempBuf[1] = (accelerated >> 8) & 0xFF;//0x03;//800
    write_SDO(CO->SDOclient, node_id, 0x6083, 0x00, g_ucTempBuf, 2);

    //13. 设置减速度
    g_ucTempBuf[0] = accelerated & 0xFF;//0x20;
    g_ucTempBuf[1] = (accelerated >> 8) & 0xFF;//0x03;//800
    write_SDO(CO->SDOclient, node_id, 0x6084, 0x00, g_ucTempBuf, 2);
    
    // 14. 设置模式（0x6060:00）为1（轮廓位置模式）
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x6060, 0x00, g_ucTempBuf, 1);  //轮廓位置模式

    // 15. 依次写入控制字（0x6040:00）为0x06、0x07、0x0F、0x1F，完成电机使能、准备、启动等状态切换
    g_ucTempBuf[0] = 0x06;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);//0000000000000110
    g_ucTempBuf[0] = 0x07;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);//0000000000000111
    g_ucTempBuf[0] = 0x0F;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);//0000000000001111
    g_ucTempBuf[0] = 0x1F;
    g_ucTempBuf[1] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);//0000000000011111
}

/**
 * @brief 初始化禾川电机的CANopen参数
 * 
 * @param node_id 
 */
void canopen_init_hechuan(uint8_t node_id) //SDO 禾川
{
        // 1. 设置模式（0x6060:00）为1，位置模式
        g_ucTempBuf[0] = 0x01;
        g_ucTempBuf[1] = 0x00;
        write_SDO(CO->SDOclient, node_id, 0x6060, 0x00, g_ucTempBuf, 2);  //位置模式

        // 2. 写入控制字（0x6040:00）为0x06，使能准备
        g_ucTempBuf[0] = 0x06;
        g_ucTempBuf[1] = 0x00;
        write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);

        // 3. 写入控制字（0x6040:00）为0x07，切换到使能状态
        g_ucTempBuf[0] = 0x07;
        g_ucTempBuf[1] = 0x00;
        write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);

        // 4. 写入控制字（0x6040:00）为0x0F，启动电机
        g_ucTempBuf[0] = 0x0F;
        g_ucTempBuf[1] = 0x00;
        write_SDO(CO->SDOclient, node_id, 0x6040, 0x00, g_ucTempBuf, 2);
}

/**
 * @brief 
 * 
 * @param SDO_C CANopen SDO 客户端对象指针
 * @param nodeId 目标 CANopen 从站的节点ID
 * @param index 对象字典的索引
 * @param subIndex 对象字典的子索引
 * @param data 指向要写入的数据缓冲区
 * @param dataSize 要写入的数据长度
 * @return CO_SDO_abortCode_t 
 */
CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;
 
    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }
 
    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex,
                                           dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }
 
    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }
 
    //download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
 
        SDO_ret = CO_SDOclientDownload(SDO_C,
                                       timeDifference_us,
                                       false,
                                       bufferPartial,
                                       &abortCode,
                                       NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }
 
        osDelay(timeDifference_us / 1000);
    } while(SDO_ret > 0);
 
    return CO_SDO_AB_NONE;
}

/**
 * @brief 
 * 
 * @param SDO_C CANopen SDO 客户端对象指针
 * @param nodeId 目标 CANopen 从站的节点ID
 * @param index 对象字典的索引
 * @param subIndex 对象字典的子索引
 * @param buf 指向用于存储读取数据的缓冲区
 * @param bufSize 缓冲区的大小
 * @param readSize 实际读取的数据长度指针
 * @return CO_SDO_abortCode_t 
 */
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize)
{
    CO_SDO_return_t SDO_ret;
 
    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId); 
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }
 
    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }
 
    // upload data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
 
        SDO_ret = CO_SDOclientUpload(SDO_C,
                                     timeDifference_us,
                                     false,
                                     &abortCode,
                                     NULL, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }
 
        osDelay(timeDifference_us / 1000);
    } while(SDO_ret > 0);
 
    // copy data to the user buffer (for long data function must be called
    // several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
 
    return CO_SDO_AB_NONE;
}