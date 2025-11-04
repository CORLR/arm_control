#ifndef CANOPEN_CONTROL_H
#define CANOPEN_CONTROL_H

#include "CO_app_STM32.h"
extern CANopenNodeSTM32 canOpenNodeSTM32;

void canopen_init();                                   // 初始化所有电机的CANopen参数
void canopen_init_taihu(uint8_t node_id);             // 初始化钛虎电机的CANopen PDO参数
void canopen_init_hechuan(uint8_t node_id);           // 初始化禾川电机的CANopen参数
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize);
CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize);

void callback_error(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit, const uint32_t infoCode);

#endif