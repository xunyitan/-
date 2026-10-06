#ifndef CRC_H
#define CRC_H

#include "Sys.h"

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint32_t len);
unsigned short CRC_Modbus(uint8_t *buf, uint8_t len);
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);


#endif
