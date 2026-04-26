/*
 * pn532_driver.h
 *
 *  Created on: 13-Mar-2026
 *      Author: rajes
 */

#ifndef INC_PN532_DRIVER_H_
#define INC_PN532_DRIVER_H_



#endif /* INC_PN532_DRIVER_H_ */
#ifndef PN532_DRIVER_H
#define PN532_DRIVER_H

#include "stm32f4xx_hal.h"

#define PN532_PREAMBLE     0x00
#define PN532_STARTCODE1   0x00
#define PN532_STARTCODE2   0xFF
#define PN532_POSTAMBLE    0x00

#define PN532_HOSTTOPN532  0xD4
#define PN532_PN532TOHOST  0xD5

#define PN532_COMMAND_GETFIRMWAREVERSION 0x02
#define PN532_COMMAND_SAMCONFIGURATION   0x14
#define PN532_COMMAND_INLISTPASSIVETARGET 0x4A

void PN532_Init(void);
uint8_t PN532_GetFirmware(void);
uint8_t PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uidLength);

#endif
