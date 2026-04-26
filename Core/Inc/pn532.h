#ifndef PN532_H
#define PN532_H

#include "stm32f4xx_hal.h"

void PN532_Init(void);
uint32_t PN532_GetFirmwareVersion(void);
void PN532_SAMConfig(void);
uint8_t PN532_ReadPassiveTargetID(uint8_t *uid, uint8_t *uidLength);
uint8_t PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uidLength);

#endif
