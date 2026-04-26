/*
 * pn532_spi.h
 *
 *  Created on: 26-Mar-2026
 *      Author: rajes
 */
#ifndef PN532_SPI_H
#define PN532_SPI_H

#include "main.h"

void PN532_SPI_Init(void);
uint8_t PN532_SPI_GetFirmware(void);
uint8_t PN532_SPI_ReadUID(uint8_t *uid, uint8_t *uidLength);
uint8_t PN532_SPI_EmulateTag(uint8_t *uid, uint8_t uidLength,
                              uint8_t *ndefData, uint16_t ndefLen);
uint8_t PN532_SPI_ReadNDEF_Mifare(uint8_t *uid,
                                   uint8_t *ndefData,
                                   uint16_t *ndefLen);
uint8_t PN532_SPI_WriteNDEF_Mifare(uint8_t *uid,
                                    uint8_t *ndefData,
                                    uint16_t ndefLen);

uint8_t PN532_SPI_DebugScan(uint8_t *uid, uint8_t *outSector, uint8_t *outKey);
#endif
