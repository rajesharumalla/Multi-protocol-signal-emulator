#ifndef CC1101_H
#define CC1101_H

#include "main.h"
#include <stdint.h>

#define CC1101_OK           1
#define CC1101_FAIL         0
#define CC1101_MAX_PAYLOAD  61

// Expose strobe commands needed in main.c
#define CC1101_SRX   0x34
#define CC1101_SIDLE 0x36

// Public functions
uint8_t CC1101_Init(void);
uint8_t CC1101_SendData(uint8_t *data, uint8_t len);
uint8_t CC1101_ReceiveData(uint8_t *data, uint8_t *len,
                            uint32_t timeout_ms);
void    CC1101_SetIdle(void);
void    CC1101_Strobe(uint8_t cmd);  // expose strobe

#endif
