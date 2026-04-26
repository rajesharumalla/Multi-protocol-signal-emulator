#include "pn532_spi.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;

#define NFC_MAX_NDEF_SIZE  256

#define PN532_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define PN532_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define PN532_SPI_DATAWRITE   0x01
#define PN532_SPI_STATUSREAD  0x02
#define PN532_SPI_DATAREAD    0x03

static const uint8_t ACK_FRAME[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

static void PN532_Wakeup(void)
{
    PN532_CS_LOW();
    HAL_Delay(2);
    uint8_t wake = 0x00;
    HAL_SPI_Transmit(&hspi2, &wake, 1, 10);
    PN532_CS_HIGH();
    HAL_Delay(50);
}

static uint8_t PN532_ReadStatus(void)
{
    uint8_t cmd = PN532_SPI_STATUSREAD;
    uint8_t status = 0;
    PN532_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    HAL_SPI_Receive(&hspi2, &status, 1, 10);
    PN532_CS_HIGH();
    return status;
}

static uint8_t PN532_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        uint8_t status = PN532_ReadStatus();
        if (status == 0x01) return 1;
        if (status == 0x00) PN532_Wakeup();
        HAL_Delay(10);
    }
    return 0;
}

static void PN532_WriteFrame(uint8_t *cmd, uint8_t cmdLen)
{
    uint8_t frame[64];
    uint8_t idx = 0;
    uint8_t len = cmdLen + 1;
    uint8_t lcs = (~len + 1) & 0xFF;
    uint8_t dcs = 0xD4;
    for (int i = 0; i < cmdLen; i++) dcs += cmd[i];
    dcs = (~dcs + 1) & 0xFF;
    frame[idx++] = 0x00;
    frame[idx++] = 0x00;
    frame[idx++] = 0xFF;
    frame[idx++] = len;
    frame[idx++] = lcs;
    frame[idx++] = 0xD4;
    for (int i = 0; i < cmdLen; i++) frame[idx++] = cmd[i];
    frame[idx++] = dcs;
    frame[idx++] = 0x00;
    uint8_t dir = PN532_SPI_DATAWRITE;
    PN532_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi2, &dir, 1, 10);
    HAL_SPI_Transmit(&hspi2, frame, idx, 100);
    PN532_CS_HIGH();
}

static void PN532_ReadFrame(uint8_t *buf, uint8_t len)
{
    uint8_t dir = PN532_SPI_DATAREAD;
    PN532_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi2, &dir, 1, 10);
    HAL_SPI_Receive(&hspi2, buf, len, 200);
    PN532_CS_HIGH();
}

static void PN532_SendACK(void)
{
    uint8_t dir = PN532_SPI_DATAWRITE;
    PN532_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi2, &dir, 1, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)ACK_FRAME, 6, 50);
    PN532_CS_HIGH();
    HAL_Delay(1);
}

// ════════════════════════════════════════════════════════
// InDataExchange — sends command to selected card
// cmdData: bytes after the 0x40, 0x01 header
// Returns 1 if status byte == 0x00 (success)
// ════════════════════════════════════════════════════════
static uint8_t PN532_InDataExchange(uint8_t *cmdData, uint8_t cmdLen,
                                     uint8_t *response, uint8_t responseLen)
{
    uint8_t fullCmd[34];
    fullCmd[0] = 0x40;   // InDataExchange
    fullCmd[1] = 0x01;   // target number 1
    memcpy(&fullCmd[2], cmdData, cmdLen);

    PN532_WriteFrame(fullCmd, cmdLen + 2);

    if (!PN532_WaitReady(500)) return 0;
    uint8_t ack[6];
    PN532_ReadFrame(ack, 6);
    if (ack[0]!=0x00 || ack[2]!=0xFF) return 0;

    if (!PN532_WaitReady(500)) return 0;
    PN532_ReadFrame(response, responseLen);

    // D5 41 = InDataExchange response code
    // response[7] = error byte (0x00 = success)
    if (response[5] != 0xD5 || response[6] != 0x41) return 0;
    if (response[7] != 0x00) return 0;

    PN532_SendACK();
    return 1;
}

// Authenticate Mifare block — ALL 4 UID bytes required
static uint8_t Mifare_Auth(uint8_t block, uint8_t keyType,
                             uint8_t *key, uint8_t *uid)
{
    uint8_t authCmd[12];
    authCmd[0]  = keyType;   // 0x60=KeyA, 0x61=KeyB
    authCmd[1]  = block;
    authCmd[2]  = key[0]; authCmd[3]  = key[1];
    authCmd[4]  = key[2]; authCmd[5]  = key[3];
    authCmd[6]  = key[4]; authCmd[7]  = key[5];
    authCmd[8]  = uid[0]; authCmd[9]  = uid[1];
    authCmd[10] = uid[2]; authCmd[11] = uid[3]; // all 4 bytes!

    uint8_t response[12];
    memset(response, 0, sizeof(response));
    return PN532_InDataExchange(authCmd, 12, response, sizeof(response));
}

// Read 16 bytes from Mifare block
static uint8_t Mifare_ReadBlock(uint8_t block, uint8_t *buf16)
{
    uint8_t readCmd[2] = {0x30, block};
    uint8_t response[26];
    memset(response, 0, sizeof(response));
    if (!PN532_InDataExchange(readCmd, 2, response, sizeof(response)))
        return 0;
    memcpy(buf16, &response[8], 16); // data at [8..23]
    return 1;
}

// Write 16 bytes to Mifare block
static uint8_t Mifare_WriteBlock(uint8_t block, uint8_t *buf16)
{
    uint8_t writeCmd[18];
    writeCmd[0] = 0xA0; writeCmd[1] = block;
    memcpy(&writeCmd[2], buf16, 16);
    uint8_t response[12];
    memset(response, 0, sizeof(response));
    return PN532_InDataExchange(writeCmd, 18, response, sizeof(response));
}

// ════════════════════════════════════════════════════════
// PUBLIC: Init
// ════════════════════════════════════════════════════════
void PN532_SPI_Init(void)
{
    PN532_CS_HIGH();
    HAL_Delay(100);
    PN532_Wakeup();
}

// ════════════════════════════════════════════════════════
// PUBLIC: GetFirmware + SAMConfig
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_GetFirmware(void)
{
    PN532_Wakeup();
    uint8_t cmd[] = {0x02};
    uint8_t response[12];
    memset(response, 0, sizeof(response));

    PN532_WriteFrame(cmd, 1);
    if (!PN532_WaitReady(1000)) return 0;
    PN532_ReadFrame(response, 6);
    if (response[0]!=0x00 || response[1]!=0x00 || response[2]!=0xFF ||
        response[3]!=0x00 || response[4]!=0xFF || response[5]!=0x00)
        return 0;

    if (!PN532_WaitReady(1000)) return 0;
    PN532_ReadFrame(response, 12);

    if (response[5] == 0xD5 && response[6] == 0x03) {
        PN532_SendACK();
        HAL_Delay(50);
        // SAMConfig
        uint8_t sam[] = {0x14, 0x01, 0x14, 0x01};
        uint8_t sr[10]; memset(sr, 0, sizeof(sr));
        PN532_WriteFrame(sam, 4);
        if (!PN532_WaitReady(1000)) return 0;
        PN532_ReadFrame(sr, 6);
        if (!PN532_WaitReady(1000)) return 0;
        PN532_ReadFrame(sr, sizeof(sr));
        if (sr[5]==0xD5 && sr[6]==0x15) PN532_SendACK();
        return 1;
    }
    return 0;
}

// ════════════════════════════════════════════════════════
// PUBLIC: ReadUID
// Card remains selected after this call!
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_ReadUID(uint8_t *uid, uint8_t *uidLength)
{
    PN532_Wakeup();
    uint8_t cmd[] = {0x4A, 0x01, 0x00};
    uint8_t response[30];
    memset(response, 0, sizeof(response));

    PN532_WriteFrame(cmd, 3);
    if (!PN532_WaitReady(1000)) return 0;
    PN532_ReadFrame(response, 6);
    if (response[0]!=0x00 || response[1]!=0x00 || response[2]!=0xFF)
        return 0;

    if (!PN532_WaitReady(1000)) return 0;
    PN532_ReadFrame(response, sizeof(response));

    if (response[5] != 0xD5 || response[6] != 0x4B) return 0;
    if (response[7] == 0) return 0;

    *uidLength = response[12];
    if (*uidLength == 0 || *uidLength > 7) return 0;

    for (int i = 0; i < *uidLength; i++)
        uid[i] = response[13 + i];

    PN532_SendACK();
    HAL_Delay(5); // card stays selected in PN532
    return 1;
}

// ════════════════════════════════════════════════════════
// PUBLIC: ReadNDEF_Mifare
// Call IMMEDIATELY after ReadUID — card still selected!
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_ReadNDEF_Mifare(uint8_t *uid,
                                   uint8_t *ndefData,
                                   uint16_t *ndefLen)
{
    *ndefLen = 0;

    uint8_t ndefKeyA[] = {0xD3,0xF7,0xD3,0xF7,0xD3,0xF7};
    uint8_t defKeyA[]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t buf[16];

    // Authenticate Sector 1 Block 4
    uint8_t authOK = Mifare_Auth(4, 0x60, ndefKeyA, uid);
    if (!authOK) {
        HAL_Delay(10);
        authOK = Mifare_Auth(4, 0x60, defKeyA, uid);
    }
    if (!authOK) return 0;

    HAL_Delay(5);

    if (!Mifare_ReadBlock(4, buf)) return 0;

    // Search for NDEF TLV 0x03
    for (uint8_t i = 0; i < 16; i++)
    {
        if (buf[i] == 0x03)
        {
            uint8_t len8 = buf[i+1];
            *ndefLen = len8;
            uint16_t collected = 0;
            uint8_t startByte = i + 2;

            for (uint8_t b = 0; b < 3 && collected < len8; b++)
            {
                uint8_t bd[16];
                if (b == 0) {
                    memcpy(bd, buf, 16);
                } else {
                    HAL_Delay(5);
                    if (!Mifare_Auth(4+b, 0x60, ndefKeyA, uid))
                        if (!Mifare_Auth(4+b, 0x60, defKeyA, uid))
                            break;
                    HAL_Delay(5);
                    if (!Mifare_ReadBlock(4+b, bd)) break;
                }
                uint8_t from = (b == 0) ? startByte : 0;
                for (uint8_t j = from; j < 16 && collected < len8; j++)
                    ndefData[collected++] = bd[j];
            }
            return (*ndefLen > 0) ? 1 : 0;
        }
        if (buf[i] == 0xFE) break;
    }
    return 0;
}

// ════════════════════════════════════════════════════════
// PUBLIC: WriteNDEF_Mifare
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_WriteNDEF_Mifare(uint8_t *uid,
                                    uint8_t *ndefData,
                                    uint16_t ndefLen)
{
    uint8_t ndefKeyA[] = {0xD3,0xF7,0xD3,0xF7,0xD3,0xF7};
    uint8_t defKeyA[]  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    uint8_t raw[48];
    memset(raw, 0, sizeof(raw));
    raw[0] = 0x03;
    raw[1] = (uint8_t)ndefLen;
    for (uint16_t i = 0; i < ndefLen && i+2 < 47; i++)
        raw[i+2] = ndefData[i];
    raw[2 + ndefLen] = 0xFE;

    for (uint8_t b = 0; b < 3; b++)
    {
        uint8_t block = 4 + b;
        HAL_Delay(5);
        if (!Mifare_Auth(block, 0x60, ndefKeyA, uid))
            if (!Mifare_Auth(block, 0x60, defKeyA, uid))
                return 0;
        HAL_Delay(5);
        if (!Mifare_WriteBlock(block, &raw[b*16])) return 0;
        HAL_Delay(10);
    }
    return 1;
}

// ════════════════════════════════════════════════════════
// PUBLIC: EmulateTag
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_EmulateTag(uint8_t *uid, uint8_t uidLength,
                              uint8_t *ndefData, uint16_t ndefLen)
{
    PN532_Wakeup();
    uint8_t cmd[46];
    memset(cmd, 0, sizeof(cmd));
    uint8_t idx = 0;
    cmd[idx++] = 0x8C; cmd[idx++] = 0x04;
    cmd[idx++] = 0x04; cmd[idx++] = 0x00; // ATQA
    cmd[idx++] = 0x08; // SAK Mifare Classic 1K
    for (int i = 0; i < 10; i++)
        cmd[idx++] = (i < uidLength) ? uid[i] : 0x00;
    for (int i = 0; i < 8; i++)  cmd[idx++] = 0x00;
    for (int i = 0; i < 19; i++) cmd[idx++] = 0x00;

    uint8_t response[20];
    memset(response, 0, sizeof(response));
    PN532_WriteFrame(cmd, idx);
    if (!PN532_WaitReady(8000)) return 0;
    PN532_ReadFrame(response, 6);
    if (response[0]!=0x00 || response[2]!=0xFF) return 0;
    if (!PN532_WaitReady(8000)) return 0;
    PN532_ReadFrame(response, sizeof(response));
    if (response[5] != 0xD5 || response[6] != 0x8D) return 0;
    PN532_SendACK();
    return 1;
}

// ════════════════════════════════════════════════════════
// PUBLIC: DebugScan
// ════════════════════════════════════════════════════════
uint8_t PN532_SPI_DebugScan(uint8_t *uid,
                             uint8_t *outSector,
                             uint8_t *outKey)
{
    uint8_t keys[][6] = {
        {0xD3,0xF7,0xD3,0xF7,0xD3,0xF7},
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
        {0xA0,0xA1,0xA2,0xA3,0xA4,0xA5},
        {0x00,0x00,0x00,0x00,0x00,0x00},
    };
    uint8_t buf[16];
    for (uint8_t sector = 0; sector < 16; sector++) {
        uint8_t block = sector * 4;
        for (uint8_t k = 0; k < 4; k++) {
            if (Mifare_Auth(block, 0x60, keys[k], uid)) {
                if (Mifare_ReadBlock(block, buf)) {
                    for (uint8_t i = 0; i < 16; i++) {
                        if (buf[i] == 0x03) {
                            *outSector = sector;
                            *outKey = k;
                            return 1;
                        }
                    }
                }
            }
            HAL_Delay(5);
        }
    }
    return 0;
}
