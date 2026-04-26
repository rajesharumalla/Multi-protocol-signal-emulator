#include "cc1101.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

#define CC1101_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define CC1101_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define CC1101_GDO0()    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)

// CC1101 Register addresses
#define CC1101_IOCFG1   0x01
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_CHANNR   0x0A
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM1    0x17
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_RXBYTES  0x3B
#define CC1101_SRES     0x30
#define CC1101_SRX      0x34
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_SFRX     0x3A
#define CC1101_SFTX     0x3B
#define CC1101_WRITE_SINGLE 0x00
#define CC1101_WRITE_BURST  0x40
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0

// ── Switch SPI2 to MSB (for CC1101) ─────────────────
static void SPI2_SetMSB(void)
{
    hspi2.Instance->CR1 &= ~SPI_CR1_SPE;
    hspi2.Instance->CR1 &= ~SPI_CR1_LSBFIRST;
    hspi2.Instance->CR1 |=  SPI_CR1_SPE;
}

// ── Switch SPI2 back to LSB (for PN532) ─────────────
static void SPI2_SetLSB(void)
{
    hspi2.Instance->CR1 &= ~SPI_CR1_SPE;
    hspi2.Instance->CR1 |=  SPI_CR1_LSBFIRST;
    hspi2.Instance->CR1 |=  SPI_CR1_SPE;
}

static uint8_t SPI2_Transfer(uint8_t byte)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &byte, &rx, 1, 10);
    return rx;
}

// Wait for MISO (GDO1/PB14) to go LOW = CC1101 ready
static void CC1101_WaitMISO(void)
{
    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
        if ((HAL_GetTick() - t) > 100) break;
}

static void CC1101_WriteReg(uint8_t addr, uint8_t val)
{
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(addr | CC1101_WRITE_SINGLE);
    SPI2_Transfer(val);
    CC1101_CS_HIGH();
    SPI2_SetLSB();
}

static void CC1101_WriteBurst(uint8_t addr,
                               uint8_t *data, uint8_t len)
{
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(addr | CC1101_WRITE_BURST);
    for (uint8_t i = 0; i < len; i++)
        SPI2_Transfer(data[i]);
    CC1101_CS_HIGH();
    SPI2_SetLSB();
}

static uint8_t CC1101_ReadReg(uint8_t addr)
{
    uint8_t val;
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(addr | CC1101_READ_SINGLE);
    val = SPI2_Transfer(0x00);
    CC1101_CS_HIGH();
    SPI2_SetLSB();
    return val;
}

static uint8_t CC1101_ReadStatus(uint8_t addr)
{
    uint8_t val;
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(addr | CC1101_READ_BURST);
    val = SPI2_Transfer(0x00);
    CC1101_CS_HIGH();
    SPI2_SetLSB();
    return val;
}

void CC1101_Strobe(uint8_t cmd)
{
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(cmd);
    CC1101_CS_HIGH();
    SPI2_SetLSB();
}

// ════════════════════════════════════════════════════
// CC1101_Init — 433.92MHz, 4.8kbps, OOK modulation
// ════════════════════════════════════════════════════
uint8_t CC1101_Init(void)
{
    CC1101_CS_HIGH();
    HAL_Delay(10);

    // Hardware reset sequence
    SPI2_SetMSB();
    CC1101_CS_LOW();
    HAL_Delay(1);
    CC1101_CS_HIGH();
    HAL_Delay(1);
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(CC1101_SRES);
    HAL_Delay(10);
    CC1101_WaitMISO();
    CC1101_CS_HIGH();
    SPI2_SetLSB();
    HAL_Delay(10);

    // Verify chip — version register should be 0x14
    uint8_t version = CC1101_ReadStatus(0xF1);
    if (version == 0x00 || version == 0xFF)
        return CC1101_FAIL;

    // *** CRITICAL: Set GDO1 to HiZ when CSN HIGH ***
    // Prevents GDO1 from corrupting shared PN532 MISO line!
    CC1101_WriteReg(CC1101_IOCFG1, 0x2E);

    // GDO0 asserts when packet received (end of packet)
    CC1101_WriteReg(CC1101_IOCFG0,   0x06);

    // Packet control — variable length mode
    CC1101_WriteReg(CC1101_FIFOTHR,  0x47);
    CC1101_WriteReg(CC1101_PKTLEN,   0x3D); // max length
    CC1101_WriteReg(CC1101_PKTCTRL1, 0x04); // no addr check
    CC1101_WriteReg(CC1101_PKTCTRL0, 0x01); // variable packet length

    // 433.92 MHz frequency
    CC1101_WriteReg(CC1101_FSCTRL1,  0x06);
    CC1101_WriteReg(CC1101_FSCTRL0,  0x00);
    CC1101_WriteReg(CC1101_FREQ2,    0x10);
    CC1101_WriteReg(CC1101_FREQ1,    0xB0);
    CC1101_WriteReg(CC1101_FREQ0,    0x71);

    // Modem: 4.8kbps, OOK, no sync word
    CC1101_WriteReg(CC1101_MDMCFG4,  0x87);
    CC1101_WriteReg(CC1101_MDMCFG3,  0x83);
    CC1101_WriteReg(CC1101_MDMCFG2,  0x30);
    CC1101_WriteReg(CC1101_MDMCFG1,  0x22);
    CC1101_WriteReg(CC1101_MDMCFG0,  0xF8);
    CC1101_WriteReg(CC1101_DEVIATN,  0x00);
    CC1101_WriteReg(CC1101_MCSM1,    0x30);
    CC1101_WriteReg(CC1101_MCSM0,    0x18);
    CC1101_WriteReg(CC1101_FOCCFG,   0x16);
    CC1101_WriteReg(CC1101_BSCFG,    0x6C);
    CC1101_WriteReg(CC1101_AGCCTRL2, 0x03);
    CC1101_WriteReg(CC1101_AGCCTRL1, 0x40);
    CC1101_WriteReg(CC1101_AGCCTRL0, 0x91);
    CC1101_WriteReg(CC1101_FREND1,   0x56);
    CC1101_WriteReg(CC1101_FREND0,   0x11);
    CC1101_WriteReg(CC1101_FSCAL3,   0xE9);
    CC1101_WriteReg(CC1101_FSCAL2,   0x2A);
    CC1101_WriteReg(CC1101_FSCAL1,   0x00);
    CC1101_WriteReg(CC1101_FSCAL0,   0x1F);
    CC1101_WriteReg(CC1101_TEST2,    0x81);
    CC1101_WriteReg(CC1101_TEST1,    0x35);
    CC1101_WriteReg(CC1101_TEST0,    0x09);

    // PA table — max power for OOK
    uint8_t pa[] = {0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00};
    CC1101_WriteBurst(0x3E, pa, 8);

    CC1101_Strobe(CC1101_SIDLE);
    HAL_Delay(5);
    CC1101_Strobe(CC1101_SFRX);
    CC1101_Strobe(CC1101_SFTX);

    return CC1101_OK;
}

// ════════════════════════════════════════════════════
// CC1101_SendData
// ════════════════════════════════════════════════════
uint8_t CC1101_SendData(uint8_t *data, uint8_t len)
{
    if (len == 0 || len > CC1101_MAX_PAYLOAD)
        return CC1101_FAIL;

    CC1101_Strobe(CC1101_SIDLE);
    HAL_Delay(2);
    CC1101_Strobe(CC1101_SFTX);
    HAL_Delay(2);

    // Write to TX FIFO
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(0x7F);      // TX FIFO burst write
    SPI2_Transfer(len);       // packet length byte
    for (uint8_t i = 0; i < len; i++)
        SPI2_Transfer(data[i]);
    CC1101_CS_HIGH();
    SPI2_SetLSB();

    // Start TX
    CC1101_Strobe(CC1101_STX);

    // Wait TX complete via GDO0
    uint32_t t0 = HAL_GetTick();
    while (CC1101_GDO0() == GPIO_PIN_RESET)
        if ((HAL_GetTick()-t0) > 500) return CC1101_FAIL;
    while (CC1101_GDO0() == GPIO_PIN_SET)
        if ((HAL_GetTick()-t0) > 1000) return CC1101_FAIL;

    CC1101_Strobe(CC1101_SIDLE);
    return CC1101_OK;
}

// ════════════════════════════════════════════════════
// CC1101_ReceiveData
// ════════════════════════════════════════════════════
uint8_t CC1101_ReceiveData(uint8_t *data, uint8_t *len,
                            uint32_t timeout_ms)
{
    *len = 0;

    // Put radio in RX mode — do NOT reset each call
    // Only reset FIFO if overflow detected
    uint8_t marcState = CC1101_ReadStatus(0xF5) & 0x1F;
    if (marcState == 17 || marcState == 18) {
        // RXFIFO overflow — flush and restart
        CC1101_Strobe(CC1101_SIDLE);
        HAL_Delay(1);
        CC1101_Strobe(CC1101_SFRX);
        HAL_Delay(1);
    }
    CC1101_Strobe(CC1101_SRX);

    // Wait for GDO0 to go HIGH (packet received)
    uint32_t t0 = HAL_GetTick();
    while (CC1101_GDO0() == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > timeout_ms)
            return CC1101_FAIL; // timeout — no signal
        HAL_Delay(1);
    }

    // Wait for GDO0 to go LOW (end of packet)
    t0 = HAL_GetTick();
    while (CC1101_GDO0() == GPIO_PIN_SET) {
        if ((HAL_GetTick() - t0) > 500) break;
    }
    HAL_Delay(2); // let FIFO settle

    // Check bytes in RX FIFO
    uint8_t rxBytes = CC1101_ReadStatus(CC1101_RXBYTES) & 0x7F;
    if (rxBytes == 0 || rxBytes > CC1101_MAX_PAYLOAD + 3) {
        CC1101_Strobe(CC1101_SFRX);
        return CC1101_FAIL;
    }

    // Read from RX FIFO
    SPI2_SetMSB();
    CC1101_CS_LOW();
    CC1101_WaitMISO();
    SPI2_Transfer(0xFF); // RX FIFO burst read
    uint8_t pktLen = SPI2_Transfer(0x00); // first byte = length
    if (pktLen == 0 || pktLen > CC1101_MAX_PAYLOAD)
        pktLen = rxBytes - 1;
    for (uint8_t i = 0; i < pktLen; i++)
        data[i] = SPI2_Transfer(0x00);
    CC1101_CS_HIGH();
    SPI2_SetLSB();

    *len = pktLen;
    return CC1101_OK;
}

// ════════════════════════════════════════════════════
// CC1101_SetIdle
// ════════════════════════════════════════════════════
void CC1101_SetIdle(void)
{
    CC1101_Strobe(CC1101_SIDLE);
}
