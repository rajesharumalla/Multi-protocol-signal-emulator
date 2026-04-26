/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IR Signal Recorder with Button Control - FIXED VERSION
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "lcd_i2c.h"
#include "keypad.h"
#include "pn532_spi.h"
#include "cc1101.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart2;

typedef enum {
	STATE_BOOT, STATE_MAIN_MENU,

	// IR
	STATE_IR_MENU,
	STATE_RECORD,
	STATE_TRANSMIT,
	STATE_IR_SAVE,
	STATE_IR_PLAY_SELECT,
	STATE_IR_CONFIRM_OVERWRITE,

	// RF
	STATE_RF_MENU,

	// RFID / NFC
	STATE_RFID_MENU,
	STATE_RFID_NFC_MENU,
	STATE_NFC_MENU,
	STATE_NFC_RECORD,
	STATE_NFC_SAVE,
	STATE_NFC_CLONE_SELECT,
	STATE_NFC_CLONE_WRITE,
	STATE_NFC_EMIT_SELECT,
	STATE_NFC_EMIT_ACTIVE

} SystemState;

SystemState currentState = STATE_BOOT;
char lastKey = 0;
SystemState previousState = STATE_BOOT;
uint32_t selected_slot_address = 0;
char selected_slot_name = 0;

// IR Signal Storage
#define MAX_IR_PULSES 200
uint32_t ir_pulses[MAX_IR_PULSES];
uint16_t ir_pulse_count = 0;
volatile uint8_t is_recording = 0;
volatile uint8_t is_transmitting = 0;
volatile uint8_t ir_session_active = 0;
volatile uint8_t ir_frame_done = 0;
uint32_t ir_session_start = 0;

#define IR_RECORD_TIMEOUT 5000   // 5 seconds (ms)

// Input Capture Variables
uint32_t capture_val1 = 0;
uint32_t capture_val2 = 0;
uint8_t edge_count = 0;
uint32_t last_edge_time = 0;

// Button Detection Variables
#define DOUBLE_CLICK_TIMEOUT 300  // ms
volatile uint32_t button_press_time = 0;
volatile uint8_t button_press_count = 0;
volatile uint8_t button_event = 0; // 0=none, 1=single, 2=double
volatile uint32_t last_button_time = 0;

// ================= FLASH IR STORAGE =================

#define FLASH_SECTOR_IR        FLASH_SECTOR_7
#define FLASH_IR_BASE_ADDR     0x08060000

#define IR_SLOT_SIZE           4096
#define IR_SLOT_A_ADDR         (FLASH_IR_BASE_ADDR + 0x0000)
#define IR_SLOT_B_ADDR         (FLASH_IR_BASE_ADDR + 0x1000)
#define IR_SLOT_C_ADDR         (FLASH_IR_BASE_ADDR + 0x2000)

#define IR_VALID_FLAG          0xAABBCCDD

typedef struct {
	uint32_t valid;
	uint16_t length;
	uint32_t pulses[MAX_IR_PULSES];
} IR_FlashSlot;

// ================= FLASH NFC STORAGE =================

#define FLASH_SECTOR_NFC       FLASH_SECTOR_6
#define FLASH_NFC_BASE_ADDR    0x08040000

#define NFC_SLOT_SIZE          1024
#define NFC_SLOT_A_ADDR        (FLASH_NFC_BASE_ADDR + 0x0000)
#define NFC_SLOT_B_ADDR        (FLASH_NFC_BASE_ADDR + 0x0400)
#define NFC_SLOT_C_ADDR        (FLASH_NFC_BASE_ADDR + 0x0800)

#define NFC_VALID_FLAG         0x55AA55AA
#define NFC_MAX_NDEF_SIZE      256

typedef struct {
	uint32_t valid;

	uint8_t uid_length;
	uint8_t uid[10];

	uint16_t ndef_length;
	uint8_t ndef_data[NFC_MAX_NDEF_SIZE];

} NFC_FlashSlot;

NFC_FlashSlot nfc_temp_slot;

// ================= FLASH RF STORAGE =================
#define FLASH_SECTOR_RF       FLASH_SECTOR_5
#define FLASH_RF_BASE_ADDR    0x08020000
#define RF_SLOT_A_ADDR        (FLASH_RF_BASE_ADDR + 0x0000)
#define RF_SLOT_B_ADDR        (FLASH_RF_BASE_ADDR + 0x1000)
#define RF_SLOT_C_ADDR        (FLASH_RF_BASE_ADDR + 0x2000)
#define RF_VALID_FLAG         0xBBCCDDEE
#define RF_MAX_PAYLOAD        61

typedef struct {
    uint32_t valid;
    uint8_t  length;
    uint8_t  data[RF_MAX_PAYLOAD];
} RF_FlashSlot;

// ================= FLASH RFID TRUSTED KEYS =================
#define FLASH_SECTOR_RFID_KEYS   FLASH_SECTOR_4
#define FLASH_RFID_KEYS_ADDR     0x08010000
#define RFID_KEYS_VALID_FLAG     0x4B455931
#define RFID_MAX_KEYS            20

typedef struct {
    uint8_t used;
    uint8_t uid_length;
    uint8_t uid[10];
} RFID_KeyEntry;

typedef struct {
    uint32_t valid;
    uint32_t count;
    RFID_KeyEntry entries[RFID_MAX_KEYS];
} RFID_KeyStore;

RFID_KeyStore rfidKeyStore;
uint8_t rfid_last_uid[10];
uint8_t rfid_last_uid_len = 0;
uint8_t rfid_delete_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Flash_Save_NFC(uint32_t address);
uint8_t Flash_Save_NFC_impl(uint32_t address);
void Start_IR_Recording(void);
void Stop_IR_Recording(void);
void Transmit_IR_Signal(void);
void LED_Blink(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t times);
void Flash_Save_IR(uint32_t address);
uint8_t Flash_Load_IR(uint32_t address);
void Flash_Save_RF(uint32_t address, uint8_t *data, uint8_t len);
uint8_t Flash_Load_RF(uint32_t address, uint8_t *data, uint8_t *len);
void Flash_Save_RFID_Keys(void);
void Flash_Load_RFID_Keys(void);
void ShowWelcomeScreens(void);
void AuthenticateOnBoot(void);
void LCD_PrintCentered(uint8_t row, const char *text);
void PlayAccessGrantedAnimation(uint8_t slotIndex);
void PlayAccessDeniedAnimation(void);
void Format_UID_String(char *out, uint32_t outSize, const uint8_t *uid, uint8_t uidLen);
uint8_t RFID_FindKey(const uint8_t *uid, uint8_t uidLen);
uint8_t RFID_AddKey(const uint8_t *uid, uint8_t uidLen);
void RFID_DeleteKey(uint8_t index);
void ShowDeleteScreen(uint8_t index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
    return ch;
}
#endif

void LCD_PrintCentered(uint8_t row, const char *text)
{
	uint8_t len = (uint8_t)strlen(text);
	uint8_t col = 0;

	if (len < 16) {
		col = (16 - len) / 2;
	}

	LCD_Set_Cursor(row, col);
	LCD_Send_String((char*)text);
}

void Format_UID_String(char *out, uint32_t outSize, const uint8_t *uid, uint8_t uidLen)
{
	uint32_t pos = 0;
	if (outSize == 0) {
		return;
	}

	out[0] = '\0';
	for (uint8_t i = 0; i < uidLen && pos + 2 < outSize; i++) {
		pos += snprintf(&out[pos], outSize - pos, "%02X", uid[i]);
	}
}

uint8_t RFID_FindKey(const uint8_t *uid, uint8_t uidLen)
{
	for (uint8_t i = 0; i < RFID_MAX_KEYS; i++) {
		if (rfidKeyStore.entries[i].used &&
		    rfidKeyStore.entries[i].uid_length == uidLen &&
		    memcmp(rfidKeyStore.entries[i].uid, uid, uidLen) == 0) {
			return i;
		}
	}

	return 0xFF;
}

uint8_t RFID_AddKey(const uint8_t *uid, uint8_t uidLen)
{
	if (uidLen == 0 || uidLen > sizeof(rfidKeyStore.entries[0].uid)) {
		return 0xFF;
	}

	uint8_t existing = RFID_FindKey(uid, uidLen);
	if (existing != 0xFF) {
		return existing;
	}

	for (uint8_t i = 0; i < RFID_MAX_KEYS; i++) {
		if (!rfidKeyStore.entries[i].used) {
			memset(&rfidKeyStore.entries[i], 0, sizeof(RFID_KeyEntry));
			rfidKeyStore.entries[i].used = 1;
			rfidKeyStore.entries[i].uid_length = uidLen;
			memcpy(rfidKeyStore.entries[i].uid, uid, uidLen);
			rfidKeyStore.count++;
			Flash_Save_RFID_Keys();
			return i;
		}
	}

	return 0xFF;
}

void RFID_DeleteKey(uint8_t index)
{
	if (index >= RFID_MAX_KEYS || !rfidKeyStore.entries[index].used) {
		return;
	}

	memset(&rfidKeyStore.entries[index], 0, sizeof(RFID_KeyEntry));
	if (rfidKeyStore.count > 0) {
		rfidKeyStore.count--;
	}
	Flash_Save_RFID_Keys();
}

void ShowDeleteScreen(uint8_t index)
{
	char line0[17];

	LCD_Clear();
	LCD_Set_Cursor(0, 0);
	if (rfidKeyStore.entries[index].used) {
		snprintf(line0, sizeof(line0), "Key %02d Used", index + 1);
	} else {
		snprintf(line0, sizeof(line0), "Key %02d Empty", index + 1);
	}
	LCD_Send_String(line0);
	LCD_Set_Cursor(1, 0);
	LCD_Send_String("1Del 2< 8> DBack");
}

void ShowWelcomeScreens(void)
{
	uint8_t irChar[8] = {0x04, 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x00};
	uint8_t rfChar[8] = {0x04, 0x06, 0x0F, 0x04, 0x04, 0x0E, 0x0E, 0x00};
	uint8_t rfidChar[8] = {0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x11, 0x0E, 0x00};
	uint8_t nfcChar[8] = {0x0E, 0x11, 0x15, 0x11, 0x15, 0x11, 0x0E, 0x00};

	LCD_Create_Char(0, irChar);
	LCD_Create_Char(1, rfChar);
	LCD_Create_Char(2, rfidChar);
	LCD_Create_Char(3, nfcChar);

	LCD_Clear();
	LCD_PrintCentered(0, "Hi Welcome To");
	LCD_PrintCentered(1, "Key Unifier");
	HAL_Delay(5000);

	LCD_Clear();
	LCD_PrintCentered(0, "Multi-Protocol");
	LCD_PrintCentered(1, "Signal Emulator");
	HAL_Delay(2000);

	LCD_Clear();
	LCD_Set_Cursor(0, 1);
	LCD_Send_Char(0);
	LCD_Send_String("  ");
	LCD_Send_Char(1);
	LCD_Send_String("  ");
	LCD_Send_Char(2);
	LCD_Send_String("  ");
	LCD_Send_Char(3);
	LCD_Set_Cursor(1, 1);
	LCD_Send_String("IR RF ID NFC ");
	HAL_Delay(2000);
}

void AuthenticateOnBoot(void)
{
	uint8_t uid[10];
	uint8_t uidLen = 0;

	while (1) {
		LCD_Clear();
		LCD_PrintCentered(0, "Tap Trusted Key");
		LCD_PrintCentered(1, "To Continue");

		while (1) {
			char key = Keypad_GetKey();
			if (key == '*') {
				LCD_Clear();
				LCD_PrintCentered(0, "Bypass Mode");
				LCD_PrintCentered(1, "Opening Menu");
				HAL_Delay(1000);
				return;
			}

			if (PN532_SPI_ReadUID(uid, &uidLen)) {
				uint8_t match = RFID_FindKey(uid, uidLen);
				if (match != 0xFF) {
					PlayAccessGrantedAnimation(match);
					return;
				}

				PlayAccessDeniedAnimation();
				LCD_Clear();
				LCD_PrintCentered(0, "Use Trusted Key");
				LCD_PrintCentered(1, "Or Press *");
				HAL_Delay(1200);
				break;
			}

			HAL_Delay(80);
		}
	}
}

void PlayAccessGrantedAnimation(uint8_t slotIndex)
{
	char slotMsg[17];

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	LCD_Clear();
	LCD_PrintCentered(0, "Checking Key");
	LCD_PrintCentered(1, "Please Wait");
	HAL_Delay(180);

	for (uint8_t i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access");
		LCD_PrintCentered(1, "Approved");
		HAL_Delay(140);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access");
		LCD_PrintCentered(1, "Approved.");
		HAL_Delay(140);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access");
		LCD_PrintCentered(1, "Approved..");
		HAL_Delay(140);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access");
		LCD_PrintCentered(1, "Approved...");
		HAL_Delay(160);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	LCD_Clear();
	LCD_PrintCentered(0, "Access Granted");
	snprintf(slotMsg, sizeof(slotMsg), "Trusted Key %02d", slotIndex + 1);
	LCD_PrintCentered(1, slotMsg);
	HAL_Delay(1200);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
}

void PlayAccessDeniedAnimation(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	for (uint8_t i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access Denied");
		LCD_PrintCentered(1, "Wrong Key");
		HAL_Delay(180);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
		LCD_Clear();
		LCD_PrintCentered(0, "Access Denied");
		HAL_Delay(120);
	}
}

void Flash_Load_RFID_Keys(void)
{
	RFID_KeyStore *stored = (RFID_KeyStore*)FLASH_RFID_KEYS_ADDR;
	if (stored->valid == RFID_KEYS_VALID_FLAG) {
		memcpy(&rfidKeyStore, stored, sizeof(RFID_KeyStore));
	} else {
		memset(&rfidKeyStore, 0, sizeof(rfidKeyStore));
		rfidKeyStore.valid = RFID_KEYS_VALID_FLAG;
	}
}

void Flash_Save_RFID_Keys(void)
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erase;
	uint32_t sectorError;
	uint32_t *data = (uint32_t*)&rfidKeyStore;

	rfidKeyStore.valid = RFID_KEYS_VALID_FLAG;

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Sector = FLASH_SECTOR_RFID_KEYS;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&erase, &sectorError);

	for (uint32_t i = 0; i < sizeof(RFID_KeyStore) / 4; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		                  FLASH_RFID_KEYS_ADDR + (i * 4), data[i]);
	}

	HAL_FLASH_Lock();
}

void DrawScreen(SystemState state) {
	LCD_Clear();

	switch (state) {
	case STATE_MAIN_MENU:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1.IR   2.RF");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("3.RFID/NFC");
		break;
	case STATE_IR_SAVE:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Save Slot?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("A B C  D:Back");
		break;

	case STATE_IR_PLAY_SELECT:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Play Slot?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("A B C  D:Back");
		break;

	case STATE_IR_MENU:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1.Record");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("2.Transmit D:Back");
		break;

	case STATE_RF_MENU:
	    LCD_Set_Cursor(0, 0);
	    LCD_Send_String("1.Record 2.Emit");
	    LCD_Set_Cursor(1, 0);
	    LCD_Send_String("D:Back");
	    break;

	case STATE_RFID_MENU:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1.Add 2.Verify");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("3.Delete D:Back");
		break;
	case STATE_NFC_MENU:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1.Record 2.Clone");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("3.Emit  D:Back");
		break;

	case STATE_TRANSMIT:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1:Send");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("D:Back");
		break;

	case STATE_RECORD:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Recording...");
		break;
	case STATE_IR_CONFIRM_OVERWRITE:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Overwrite ");
		LCD_Send_Char(selected_slot_name);
		LCD_Send_String("?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("1.Yes 2.No");
		break;
	case STATE_RFID_NFC_MENU:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("1.RFID 2.NFC");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("D:Back");
		break;
	case STATE_NFC_RECORD:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Scan NFC Tag");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("D:Cancel");
		break;

	case STATE_NFC_SAVE:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Save Slot?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("A B C  D:Back");
		break;

	case STATE_NFC_CLONE_SELECT:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Clone From?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("A B C  D:Back");
		break;

	case STATE_NFC_CLONE_WRITE:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Place BlankTag");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("Writing...");
		break;

	case STATE_NFC_EMIT_SELECT:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Emit Slot?");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("A B C  D:Back");
		break;

	case STATE_NFC_EMIT_ACTIVE:
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Emulating...");
		LCD_Set_Cursor(1, 0);
		LCD_Send_String("D:Stop");
		break;

	default:
		break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Enable DWT cycle counter for IR timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  LCD_Init();   // ✅ MUST BE FIRST
  HAL_Delay(50);
  PN532_SPI_Init();
  HAL_Delay(50);
  uint8_t pn532Ready = PN532_SPI_GetFirmware();
  uint8_t cc1101Ready = CC1101_Init();
  Flash_Load_RFID_Keys();

  if (!pn532Ready || !cc1101Ready) {
	  LCD_Clear();
	  LCD_Set_Cursor(0, 0);
	  LCD_Send_String("Module Init Fail");
	  LCD_Set_Cursor(1, 0);
	  if (!pn532Ready && !cc1101Ready) {
		  LCD_Send_String("PN532 CC1101");
	  } else if (!pn532Ready) {
		  LCD_Send_String("Check PN532");
	  } else {
		  LCD_Send_String("Check CC1101");
	  }
	  HAL_Delay(2500);
  }

  if (pn532Ready) {
	  AuthenticateOnBoot();
  }

  ShowWelcomeScreens();

	DrawScreen(STATE_MAIN_MENU);
	currentState = STATE_MAIN_MENU;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* USER CODE BEGIN WHILE */
	  while (1)
	  {
	    /* USER CODE END WHILE */
	    /* USER CODE BEGIN 3 */

	    char key = Keypad_GetKey();

	    // ══════════════════════════════════════════
	    // MAIN MENU
	    // ══════════════════════════════════════════
	    if (currentState == STATE_MAIN_MENU)
	    {
	        if (key == '1') {
	            currentState = STATE_IR_MENU;
	            DrawScreen(STATE_IR_MENU);
	        }
	        else if (key == '2') {
	            currentState = STATE_RF_MENU;
	            DrawScreen(STATE_RF_MENU);
	        }
	        else if (key == '3') {
	            currentState = STATE_RFID_NFC_MENU;
	            DrawScreen(STATE_RFID_NFC_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // IR MENU  —  1.Record  2.Emit  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_IR_MENU)
	    {
	        if (key == '1') {
	            // Start 10-second IR recording
	            ir_pulse_count = 0;
	            edge_count = 0;
	            last_edge_time = 0;
	            is_recording = 1;
	            ir_session_active = 1;
	            ir_frame_done = 0;
	            ir_session_start = HAL_GetTick();
	            HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("Recording IR...");

	            uint8_t signalCaptured = 0;
	            uint32_t lastPulseTime = HAL_GetTick();

	            // 10 second recording window
	            while ((HAL_GetTick() - ir_session_start) < 10000)
	            {
	                // Show countdown
	                uint32_t elapsed = HAL_GetTick() - ir_session_start;
	                uint8_t secsLeft = 10 - (elapsed / 1000);
	                LCD_Set_Cursor(1, 0);
	                char buf[17];
	                sprintf(buf, "D:Stop    [%2ds]", secsLeft);
	                LCD_Send_String(buf);

	                // If pulses received, track last pulse time
	                if (ir_pulse_count > 0) {
	                    signalCaptured = 1;
	                    lastPulseTime = HAL_GetTick();
	                }

	                // Auto-stop 500ms after last pulse (signal complete)
	                if (signalCaptured &&
	                    (HAL_GetTick() - lastPulseTime) > 500 &&
	                    ir_pulse_count > 10)
	                {
	                    break;
	                }

	                // Manual stop
	                char k = Keypad_GetKey();
	                if (k == 'D') {
	                    signalCaptured = 0; // cancelled
	                    break;
	                }

	                HAL_Delay(50);
	            }

	            // Stop recording
	            is_recording = 0;
	            ir_session_active = 0;
	            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	            if (signalCaptured && ir_pulse_count > 10)
	            {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Signal Got!");
	                LCD_Set_Cursor(1, 0);
	                char info[17];
	                sprintf(info, "%d pulses", ir_pulse_count);
	                LCD_Send_String(info);
	                HAL_Delay(1500);
	                currentState = STATE_IR_SAVE;
	                DrawScreen(STATE_IR_SAVE);
	            }
	            else
	            {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("No Signal!");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("Please Try Again");
	                HAL_Delay(2000);
	                currentState = STATE_IR_MENU;
	                DrawScreen(STATE_IR_MENU);
	            }
	        }
	        else if (key == '2') {
	            currentState = STATE_IR_PLAY_SELECT;
	            DrawScreen(STATE_IR_PLAY_SELECT);
	        }
	        else if (key == 'D') {
	            currentState = STATE_MAIN_MENU;
	            DrawScreen(STATE_MAIN_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // IR SAVE  —  A/B/C  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_IR_SAVE)
	    {
	        if (key == 'A' || key == 'B' || key == 'C') {
	            selected_slot_address = (key == 'A') ? IR_SLOT_A_ADDR :
	                                    (key == 'B') ? IR_SLOT_B_ADDR :
	                                                   IR_SLOT_C_ADDR;
	            selected_slot_name = key;

	            // Check if slot already has data
	            IR_FlashSlot *existing = (IR_FlashSlot*)selected_slot_address;
	            if (existing->valid == IR_VALID_FLAG) {
	                // Ask overwrite confirmation
	                currentState = STATE_IR_CONFIRM_OVERWRITE;
	                DrawScreen(STATE_IR_CONFIRM_OVERWRITE);
	            } else {
	                // Empty slot — save directly
	                Flash_Save_IR(selected_slot_address);
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Saved to Slot");
	                LCD_Set_Cursor(1, 0);
	                char msg[4] = {selected_slot_name, '!', 0};
	                LCD_Send_String(msg);
	                HAL_Delay(1500);
	                currentState = STATE_IR_MENU;
	                DrawScreen(STATE_IR_MENU);
	            }
	        }
	        else if (key == 'D') {
	            currentState = STATE_IR_MENU;
	            DrawScreen(STATE_IR_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // IR CONFIRM OVERWRITE  —  1.Yes  2.No
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_IR_CONFIRM_OVERWRITE)
	    {
	        if (key == '1') {
	            Flash_Save_IR(selected_slot_address);
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("Overwritten!");
	            LCD_Set_Cursor(1, 0);
	            LCD_Send_String("Slot ");
	            LCD_Send_Char(selected_slot_name);
	            HAL_Delay(1500);
	            currentState = STATE_IR_MENU;
	            DrawScreen(STATE_IR_MENU);
	        }
	        else if (key == '2') {
	            currentState = STATE_IR_SAVE;
	            DrawScreen(STATE_IR_SAVE);
	        }
	        else if (key == 'D') {
	            currentState = STATE_IR_MENU;
	            DrawScreen(STATE_IR_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // IR EMIT SELECT  —  A/B/C  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_IR_PLAY_SELECT)
	    {
	        if (key == 'A' || key == 'B' || key == 'C') {
	            uint32_t addr = (key == 'A') ? IR_SLOT_A_ADDR :
	                            (key == 'B') ? IR_SLOT_B_ADDR :
	                                           IR_SLOT_C_ADDR;

	            if (Flash_Load_IR(addr)) {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Emitting...");
	                LCD_Set_Cursor(1, 0);
	                char info[20];
	                sprintf(info, "%c: %d pulses", key, ir_pulse_count);
	                LCD_Send_String(info);
	                HAL_Delay(500);
	                Transmit_IR_Signal();
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Done!");
	                HAL_Delay(1000);
	                DrawScreen(STATE_IR_PLAY_SELECT);
	            } else {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Slot ");
	                LCD_Send_Char(key);
	                LCD_Send_String(" Empty!");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("Nothing saved");
	                HAL_Delay(2000);
	                DrawScreen(STATE_IR_PLAY_SELECT);
	            }
	        }
	        else if (key == 'D') {
	            currentState = STATE_IR_MENU;
	            DrawScreen(STATE_IR_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // RF MENU
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_RF_MENU)
	    {
	        if (key == '1') {
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("RF Recording...");
	            uint8_t rfData[RF_MAX_PAYLOAD];
	            uint8_t rfLen = 0;
	            uint8_t rfCaptured = 0;
	            uint32_t rfStart = HAL_GetTick();
	            CC1101_Strobe(CC1101_SRX); // start listening before loop

	            while ((HAL_GetTick() - rfStart) < 10000)
	            {
	                uint8_t secsLeft = 10 - ((HAL_GetTick()-rfStart)/1000);
	                LCD_Set_Cursor(1, 0);
	                char buf[17];
	                sprintf(buf, "D:Stop    [%2ds]", secsLeft);
	                LCD_Send_String(buf);

	                char k = Keypad_GetKey();
	                if (k == 'D') break;

	                if (CC1101_ReceiveData(rfData, &rfLen, 500) == CC1101_OK
	                    && rfLen > 0)
	                {
	                    rfCaptured = 1;
	                    LCD_Clear();
	                    LCD_Set_Cursor(0, 0);
	                    LCD_Send_String("RF Signal Got!");
	                    LCD_Set_Cursor(1, 0);
	                    char info[17];
	                    sprintf(info, "%d bytes", rfLen);
	                    LCD_Send_String(info);
	                    HAL_Delay(1500);
	                    break;
	                }
	            }
	            CC1101_SetIdle();

	            if (rfCaptured) {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Save RF Slot?");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("A B C  D:Discard");
	                while (1) {
	                    char k = Keypad_GetKey();
	                    if (k == 'A' || k == 'B' || k == 'C') {
	                        uint32_t addr = (k=='A') ? RF_SLOT_A_ADDR :
	                                        (k=='B') ? RF_SLOT_B_ADDR :
	                                                   RF_SLOT_C_ADDR;
	                        Flash_Save_RF(addr, rfData, rfLen);
	                        LCD_Clear();
	                        LCD_Set_Cursor(0, 0);
	                        LCD_Send_String("RF Saved!");
	                        LCD_Set_Cursor(1, 0);
	                        char msg[3] = {k, '!', 0};
	                        LCD_Send_String(msg);
	                        HAL_Delay(1500);
	                        break;
	                    } else if (k == 'D') break;
	                }
	            } else {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("No RF Signal!");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("Try Again");
	                HAL_Delay(2000);
	            }
	            DrawScreen(STATE_RF_MENU);
	        }
	        else if (key == '2') {
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("Emit RF Slot?");
	            LCD_Set_Cursor(1, 0);
	            LCD_Send_String("A B C  D:Back");
	            while (1) {
	                char k = Keypad_GetKey();
	                if (k == 'A' || k == 'B' || k == 'C') {
	                    uint32_t addr = (k=='A') ? RF_SLOT_A_ADDR :
	                                    (k=='B') ? RF_SLOT_B_ADDR :
	                                               RF_SLOT_C_ADDR;
	                    uint8_t rfData[RF_MAX_PAYLOAD];
	                    uint8_t rfLen = 0;
	                    if (Flash_Load_RF(addr, rfData, &rfLen)) {
	                        LCD_Clear();
	                        LCD_Set_Cursor(0, 0);
	                        LCD_Send_String("Sending RF...");
	                        LCD_Set_Cursor(1, 0);
	                        char info[17];
	                        sprintf(info, "Slot %c %dbytes", k, rfLen);
	                        LCD_Send_String(info);
	                        HAL_Delay(300);
	                        if (CC1101_SendData(rfData, rfLen)==CC1101_OK){
	                            LCD_Clear();
	                            LCD_Set_Cursor(0, 0);
	                            LCD_Send_String("RF Sent! :)");
	                        } else {
	                            LCD_Clear();
	                            LCD_Set_Cursor(0, 0);
	                            LCD_Send_String("Send Failed :(");
	                        }
	                        HAL_Delay(1500);
	                    } else {
	                        LCD_Clear();
	                        LCD_Set_Cursor(0, 0);
	                        LCD_Send_String("Slot Empty!");
	                        HAL_Delay(1500);
	                    }
	                    break;
	                } else if (k == 'D') break;
	            }
	            DrawScreen(STATE_RF_MENU);
	        }
	        else if (key == 'D') {
	            currentState = STATE_MAIN_MENU;
	            DrawScreen(STATE_MAIN_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // RFID/NFC MENU  —  1.RFID  2.NFC  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_RFID_NFC_MENU)
	    {
	        if (key == '1') {
	            currentState = STATE_RFID_MENU;
	            DrawScreen(STATE_RFID_MENU);
	        }
	        else if (key == '2') {
	            currentState = STATE_NFC_MENU;
	            DrawScreen(STATE_NFC_MENU);
	        }
	        else if (key == 'D') {
	            currentState = STATE_MAIN_MENU;
	            DrawScreen(STATE_MAIN_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // NFC MENU  —  1.Record  2.Clone  3.Emit  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_NFC_MENU)
	    {
	        if (key == '1') {
	            currentState = STATE_NFC_RECORD;
	            // Start NFC scan immediately
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("Scanning NFC...");

	            uint8_t uid[7];
	            uint8_t uidLen = 0;
	            uint8_t cardFound = 0;
	            uint32_t scanStart = HAL_GetTick();

	            while ((HAL_GetTick() - scanStart) < 10000)
	            {
	                uint32_t elapsed = HAL_GetTick() - scanStart;
	                uint8_t secsLeft = 10 - (elapsed / 1000);
	                LCD_Set_Cursor(1, 0);
	                char buf[17];
	                sprintf(buf, "D:Cancel  [%2ds]", secsLeft);
	                LCD_Send_String(buf);

	                char k = Keypad_GetKey();
	                if (k == 'D') break;

	                if (PN532_SPI_ReadUID(uid, &uidLen))
	                                {
	                                    cardFound = 1;
	                                    memset(&nfc_temp_slot, 0, sizeof(nfc_temp_slot));
	                                    nfc_temp_slot.uid_length = uidLen;
	                                    memcpy(nfc_temp_slot.uid, uid, uidLen);
	                                    nfc_temp_slot.valid = NFC_VALID_FLAG;

	                                    // Show UID
	                                    LCD_Clear();
	                                    LCD_Set_Cursor(0, 0);
	                                    LCD_Send_String("Card Found!");
	                                    LCD_Set_Cursor(1, 0);
	                                    char uidStr[17] = {0};
	                                    for (int i = 0; i < uidLen && i < 6; i++) {
	                                        char b[3];
	                                        sprintf(b, "%02X", uid[i]);
	                                        strcat(uidStr, b);
	                                    }
	                                    LCD_Send_String(uidStr);
	                                    HAL_Delay(1500);

	                                    LCD_Clear();
	                                    LCD_Set_Cursor(0, 0);
	                                    LCD_Send_String("Reading NDEF...");
	                                    uint16_t ndefLen = 0;
	                                    if (PN532_SPI_ReadNDEF_Mifare(uid,
	                                            nfc_temp_slot.ndef_data, &ndefLen)) {
	                                        nfc_temp_slot.ndef_length = ndefLen;
	                                        LCD_Clear();
	                                        LCD_Set_Cursor(0, 0);
	                                        LCD_Send_String("NDEF Read OK!");
	                                        LCD_Set_Cursor(1, 0);
	                                        char info[20];
	                                        sprintf(info, "%d bytes found", ndefLen);
	                                        LCD_Send_String(info);
	                                    } else {
	                                        nfc_temp_slot.ndef_length = 0;
	                                        LCD_Clear();
	                                        LCD_Set_Cursor(0, 0);
	                                        LCD_Send_String("UID saved only");
	                                        LCD_Set_Cursor(1, 0);
	                                        LCD_Send_String("No NDEF found");
	                                    }
	                                    HAL_Delay(2000);
	                                    currentState = STATE_NFC_SAVE;
	                                    DrawScreen(STATE_NFC_SAVE);
	                                    break;
	                                }
	                HAL_Delay(50);
	            }

	            if (!cardFound && currentState == STATE_NFC_RECORD)
	            {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("No Card Found");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("Try Again");
	                HAL_Delay(2000);
	                currentState = STATE_NFC_MENU;
	                DrawScreen(STATE_NFC_MENU);
	            }
	        }
	        else if (key == '2') {
	            currentState = STATE_NFC_CLONE_SELECT;
	            DrawScreen(STATE_NFC_CLONE_SELECT);
	        }
	        else if (key == '3') {
	            currentState = STATE_NFC_EMIT_SELECT;
	            DrawScreen(STATE_NFC_EMIT_SELECT);
	        }
	        else if (key == 'D') {
	            currentState = STATE_RFID_NFC_MENU;
	            DrawScreen(STATE_RFID_NFC_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // NFC SAVE  —  A/B/C  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_NFC_SAVE)
	    {
	        if (key == 'A' || key == 'B' || key == 'C') {
	            uint32_t addr = (key == 'A') ? NFC_SLOT_A_ADDR :
	                            (key == 'B') ? NFC_SLOT_B_ADDR :
	                                           NFC_SLOT_C_ADDR;
	            nfc_temp_slot.valid = NFC_VALID_FLAG;
	            Flash_Save_NFC(addr);
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("NFC Saved!");
	            LCD_Set_Cursor(1, 0);
	            char msg[3] = {'A' + (key-'A'), '!', 0};
	            LCD_Send_String(msg);
	            HAL_Delay(1500);
	            currentState = STATE_NFC_MENU;
	            DrawScreen(STATE_NFC_MENU);
	        }
	        else if (key == 'D') {
	            currentState = STATE_NFC_MENU;
	            DrawScreen(STATE_NFC_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // NFC EMIT SELECT  —  A/B/C  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_NFC_EMIT_SELECT)
	    {
	        if (key == 'A' || key == 'B' || key == 'C') {
	            uint32_t addr = (key == 'A') ? NFC_SLOT_A_ADDR :
	                            (key == 'B') ? NFC_SLOT_B_ADDR :
	                                           NFC_SLOT_C_ADDR;
	            NFC_FlashSlot *slot = (NFC_FlashSlot*)addr;
	            if (slot->valid == NFC_VALID_FLAG) {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Emulating NFC");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("D:Stop");
	                // Emit loop — keep re-broadcasting UID
	                while (1) {
	                    char k = Keypad_GetKey();
	                    if (k == 'D') break;
	                    PN532_SPI_EmulateTag(slot->uid, slot->uid_length,
	                                         slot->ndef_data, slot->ndef_length);
	                    HAL_Delay(100);
	                }
	                currentState = STATE_NFC_MENU;
	                DrawScreen(STATE_NFC_MENU);
	            } else {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Slot Empty!");
	                HAL_Delay(2000);
	                DrawScreen(STATE_NFC_EMIT_SELECT);
	            }
	        }
	        else if (key == 'D') {
	            currentState = STATE_NFC_MENU;
	            DrawScreen(STATE_NFC_MENU);
	        }
	    }

	    // ══════════════════════════════════════════
	    // NFC CLONE SELECT  —  A/B/C  D.Back
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_NFC_CLONE_SELECT)
	    {
	        if (key == 'A' || key == 'B' || key == 'C') {
	            uint32_t addr = (key == 'A') ? NFC_SLOT_A_ADDR :
	                            (key == 'B') ? NFC_SLOT_B_ADDR :
	                                           NFC_SLOT_C_ADDR;
	            NFC_FlashSlot *slot = (NFC_FlashSlot*)addr;
	            if (slot->valid == NFC_VALID_FLAG) {
	                memcpy(&nfc_temp_slot, slot, sizeof(NFC_FlashSlot));
	                currentState = STATE_NFC_CLONE_WRITE;
	                DrawScreen(STATE_NFC_CLONE_WRITE);
	            } else {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Slot Empty!");
	                HAL_Delay(2000);
	                DrawScreen(STATE_NFC_CLONE_SELECT);
	            }
	        }
	        else if (key == 'D') {
	            currentState = STATE_NFC_MENU;
	            DrawScreen(STATE_NFC_MENU);
	        }
	    }


	    // ══════════════════════════════════════════
	        // NFC CLONE WRITE — place target tag
	        // ══════════════════════════════════════════
	        else if (currentState == STATE_NFC_CLONE_WRITE)
	        {
	            LCD_Clear();
	            LCD_Set_Cursor(0, 0);
	            LCD_Send_String("Place Tag Now");
	            LCD_Set_Cursor(1, 0);
	            LCD_Send_String("D:Cancel");

	            uint8_t tuid[7];
	            uint8_t tuidLen = 0;
	            uint8_t done = 0;
	            uint32_t t0 = HAL_GetTick();

	            while ((HAL_GetTick() - t0) < 15000 && !done)
	            {
	                // Countdown
	                uint8_t secsLeft = 15 - ((HAL_GetTick()-t0)/1000);
	                LCD_Set_Cursor(1, 0);
	                char buf[17];
	                sprintf(buf, "D:Cancel  [%2ds]", secsLeft);
	                LCD_Send_String(buf);

	                char k = Keypad_GetKey();
	                if (k == 'D') {
	                    currentState = STATE_NFC_MENU;
	                    DrawScreen(STATE_NFC_MENU);
	                    done = 1;
	                    break;
	                }

	                if (PN532_SPI_ReadUID(tuid, &tuidLen))
	                {
	                    LCD_Clear();
	                    LCD_Set_Cursor(0, 0);
	                    LCD_Send_String("Tag detected!");
	                    LCD_Set_Cursor(1, 0);
	                    LCD_Send_String("Writing NDEF...");
	                    HAL_Delay(500);

	                    if (nfc_temp_slot.ndef_length > 0) {
	                        if (PN532_SPI_WriteNDEF_Mifare(tuid,
	                                nfc_temp_slot.ndef_data,
	                                nfc_temp_slot.ndef_length)) {
	                            LCD_Clear();
	                            LCD_Set_Cursor(0, 0);
	                            LCD_Send_String("Clone Done! :)");
	                            LCD_Set_Cursor(1, 0);
	                            char info[20];
	                            sprintf(info, "%d written", nfc_temp_slot.ndef_length);
	                            LCD_Send_String(info);
	                        } else {
	                            LCD_Clear();
	                            LCD_Set_Cursor(0, 0);
	                            LCD_Send_String("Write Failed!");
	                            LCD_Set_Cursor(1, 0);
	                            LCD_Send_String("Check tag type");
	                        }
	                    } else {
	                        LCD_Clear();
	                        LCD_Set_Cursor(0, 0);
	                        LCD_Send_String("No NDEF to");
	                        LCD_Set_Cursor(1, 0);
	                        LCD_Send_String("clone! Empty");
	                    }
	                    HAL_Delay(3000);
	                    done = 1;
	                    currentState = STATE_NFC_MENU;
	                    DrawScreen(STATE_NFC_MENU);
	                }
	                HAL_Delay(200);
	            }

	            // Timeout
	            if (!done) {
	                LCD_Clear();
	                LCD_Set_Cursor(0, 0);
	                LCD_Send_String("Timeout!");
	                LCD_Set_Cursor(1, 0);
	                LCD_Send_String("No tag placed");
	                HAL_Delay(2000);
	                currentState = STATE_NFC_MENU;
	                DrawScreen(STATE_NFC_MENU);
	            }
	        }

	    // ══════════════════════════════════════════
	    // RFID MENU (placeholder)
	    // ══════════════════════════════════════════
	    else if (currentState == STATE_RFID_MENU)
	    {
	        if (key == '1') {
	        	uint8_t uid[10];
	        	uint8_t uidLen = 0;
	        	uint8_t cardFound = 0;
	        	uint32_t scanStart = HAL_GetTick();

	        	LCD_Clear();
	        	LCD_Set_Cursor(0, 0);
	        	LCD_Send_String("Scan Key To Add");

	        	while ((HAL_GetTick() - scanStart) < 10000) {
	        		uint8_t secsLeft = 10 - ((HAL_GetTick() - scanStart) / 1000);
	        		char line1[17];
	        		snprintf(line1, sizeof(line1), "D:Cancel [%2ds]", secsLeft);
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String(line1);

	        		char k = Keypad_GetKey();
	        		if (k == 'D') {
	        			break;
	        		}

	        		if (PN532_SPI_ReadUID(uid, &uidLen)) {
	        			cardFound = 1;
	        			memcpy(rfid_last_uid, uid, uidLen);
	        			rfid_last_uid_len = uidLen;
	        			break;
	        		}

	        		HAL_Delay(80);
	        	}

	        	if (!cardFound) {
	        		LCD_Clear();
	        		LCD_Set_Cursor(0, 0);
	        		LCD_Send_String("No Key Scanned");
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String("Try Again");
	        		HAL_Delay(1500);
	        		DrawScreen(STATE_RFID_MENU);
	        	} else {
	        		char uidStr[17];
	        		Format_UID_String(uidStr, sizeof(uidStr), rfid_last_uid, rfid_last_uid_len);
	        		LCD_Clear();
	        		LCD_Set_Cursor(0, 0);
	        		LCD_Send_String("Add Trusted Key?");
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String("1.Yes 2.No");
	        		HAL_Delay(1200);
	        		LCD_Clear();
	        		LCD_Set_Cursor(0, 0);
	        		LCD_Send_String(uidStr);
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String("1.Yes 2.No");

	        		while (1) {
	        			char k = Keypad_GetKey();
	        			if (k == '1') {
	        				uint8_t existing = RFID_FindKey(rfid_last_uid, rfid_last_uid_len);
	        				if (existing != 0xFF) {
	        					LCD_Clear();
	        					LCD_Set_Cursor(0, 0);
	        					LCD_Send_String("Already Trusted");
	        					LCD_Set_Cursor(1, 0);
	        					snprintf(uidStr, sizeof(uidStr), "Key %02d", existing + 1);
	        					LCD_Send_String(uidStr);
	        				} else {
	        					uint8_t added = RFID_AddKey(rfid_last_uid, rfid_last_uid_len);
	        					LCD_Clear();
	        					if (added == 0xFF) {
	        						LCD_Set_Cursor(0, 0);
	        						LCD_Send_String("Key Store Full");
	        						LCD_Set_Cursor(1, 0);
	        						LCD_Send_String("Delete One Key");
	        					} else {
	        						LCD_Set_Cursor(0, 0);
	        						LCD_Send_String("Trusted Saved");
	        						LCD_Set_Cursor(1, 0);
	        						snprintf(uidStr, sizeof(uidStr), "Key %02d Added", added + 1);
	        						LCD_Send_String(uidStr);
	        					}
	        				}
	        				HAL_Delay(1800);
	        				DrawScreen(STATE_RFID_MENU);
	        				break;
	        			} else if (k == '2' || k == 'D') {
	        				DrawScreen(STATE_RFID_MENU);
	        				break;
	        			}
	        		}
	        	}
	        }
	        else if (key == '2') {
	        	uint8_t uid[10];
	        	uint8_t uidLen = 0;
	        	uint8_t cardFound = 0;
	        	uint32_t scanStart = HAL_GetTick();

	        	LCD_Clear();
	        	LCD_Set_Cursor(0, 0);
	        	LCD_Send_String("Tap Key Verify");

	        	while ((HAL_GetTick() - scanStart) < 10000) {
	        		uint8_t secsLeft = 10 - ((HAL_GetTick() - scanStart) / 1000);
	        		char line1[17];
	        		snprintf(line1, sizeof(line1), "D:Cancel [%2ds]", secsLeft);
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String(line1);

	        		char k = Keypad_GetKey();
	        		if (k == 'D') {
	        			break;
	        		}

	        		if (PN532_SPI_ReadUID(uid, &uidLen)) {
	        			cardFound = 1;
	        			break;
	        		}

	        		HAL_Delay(80);
	        	}

	        	if (!cardFound) {
	        		LCD_Clear();
	        		LCD_Set_Cursor(0, 0);
	        		LCD_Send_String("No Key Scanned");
	        		LCD_Set_Cursor(1, 0);
	        		LCD_Send_String("Try Again");
	        		HAL_Delay(1500);
	        		DrawScreen(STATE_RFID_MENU);
	        	} else {
	        		uint8_t match = RFID_FindKey(uid, uidLen);
	        		char uidStr[17];
	        		Format_UID_String(uidStr, sizeof(uidStr), uid, uidLen);
	        		LCD_Clear();
	        		if (match != 0xFF) {
	        			LCD_Set_Cursor(0, 0);
	        			LCD_Send_String("Access Granted");
	        			LCD_Set_Cursor(1, 0);
	        			snprintf(uidStr, sizeof(uidStr), "Trusted %02d", match + 1);
	        			LCD_Send_String(uidStr);
	        		} else {
	        			LCD_Set_Cursor(0, 0);
	        			LCD_Send_String("Unknown Key");
	        			LCD_Set_Cursor(1, 0);
	        			LCD_Send_String(uidStr);
	        		}
	        		HAL_Delay(1800);
	        		DrawScreen(STATE_RFID_MENU);
	        	}
	        }
	        else if (key == '3') {
	        	rfid_delete_index = 0;
	        	ShowDeleteScreen(rfid_delete_index);

	        	while (1) {
	        		char k = Keypad_GetKey();
	        		if (k == '2') {
	        			rfid_delete_index = (rfid_delete_index == 0) ? (RFID_MAX_KEYS - 1) : (rfid_delete_index - 1);
	        			ShowDeleteScreen(rfid_delete_index);
	        		} else if (k == '8') {
	        			rfid_delete_index = (rfid_delete_index + 1) % RFID_MAX_KEYS;
	        			ShowDeleteScreen(rfid_delete_index);
	        		} else if (k == '1') {
	        			if (rfidKeyStore.entries[rfid_delete_index].used) {
	        				RFID_DeleteKey(rfid_delete_index);
	        				LCD_Clear();
	        				LCD_Set_Cursor(0, 0);
	        				LCD_Send_String("Key Deleted");
	        				LCD_Set_Cursor(1, 0);
	        				LCD_Send_String("From Trusted");
	        				HAL_Delay(1500);
	        			} else {
	        				LCD_Clear();
	        				LCD_Set_Cursor(0, 0);
	        				LCD_Send_String("Slot Empty");
	        				LCD_Set_Cursor(1, 0);
	        				LCD_Send_String("Pick Another");
	        				HAL_Delay(1200);
	        			}
	        			ShowDeleteScreen(rfid_delete_index);
	        		} else if (k == 'D') {
	        			DrawScreen(STATE_RFID_MENU);
	        			break;
	        		}
	        	}
	        }
	        else if (key == 'D') {
	            currentState = STATE_RFID_NFC_MENU;
	            DrawScreen(STATE_RFID_NFC_MENU);
	        }
	    }

	  }
	  /* USER CODE END 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PN532_CS_GPIO_Port, PN532_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : user_button_Pin */
  GPIO_InitStruct.Pin = user_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(user_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 = CC1101 GDO0 (Input) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 (I2C2 SCL) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 = CC1101 CS (Output HIGH idle) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PN532_CS_Pin */
  GPIO_InitStruct.Pin = PN532_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PN532_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(OTG_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
 * @brief  Start IR signal recording
 */
void Start_IR_Recording(void) {
	ir_pulse_count = 0;
	edge_count = 0;
	last_edge_time = 0;

	is_recording = 1;
	ir_session_active = 1;
	ir_frame_done = 0;

	ir_session_start = HAL_GetTick();

	// Turn ON orange LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

	LCD_Clear();
	LCD_Set_Cursor(0, 0);
	LCD_Send_String("Recording...");
}

/**
 * @brief  Stop IR signal recording
 */
void Stop_IR_Recording(void) {
	is_recording = 0;
	ir_session_active = 0;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	LCD_Clear();

	if (ir_pulse_count > 10) {
		currentState = STATE_IR_SAVE;
	} else {
		LCD_Set_Cursor(0, 0);
		LCD_Send_String("Not Recorded");
		HAL_Delay(1000);
		currentState = STATE_IR_MENU;
	}

	HAL_Delay(1000);
}

/**
 * @brief  Transmit recorded IR signal
 */
void Transmit_IR_Signal(void) {
	if (ir_pulse_count == 0)
		return;

	is_transmitting = 1;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

	// Keep the timer running continuously and gate the carrier by duty.
	// This is more reliable for IR TX modules on PA6/TIM3_CH1 than
	// repeatedly starting and stopping PWM on every short pulse.
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	for (uint16_t i = 0; i < ir_pulse_count; i++) {
		if (i % 2 == 0) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 13);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		}

		uint32_t start = DWT->CYCCNT;
		uint32_t cycles = ir_pulses[i] * (SystemCoreClock / 1000000);

		while ((DWT->CYCCNT - start) < cycles)
			;
	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	is_transmitting = 0;
}

void Flash_Save_IR(uint32_t address) {
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erase;
	uint32_t sectorError;

	// Temporary buffers for all 3 slots
	IR_FlashSlot slotA;
	IR_FlashSlot slotB;
	IR_FlashSlot slotC;

	// Load existing slots
	memcpy(&slotA, (void*) IR_SLOT_A_ADDR, sizeof(IR_FlashSlot));
	memcpy(&slotB, (void*) IR_SLOT_B_ADDR, sizeof(IR_FlashSlot));
	memcpy(&slotC, (void*) IR_SLOT_C_ADDR, sizeof(IR_FlashSlot));

	// Prepare new slot data
	IR_FlashSlot newSlot;
	newSlot.valid = IR_VALID_FLAG;
	newSlot.length = ir_pulse_count;

	for (uint16_t i = 0; i < ir_pulse_count; i++)
		newSlot.pulses[i] = ir_pulses[i];

	// Replace only selected slot
	if (address == IR_SLOT_A_ADDR)
		memcpy(&slotA, &newSlot, sizeof(IR_FlashSlot));
	else if (address == IR_SLOT_B_ADDR)
		memcpy(&slotB, &newSlot, sizeof(IR_FlashSlot));
	else if (address == IR_SLOT_C_ADDR)
		memcpy(&slotC, &newSlot, sizeof(IR_FlashSlot));

	// Erase sector once
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Sector = FLASH_SECTOR_IR;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASHEx_Erase(&erase, &sectorError);

	// Write back all slots
	IR_FlashSlot *slots[3] = { &slotA, &slotB, &slotC };
	uint32_t addresses[3] = { IR_SLOT_A_ADDR, IR_SLOT_B_ADDR, IR_SLOT_C_ADDR };

	for (int s = 0; s < 3; s++) {
		uint32_t *data = (uint32_t*) slots[s];
		for (uint32_t i = 0; i < sizeof(IR_FlashSlot) / 4; i++) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addresses[s] + (i * 4),
					data[i]);
		}
	}

	HAL_FLASH_Lock();
}

uint8_t Flash_Load_IR(uint32_t address) {
	IR_FlashSlot *slot = (IR_FlashSlot*) address;

	if (slot->valid != IR_VALID_FLAG)
		return 0;

	ir_pulse_count = slot->length;

	for (uint16_t i = 0; i < ir_pulse_count; i++)
		ir_pulses[i] = slot->pulses[i];

	return 1;
}

void LED_Blink(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t times) {
	for (uint8_t i = 0; i < times; i++) {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(150);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(150);
	}
}
/**
 * @brief  Timer Input Capture Callback
 * CRITICAL FIX: Changed from TIM_CHANNEL_2 to TIM_CHANNEL_1!
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2 && is_recording && !is_transmitting) {
		// DEBUG toggle
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

		last_edge_time = HAL_GetTick();

		if (edge_count == 0) {
			capture_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			edge_count = 1;
		} else {
			capture_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			uint32_t pulse_width;

			if (capture_val2 >= capture_val1) {
				pulse_width = capture_val2 - capture_val1;
			} else {
				pulse_width = (0xFFFFFFFF - capture_val1) + capture_val2;
			}

			if (ir_pulse_count < MAX_IR_PULSES && pulse_width > 50) {
				ir_pulses[ir_pulse_count++] = pulse_width;
			}

			capture_val1 = capture_val2;
		}
	}
}

/**
 * @brief  External Interrupt Callback (Button)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		uint32_t current_time = HAL_GetTick();

		// Debounce - ignore if pressed within 50ms
		if (current_time - last_button_time < 50) {
			return;
		}
		last_button_time = current_time;

		// User button pressed
		if (button_press_count == 0) {
			// First press
			button_press_time = current_time;
			button_press_count = 1;

		} else if (button_press_count == 1) {
			// Check if second press is within double-click timeout
			if ((current_time - button_press_time) < DOUBLE_CLICK_TIMEOUT) {
				// Double click detected
				button_event = 2;
				button_press_count = 0;
			}
		}
	}
}

/**
 * @brief  SysTick Callback - called every 1ms
 */
void HAL_SYSTICK_Callback(void) {
	// Check for single click timeout
	if (button_press_count == 1) {
		uint32_t current_time = HAL_GetTick();
		if ((current_time - button_press_time) >= DOUBLE_CLICK_TIMEOUT) {
			// Single click detected
			button_event = 1;
			button_press_count = 0;
		}
	}
}

void Flash_Save_NFC(uint32_t address)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t sectorError;
    NFC_FlashSlot slotA;
    NFC_FlashSlot slotB;
    NFC_FlashSlot slotC;

    memcpy(&slotA, (void*)NFC_SLOT_A_ADDR, sizeof(NFC_FlashSlot));
    memcpy(&slotB, (void*)NFC_SLOT_B_ADDR, sizeof(NFC_FlashSlot));
    memcpy(&slotC, (void*)NFC_SLOT_C_ADDR, sizeof(NFC_FlashSlot));

    if (address == NFC_SLOT_A_ADDR)
    	memcpy(&slotA, &nfc_temp_slot, sizeof(NFC_FlashSlot));
    else if (address == NFC_SLOT_B_ADDR)
    	memcpy(&slotB, &nfc_temp_slot, sizeof(NFC_FlashSlot));
    else if (address == NFC_SLOT_C_ADDR)
    	memcpy(&slotC, &nfc_temp_slot, sizeof(NFC_FlashSlot));

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_NFC;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&erase, &sectorError);

    NFC_FlashSlot *slots[3] = { &slotA, &slotB, &slotC };
    uint32_t addresses[3] = { NFC_SLOT_A_ADDR, NFC_SLOT_B_ADDR, NFC_SLOT_C_ADDR };
    for (int s = 0; s < 3; s++) {
    	uint32_t *data = (uint32_t*)slots[s];
    	for (uint32_t i = 0; i < sizeof(NFC_FlashSlot) / 4; i++) {
    		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
    		                  addresses[s] + (i * 4), data[i]);
    	}
    }

    HAL_FLASH_Lock();
}

void Flash_Save_RF(uint32_t address, uint8_t *data, uint8_t len)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    uint32_t sectorError;

    RF_FlashSlot slotA, slotB, slotC;
    memcpy(&slotA, (void*)RF_SLOT_A_ADDR, sizeof(RF_FlashSlot));
    memcpy(&slotB, (void*)RF_SLOT_B_ADDR, sizeof(RF_FlashSlot));
    memcpy(&slotC, (void*)RF_SLOT_C_ADDR, sizeof(RF_FlashSlot));

    RF_FlashSlot newSlot;
    newSlot.valid = RF_VALID_FLAG;
    newSlot.length = len;
    memcpy(newSlot.data, data, len);

    if (address == RF_SLOT_A_ADDR)
        memcpy(&slotA, &newSlot, sizeof(RF_FlashSlot));
    else if (address == RF_SLOT_B_ADDR)
        memcpy(&slotB, &newSlot, sizeof(RF_FlashSlot));
    else if (address == RF_SLOT_C_ADDR)
        memcpy(&slotC, &newSlot, sizeof(RF_FlashSlot));

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_RF;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&erase, &sectorError);

    RF_FlashSlot *slots[3] = {&slotA, &slotB, &slotC};
    uint32_t addrs[3] = {RF_SLOT_A_ADDR, RF_SLOT_B_ADDR,
                          RF_SLOT_C_ADDR};
    for (int s = 0; s < 3; s++) {
        uint32_t *d = (uint32_t*)slots[s];
        for (uint32_t i = 0; i < sizeof(RF_FlashSlot)/4; i++)
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              addrs[s]+(i*4), d[i]);
    }
    HAL_FLASH_Lock();
}

uint8_t Flash_Load_RF(uint32_t address,
                       uint8_t *data, uint8_t *len)
{
    RF_FlashSlot *slot = (RF_FlashSlot*)address;
    if (slot->valid != RF_VALID_FLAG) return 0;
    *len = slot->length;
    memcpy(data, slot->data, *len);
    return 1;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
