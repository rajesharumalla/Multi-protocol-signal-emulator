#include "keypad.h"

char keymap[4][4] = {
    {'*','7','4','1'},
    {'0','8','5','2'},
    {'#','9','6','3'},
    {'D','C','B','A'}
};

GPIO_TypeDef* rowPorts[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
uint16_t rowPins[4]  = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
GPIO_TypeDef* colPorts[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
uint16_t colPins[4]  = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4};

static char  lastKey     = 0;
static uint32_t lastTime = 0;

char Keypad_GetKey(void)
{
    for (uint8_t row = 0; row < 4; row++)
    {
        // All rows HIGH
        HAL_GPIO_WritePin(GPIOC,
            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,
            GPIO_PIN_SET);

        // Pull this row LOW
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);
        HAL_Delay(2);

        for (uint8_t col = 0; col < 4; col++)
        {
            if (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET)
            {
                char key = keymap[row][col];

                // Wait for release
                while (HAL_GPIO_ReadPin(colPorts[col], colPins[col])
                       == GPIO_PIN_RESET) {
                    HAL_Delay(5);
                }
                HAL_Delay(50); // settle after release

                // Ignore same key pressed within 300ms
                uint32_t now = HAL_GetTick();
                if (key == lastKey && (now - lastTime) < 300) {
                    return 0;
                }

                lastKey  = key;
                lastTime = now;
                return key;
            }
        }
    }
    return 0;
}
