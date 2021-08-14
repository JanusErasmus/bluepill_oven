#include "main.h"

typedef enum
{
    LED_IDLE,
    LED_BUSY,
    LED_ERROR
} eLEDstate;

static eLEDstate mLEDstate = LED_ERROR;
static uint32_t mFlashState = 0;
static uint32_t led_tick = 0;

void led_run()
{
    if(led_tick > HAL_GetTick())
        return;

    led_tick = HAL_GetTick() + 20;

    switch(mLEDstate)
    {
    case LED_IDLE:
    {
        mFlashState++;
        if(mFlashState == 1)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else if(mFlashState == 8)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        else if(mFlashState == 16)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else if(mFlashState == 24)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        else if(mFlashState > 50)
        {
            mFlashState = 0;
        }
    }
        break;
    case LED_BUSY:
    {
        mFlashState++;
        if(mFlashState == 1)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else if(mFlashState == 15)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        else if(mFlashState >= 30)
        {
            mFlashState = 0;
        }
    }
        break;
    case LED_ERROR:
    {
        mFlashState++;
        if(mFlashState == 1)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else if(mFlashState == 2)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        else if(mFlashState >= 4)
        {
            mFlashState = 0;
        }
    }
        break;
    }
}

void led_error()
{
    mFlashState = 0;
    mLEDstate = LED_ERROR;
}

void led_idle()
{
    mFlashState = 0;
    mLEDstate = LED_IDLE;
}

void led_busy()
{
    mFlashState = 0;
    mLEDstate = LED_BUSY;
}
