#include "../include/Utils/cli.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "Utils/commands.h"
#include "Utils/utils.h"
#include "main.h"
#include "adc.h"
#include "tim.h"

#define CMD_BUFFER_SIZE     128
#define CMD_SIZE            32
#define HISTORY_DEPTH       16

static char mCmdBuffer[CMD_BUFFER_SIZE];
static uint8_t mCmdHead = 0;
static uint8_t mCmdTail = 0;

static char mCommand[CMD_SIZE];
static uint8_t mCmdIndex = 0;

static char mHistory[HISTORY_DEPTH][CMD_SIZE] = {0};
static int mHistoryUse = 0;
static int mHistoryFlag = 0;
static int mHistoryIndex = 0;
static int mHistoryCount = 0;
static int mEscapeState = 0;

extern void WWDG_Refresh();
static const char *mPrompt = 0;

void cli_init(const char *prompt)
{
    memset(mCmdBuffer, 0, CMD_BUFFER_SIZE);
    memset(mCommand, 0, CMD_SIZE);

    mPrompt = prompt;
    printf("%s", prompt);
}

void cli_handleByte(uint8_t byte)
{
    mCmdBuffer[mCmdHead] = byte;
    mCmdHead = (mCmdHead + 1) % CMD_BUFFER_SIZE;
}

static uint8_t parseCommand(char *command, uint8_t argc, char **argv)
{
    uint8_t count = 0;
    argc -= 2;
    argv[count] = command;
    char *ptr = strchr(argv[count], ' ');

    while (ptr && (count++ <= argc))
    {
        ptr[0] = 0;
        ptr++;

        argv[count] = ptr;

        ptr = strchr(argv[count], ' ');
    }

    count++;

    return count;
}

int strip_whiteSpaces(char *cmd)
{
    int len = strlen(cmd);
    int cmd_len = 0;
    for(uint32_t idx = 0; idx < len; idx++)
    {
        cmd_len++;
        if((cmd[idx] == '\n') || (cmd[idx] == '\r'))
        {
            cmd[idx] = 0;
        }
    }
    // printf("\n***\nlen:%d\n***\n", cmd_len);

    return cmd_len;
}

static void cli_handleCommand(char *cmd)
{
    int len = strip_whiteSpaces(cmd);
    if(len > 1)
    {
        if(mHistoryFlag == 0)
        {
            strcpy(mHistory[mHistoryIndex], cmd);
            mHistoryUse = mHistoryIndex;
            mHistoryIndex = (mHistoryIndex + 1) % HISTORY_DEPTH;
            mHistoryCount++;
            if(mHistoryCount >= HISTORY_DEPTH)
                mHistoryCount = HISTORY_DEPTH;

        }
    }
    mHistoryFlag = 0;

    uint8_t processed = 0;
    char* argv[6];
    uint8_t argc = 0;
    uint8_t cmdIndex = 0;
    argc = parseCommand(cmd, 6, argv);
    if (argc)
    {
        const sTermEntry_t *entry = cli_entries[cmdIndex++];
        while (entry)
        {
            if (!strcmp(entry->cmd, cmd))
            {
                entry->cmdFunc(argc, argv);
                processed = 1;
                break;
            }
            entry = cli_entries[cmdIndex++];
        }
    }

    if (!processed && (strlen(cmd) > 0))
    {
        printf(RED("Unknown command '%s', try help\n"), cmd);
    }

    printf(mPrompt);
}

static void print_history()
{
    printf("********HISTORY**********\n");
    for (int k = 0; k < mHistoryCount; ++k)
    {
        printf("%2d: %s\n", k, mHistory[k]);
    }

    printf("*************************\n");
}

static void parseByte(char byte)
{
    //Handle escape characters
    if(byte == 0x1B)
    {
        mEscapeState = 1;
        return;
    }
    if(mEscapeState == 1)
    {
        if(byte == 0x5B)
        {
            mEscapeState = 2;
            return;
        }
        mEscapeState = 0;
    }
    if(mEscapeState == 2)
    {
        mEscapeState = 0;

        //Use latest typed history
        if(mHistoryFlag == 0)
        {
            mHistoryUse++;
        }
        // print_history();
        mHistoryFlag = 1;

        //UP
        if(byte == 0x41)
        {
            mHistoryUse--;
            if(mHistoryUse < 0)
            {
                mHistoryUse = 0;
            }
            printf(CLEAR_LINE "\r%s%s", mPrompt, mHistory[mHistoryUse]);
            strcpy(mCommand, mHistory[mHistoryUse]);
            mCmdIndex = strlen(mHistory[mHistoryUse]);
        }
        //DOWN
        if(byte == 0x42)
        {
            mHistoryUse++;
            if(mHistoryUse >= mHistoryCount)
            {
                mHistoryUse = mHistoryCount - 1;
                return;
            }
            printf(CLEAR_LINE "\r%s%s", mPrompt, mHistory[mHistoryUse]);
            strcpy(mCommand, mHistory[mHistoryUse]);
            mCmdIndex = strlen(mHistory[mHistoryUse]);
        }
        return;
    }

    //Handle backspace
    if(byte == 0x08)
    {
        mCmdIndex--;
        if(mCmdIndex < 0)
            mCmdIndex = 0;
        mCommand[mCmdIndex] = 0;
        printf(CLEAR_LINE "\r%s%s", mPrompt, mCommand);
        return;
    }

    //echo typed character
    printf("%c", byte);
    //printf("0x%02X ", byte);
    if (byte == '\r' || (byte == '\n'))
    {
        printf("\n");

        mCommand[mCmdIndex] = byte;
        if(++mCmdIndex >= CMD_SIZE)
            mCmdIndex = CMD_SIZE - 1;

        mCommand[mCmdIndex] = 0;
        if(++mCmdIndex >= CMD_SIZE)
            mCmdIndex = CMD_SIZE - 1;

        cli_handleCommand(mCommand);
        mCmdIndex = 0;
    }
    else
    {
        mHistoryFlag = 0;
        mCommand[mCmdIndex] = byte;
        if(++mCmdIndex >= CMD_SIZE)
            mCmdIndex = CMD_SIZE - 1;
    }

    if (mCmdIndex >= (CMD_SIZE - 1))
        mCmdIndex = 0;
}

void cli_run()
{
    while (mCmdHead != mCmdTail)
    {
        parseByte(mCmdBuffer[mCmdTail]);
        mCmdTail = (mCmdTail + 1) % CMD_BUFFER_SIZE;
    }
}

void help(uint8_t argc, char **argv)
{
    uint8_t cmdIndex = 0;
    const sTermEntry_t *entry = cli_entries[cmdIndex++];

    printf(YELLOW("Available commands:\n"));
    while (entry)
    {
        printf("%s\t- %s\n", entry->cmd, entry->description);
        entry = cli_entries[cmdIndex++];
    }
}
const sTermEntry_t hEntry =
{ "h", "This help", help };

const sTermEntry_t helpEntry =
{ "help", "This help", help };

void history(uint8_t argc, char **argv)
{
    print_history();
}
const sTermEntry_t histEntry =
{ "history", "Show history buffer", history };

void reboot(uint8_t argc, char **argv)
{
    printf("Rebooting...\n");
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_Delay(100);
    HAL_DeInit();
    NVIC_SystemReset();
}

const sTermEntry_t rebootEntry =
{ "reset", "Reboot the device", reboot };

void jumpBoot(uint8_t argc, char **argv)
{
    printf("Jumping to ROM boot loader\n");
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_Delay(100);
    HAL_DeInit();
    //
    //
    //    printf("Bye\n");
    //    __HAL_RCC_USART1_FORCE_RESET();
    //    HAL_Delay(5);
    //    __HAL_RCC_USART1_RELEASE_RESET();
    //    HAL_Delay(5);
    //
    //    HAL_RCC_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    typedef void (*funcPtr)(void);
    uint32_t system_memory = 0x1FFFF000;
    uint32_t jumpAddr = *(volatile uint32_t *)(system_memory + 0x04); /* reset ptr in vector table */

    funcPtr usrMain = (funcPtr) jumpAddr;

    SCB->VTOR = (volatile uint32_t)system_memory;

    /* Initialize user application's Stack Pointer */
    //__set_MSP(*(__IO uint32_t*) 0x1FFFF000);
    asm volatile("msr msp, %0"::"g"(*(volatile uint32_t *)system_memory));

    usrMain();
}

const sTermEntry_t bootEntry =
{ "boot", "Jump to bootloader", jumpBoot };
