/*
 * sonoff_pipe.cpp
 *
 *  Created on: 29 Aug 2019
 *      Author: jcera
 */
#include <stdio.h>
#include <string.h>
#include <string.h>
#include <stdlib.h>

#include "Utils/utils.h"
#include "sonoff_pipe.h"
#include "stm32f1xx_hal.h"
#include "led.h"

#define OK_CHECK_INTERVAL 60000

SonoffPipe::SonoffPipe()
{
  mHead = 0;
  mTail = 0;
  mKeepAliveTick = 0;
  mState = UNKNOWN;
  mTerminalReady = false;
  mReceiveCB = 0;
  mOpStep = 0;
  transmitCB = 0;
  mOptimeout = 0;
  mLineReady = false;
  mNotOKcount = 1;
  mPublishError = 0;
}

SonoffPipe::~SonoffPipe()
{
}

void SonoffPipe::handleLine(const char *line)
{
   printf("Sonoff: %s\n", line);
   mLineReady = true;
}


void SonoffPipe::handleByte(uint8_t byte)
{
  mBuffer[mHead] = byte;
  mHead = (mHead + 1) % 128;
}

void SonoffPipe::serviceBuffer()
{
  while(mHead != mTail)
  {
    uint8_t byte = mBuffer[mTail];
    mTail = (mTail + 1) % 128;

    // printf("<- %c\n", byte);
    if((byte == '\n') || (byte == '\r'))
    {
      if(idx > 1)
      {
        line[idx] = 0;
        handleLine((const char*)line);
      }
      idx = 0;
    }
    else
      line[idx++] = byte;

    if(idx > 128)
    {
      idx = 0;
    }

//    //check if this was a message for this node
//    if((line[idx] == '~'))
//    {
//      if(idx > 1)
//      {
//        line[idx] = 0;
//        if(mReceiveCB)
//          mReceiveCB((const char*)line);
//      }
//      idx = 0;
//      continue;
//    }
//
    //check if this was a terminal prompt >>>
    if (idx > 2)
    {
      if(!strncmp((char*)&line[idx - 3], ">>>", 3))
      {
        printf("Terminal prompt on sonoff\n");
        mTerminalReady = true;
      }
    }
//
//    if(idx++ >= 128)
//      idx = 0;
  }
}

void SonoffPipe::resetSonoff()
{
  if((mState == UNKNOWN) || (mState == IDLE))
  {
    printf("Reseting Sonoff\n");

    mOpStep = 0;
    mState = EXIT_PY;
  }
}

void SonoffPipe::checkOK()
{
  if((mState == UNKNOWN) || (mState == IDLE))
  {
    printf("Check Sonoff OK\n");

    mOpStep = 0;
    mState = CHECK_OK;
  }
}

int SonoffPipe::doResetSonoff()
{
  switch(mOpStep)
  {
  case 0:
  {
    uint8_t buff[] = {"exit\n\r\n\r"};
    if(transmitCB(buff, 8))
    {
      printf("Waiting for Sonoff terminal\n");
      mOptimeout = HAL_GetTick() + 2000;
      mOpStep++;
    }
    else
      mOpStep = -3;
  }
  break;
  case 1:
    if(mTerminalReady)
    {
      mOpStep++;
      mTerminalReady = false;
    }

    if(mOptimeout < HAL_GetTick())
    {
      printf("Terminal timed out\n");
      mOpStep++;
    }
    break;

  case 2:
  {
    printf("Sonoff terminal available, resetting\n");
    uint8_t reset_seq[] = {0x0A, 0x0D, 0x04};
    if(transmitCB(reset_seq, 3))
    {
      printf("Software Reset signal sent\n");
      mOptimeout = HAL_GetTick() + 8000;
      mOpStep++;
    }
  }
  break;
  case 3:
    if(mOptimeout < HAL_GetTick())
    {
      printf("Sonoff should be ready...");
      mOpStep = -1;
    }
    break;
  }

  return mOpStep;
}


int SonoffPipe::doIsSonoffOK()
{
  switch(mOpStep)
  {
  case 0:
  {
    uint8_t buff[] = {"\n"};
    if(transmitCB(buff, 1))
    {
      printf("Waiting for Sonoff KO\n");
      mOptimeout = HAL_GetTick() + 2000;
      mOpStep++;
    }
    else
      mOpStep = -3;
  }
  break;
  case 1:
    if(mLineReady)
    {
      mLineReady = false;
      printf(GREEN("Sonoff is OK: %s\n"), line);
      if(!strncmp((char*)line, "KO", 2))
      {
        mNotOKcount = 0;
        mOpStep = -1;
      }
    }

    if(mOptimeout < HAL_GetTick())
    {
      printf(RED("OK timed out\n"));
      mNotOKcount++;
      mOpStep = -2;
    }
    break;
  }

  return mOpStep;
}

bool SonoffPipe::publish(const char *message)
{
  if(mState != IDLE)
    return false;

  if(mNotOKcount != 0)
    return false;

  int str_len = strlen(message);
  if(str_len < 126)
  {
    strcpy(mPublishMessage, message);
    mPublishMessage[str_len] = '~';
    mPublishMessage[str_len + 1] = 0;

    mKeepAliveTick = HAL_GetTick() + OK_CHECK_INTERVAL;

    mOpStep = 0;
    mState = PUBLISH;
    return true;
  }

  return false;
}

int SonoffPipe::doPublish()
{
  switch(mOpStep)
  {
  case 0:
  {
    printf("Sonoff: Publish %s\n", mPublishMessage);
    if(!transmitCB((uint8_t*)mPublishMessage, strlen(mPublishMessage)))
    {
      printf("Sent publish request timed out\n");
    }
    else
    {
      mOpStep++;
      mOptimeout = HAL_GetTick() + 8000;
    }
  }
  break;
  case 1:
  {
    if(mLineReady)
    {
      mLineReady = false;
      printf("Sonoff sent: %s\n", line);
      if(!strncmp((char*)line, "OK", 2))
      {
        mPublishError = 0;
        mOpStep = -1;
      }
    }

    if(mOptimeout < HAL_GetTick())
    {
      printf("Publish timed out\n");
      mPublishError++;
      mOpStep = -2;
    }
  }
  break;
  }

  return mOpStep;
}

void SonoffPipe::run()
{
  // printf(" - state %d\n", mState);
  serviceBuffer();

  switch(mState)
  {
  default:
  case UNKNOWN:
  case IDLE:
    break;
  case EXIT_PY:
  {
    if(doResetSonoff() < 0)
    {
      mOpStep = 0;
      mState = CHECK_OK;
    }
  }
  break;
  case CHECK_OK:
    if(doIsSonoffOK() < 0)
    {
        led_idle();
      mState = IDLE;
    }
    break;
  case PUBLISH:
  {
    if(doPublish() < 0)
    {
      mState = IDLE;
    }
  }
  break;
  }

  if(mKeepAliveTick < HAL_GetTick())
  {
    mKeepAliveTick = HAL_GetTick() + OK_CHECK_INTERVAL;
    //publish("{\"msg\":\"hb\"}");
    checkOK();
  }

  if(mNotOKcount > 4)
  {
    mNotOKcount = 0;
    resetSonoff();
  }

  if(mPublishError > 4)
  {
    mPublishError = 0;
    resetSonoff();
  }
}

void SonoffPipe::setReceivedCB(void (*receive_cb)(const char* line))
{
  mReceiveCB = receive_cb;
}

void SonoffPipe::setTransmitCB(int (*transmit_cb)(uint8_t *buffer, int len))
{
  transmitCB = transmit_cb;
}
