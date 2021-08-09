/*
 * sonoff_pipe.h
 *
 *  Created on: 29 Aug 2019
 *      Author: jcera
 */

#ifndef SRC_SONOFF_PIPE_H_
#define SRC_SONOFF_PIPE_H_
#include <stdint.h>

class SonoffPipe
{
  enum eState
  {
    UNKNOWN,
    IDLE,
    CHECK_OK,
    EXIT_PY,
    WAIT_TERMINAL,
    PUBLISH
  }mState;
  uint8_t mBuffer[128];
  uint8_t line[128];
  bool mLineReady;
  int idx = 0;
  int mHead;
  int mTail;
  char mPublishMessage[128];
  int (*transmitCB)(uint8_t *buffer, int len);
  uint32_t mKeepAliveTick;
  bool  mTerminalReady;
  int mOpStep;
  uint32_t mOptimeout;
  int mNotOKcount;
  int mPublishError;
  void serviceBuffer();
  void handleLine(const char* line);
  void (*mReceiveCB)(const char* line);

  int doIsSonoffOK();
  int doResetSonoff();
  int doPublish();

public:
  SonoffPipe();
  virtual ~SonoffPipe();

  void handleByte(uint8_t byte);
  void run();
  bool isIdle(){ return (mState == IDLE); }
  void resetSonoff();
  void checkOK();

  bool publish(const char *message);
  void setReceivedCB(void (*receive_cb)(const char* line));
  void setTransmitCB(int (*transmit_cb)(uint8_t *buffer, int len));
};

#endif /* SRC_SONOFF_PIPE_H_ */

