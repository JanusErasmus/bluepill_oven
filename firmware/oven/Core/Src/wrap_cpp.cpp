/*
 * wrap_cpp.cpp
 *
 *  Created on: 09 May 2020
 *      Author: jerasmus
 */
#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "sonoff_pipe.h"
#include "wrap_cpp.h"
#include "adc.h"
#include "led.h"

#define UPDATE_RATE 4000 //Match the sample rate of the power monitor mean measurement
#define REPORT_RATE 600000 // 10 min
//limit Sonoff report rate to 30 s
#define MIN_REPORT_RATE 30000
static float prev_vin = 0;
static float prev_current = 0;

SonoffPipe pipe;
static uint32_t tick = 0;
static uint32_t report_tick = 0;
static uint32_t last_report = 0;

void handleMessage(const char* line)
{
  printf("MQTT: %s\n", line);

}

int sonoff_report()
{
  float ext, vin, current, temp;
  adc_get_values(&ext, &vin, &current, &temp);

  char json[128];
  sprintf(json, "{\"uptime\":%d,"
      "\"temp\": %0.3f,"
      "\"voltages\":[%0.3f,%0.3f,%0.3f,0]"
      "}",
      (int)HAL_GetTick(),
      ext,
      vin,
      current,
      temp
  );

  // Ensure LED is on while TX
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  // printf("Sonoff publish: %s\n", msg);
  if(pipe.publish(json))
  {
    prev_vin = vin;
    prev_current = current;
    report_tick = HAL_GetTick();
    last_report = HAL_GetTick() + REPORT_RATE;

    led_idle();
    return 1;
  }

  led_error();
  return 0;
}

extern "C" {

void request_report()
{
    if(pipe.isIdle())
    {
        sonoff_report();
    }
    else
    {
        printf("Pipe NOT idle, report not requested\n");
    }
}

  void esp_handle_byte(uint8_t byte)
  {
      pipe.handleByte(byte);
  }

  int esp_transmit(uint8_t *buf, int len)
  {
    //the sonoff is not fast enough to receive the whole message at once, delay each byte
    for (int k = 0; k < len; ++k)
    {
      if(HAL_UART_Transmit(&huart2, &buf[k], 1, 300)  != HAL_OK)
        return -1;

      // printf("-> %c\n", buf[k]);
      HAL_Delay(80);
    }

    return len;
  }

 void cpp_init()
 {
   pipe.setTransmitCB(esp_transmit);
   pipe.setReceivedCB(handleMessage);
 }

 void cpp_run()
 {
     pipe.run();

     if(tick < HAL_GetTick())
     {
         tick = HAL_GetTick() + UPDATE_RATE;

         //report on regular intervals
         if(last_report < HAL_GetTick())
         {
             sonoff_report();
         }
     }
 }

 int esp_idle()
 {
   return pipe.isIdle();
 }

 void sonoff_debug(uint8_t argc, char **argv)
 {
   if(argc > 1)
   {
     if(!strcmp(argv[1], "tx"))
     {
       printf("Reporting via Sonoff: %s\n", sonoff_report()?"OK":"KO");
     }

     if(!strcmp(argv[1], "exit"))
     {
       pipe.resetSonoff();
     }

     if(!strcmp(argv[1], "ok"))
     {
         led_busy();
       pipe.checkOK();
     }
   }
 }

 const sTermEntry_t sonoffEntry =
 { "s", "Sonoff TX", sonoff_debug };

}
