/*
 * commands.c
 *
 *  Created on: May 1, 2020
 *      Author: jerasmus
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Utils/cli.h"
#include "adc.h"
#include "rtc.h"
#include "wrap_cpp.h"

extern const sTermEntry_t stepEntry;

void adcDebug(uint8_t argc, char **argv)
{
    adc_show();
}

const sTermEntry_t adcEntry =
{ "a", "Sample ADC", adcDebug };

const char *getDayName(int week_day)
{
  switch(week_day)
  {
  case RTC_WEEKDAY_MONDAY:
    return "Monday";
  case RTC_WEEKDAY_TUESDAY:
    return "Tuesday";
  case RTC_WEEKDAY_WEDNESDAY:
    return "Wednesday";
  case RTC_WEEKDAY_THURSDAY:
    return "Thursday";
  case RTC_WEEKDAY_FRIDAY:
    return "Friday";
  case RTC_WEEKDAY_SATURDAY:
    return "Saturday";
  case RTC_WEEKDAY_SUNDAY:
    return "Sunday";
  }

  return 0;
}

void rtc_debug(uint8_t argc, char **argv)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  if(argc > 5)
  {
    printf("Setting date %d\n", atoi(argv[5]));

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Year = atoi(argv[1]) - 2000;
    sDate.Month = atoi(argv[2]);
    sDate.Date = atoi(argv[3]);
    sTime.Hours = atoi(argv[4]);
    sTime.Minutes = atoi(argv[5]);
    sTime.Seconds = 0;

    RCC->APB1ENR |= (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
    //PWR->CR |= PWR_CR_DBP;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  }


  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  printf("RTC date: %s\n", getDayName(sDate.WeekDay));
  printf(" - %04d-%02d-%02d ", 2000 +sDate.Year, sDate.Month, sDate.Date);
  printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
}
void rtc_debug(uint8_t argc, char **argv);
const sTermEntry_t rtcEntry =
{ "date", "RTC date", rtc_debug };


const sTermEntry_t *cli_entries[] =
{
      &hEntry,
      &helpEntry,
      &histEntry,
      &rebootEntry,
      &bootEntry,
      &adcEntry,
      &rtcEntry,
      &sonoffEntry,
	  0
};
