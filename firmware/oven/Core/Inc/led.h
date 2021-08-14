/*
 * led.h
 *
 *  Created on: 14 Aug 2021
 *      Author: janus
 */

#ifndef CORE_INC_LED_H_
#define CORE_INC_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

void led_run();
void led_error();
void led_idle();
void led_busy();

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_LED_H_ */
