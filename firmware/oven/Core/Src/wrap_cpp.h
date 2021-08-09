/*
 * wrap_cpp.h
 *
 *  Created on: 09 May 2020
 *      Author: jerasmus
 */

#ifndef _WRAP_CPP_H_
#define _WRAP_CPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "Utils/cli.h"

 void cpp_init();
 void cpp_run();
 void esp_handle_byte(uint8_t byte);
 int esp_idle();

 void request_report();

extern const sTermEntry_t sonoffEntry;
#ifdef __cplusplus
 }
#endif
#endif /* C_WRAP_CPP_H_ */
