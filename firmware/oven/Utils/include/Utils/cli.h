#ifndef _CLI_H
#define _CLI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
   const char *cmd;
   const char *description;
   void (*cmdFunc)(uint8_t argc, char **argv);
}sTermEntry_t;

void cli_init(const char *prompt);
void cli_handleByte(uint8_t byte);
void cli_run();

extern const sTermEntry_t hEntry;
extern const sTermEntry_t helpEntry;
extern const sTermEntry_t histEntry;
extern const sTermEntry_t rebootEntry;
extern const sTermEntry_t bootEntry;

#ifdef __cplusplus
}
#endif

#endif
