#ifndef __KEY_MANAGER_H
#define __KEY_MANAGER_H

#include "main.h"

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_START,
    KEY_EVENT_STOP,
    KEY_EVENT_MODE,
    KEY_EVENT_CLEAR
} KeyEvent_t;

void KeyManager_Init(void);

KeyEvent_t KeyManager_Scan(void);

#endif

