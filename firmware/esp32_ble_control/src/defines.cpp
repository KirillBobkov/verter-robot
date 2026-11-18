#include "defines.h"

volatile ControlFrame ctrl = {};
xSemaphoreHandle xControlFrameMutex = xSemaphoreCreateMutex();