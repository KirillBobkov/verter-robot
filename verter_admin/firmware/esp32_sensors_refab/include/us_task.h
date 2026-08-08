#ifndef US_TASK_H
#define US_TASK_H

#include "defines.h"


void usTask(void *context);

float read_sensor(int trig, int echo);

#endif // US_TASK_H