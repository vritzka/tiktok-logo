#ifndef GENERAL_H
#define GENERAL_H

#include "lvgl.h"
#include "ui/ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include <stdio.h>
#include <math.h> 

extern SemaphoreHandle_t xGuiSemaphore;



#endif
