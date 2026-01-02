#ifndef MONITORING_H
#define MONITORING_H

#include "config.h"

/**
 * @brief Monitoring task (FreeRTOS)
 * Handles current sensing, safety checks, and status reporting
 */
void task_monitoring(void *pvParameters);

#endif // MONITORING_H
