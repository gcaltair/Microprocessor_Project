#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
  uint16_t battery_raw;
  uint16_t potentiometer_raw;
} RobotAdcSample_t;

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);
HAL_StatusTypeDef ADC_ReadRobotInputs(RobotAdcSample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
