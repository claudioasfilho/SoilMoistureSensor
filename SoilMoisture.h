
#ifndef SOILMOISTURE_H
#define SOILMOISTURE_H

#include <stdint.h>

/***************************************************************************//**
 * @brief
 *    External signal bit mask for the BLE API
 ******************************************************************************/
#define LE_MONITOR_SIGNAL     0x01

/***************************************************************************//**
 * @brief
 *    Number of samples to measure before calculating the average and notifying
 *    the connected device.
 ******************************************************************************/
#define NUM_OF_SAMPLES        128

/***************************************************************************//**
 * @brief
 *    Sampling frequency of the voltage reading. (Frequency that device will be
 *    notified)
 ******************************************************************************/
#define SAMPLING_FREQ_HZ      50



#define IADC_SOFTTIMER_HANDLER 0xFE
#define SENSOR_SAMPLING_TIME  1 //in Seconds
#define SENSOR_POWER_PORT gpioPortC
#define SENSOR_POWER_PIN  1
#define ADC_INT_IRQ 0xaaaa


typedef union {

        uint32_t data;
        uint8_t array[4];
}_32BArray_Union_t;

void initGPIO(void);
void initPRS(void);
void initIADC(void);
void init_SoilMoisture(void);
void Measure_SoilMoisture(void);
uint32_t Get_SensMoistData(void);

#endif // SOILMOISTURE_H
