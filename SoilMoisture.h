
#ifndef SOILMOISTURE_H
#define SOILMOISTURE_H


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
