/*
 * SoilMoisture.c
 *
 *  Created on: Apr 8, 2022
 *      Author: clfilho
 */



//IADC
#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "SoilMoisture.h"





/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
// CLK_SRC_ADC; largest division is by 4
#define CLK_SRC_ADC_FREQ        40000

// CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#define CLK_ADC_FREQ            10000

// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS          CDBUSALLOC
#define IADC_INPUT_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0



/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/

// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;

/**************************************************************************//**
 * @brief  GPIO Initializer
 *****************************************************************************/
void initGPIO(void)
{
  // Enable GPIO clock branch
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB1 as output, will indicate when conversions are being performed
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(SENSOR_POWER_PORT, SENSOR_POWER_PIN, gpioModePushPull, 0);

  //Turning Off everything else on the board

  //Turn Off temp/Hum Sensor
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);
  //Turn Off MIC
  GPIO_PinModeSet(gpioPortC, 7, gpioModePushPull, 0);
  //Turn Off SPI FLASH
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 1);
  //Turn Off IMU
  GPIO_PinModeSet(gpioPortB, 4, gpioModePushPull, 0);



}

/**************************************************************************//**
 * @brief  PRS Initializer
 *****************************************************************************/
void initPRS(void)
{
  // Enable PRS clock
  CMU_ClockEnable(cmuClock_PRS, true);

  // Connect PRS Async channel 0 to ADC single complete signal
  PRS_SourceAsyncSignalSet(0, PRS_ASYNC_CH_CTRL_SOURCESEL_IADC0,
                           PRS_ASYNC_CH_CTRL_SIGSEL_IADC0SINGLEDONE);

  // Route PRS channel 0 to PB1 to indicate a conversion complete
  PRS_PinOutput(0,prsTypeAsync, gpioPortC, 0);
}

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
//  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  // Set oversampling rate to 32x
  // resolution formula res = 11 + log2(oversampling * digital averaging)
  // in this case res = 11 + log2(32*1) = 16
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed32x;


  // Single initialization
  initSingle.dataValidLevel = _IADC_SINGLEFIFOCFG_DVL_VALID1;

  // Set conversions to run continuously
  initSingle.triggerAction = iadcTriggerActionOnce;//iadcTriggerActionContinuous; //

  // Set alignment to right justified with 16 bits for data field
  initSingle.alignment = iadcAlignRight16;

  // Configure Input sources for single ended conversion
  initSingleInput.posInput = iadcPosInputPortCPin2;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize IADC
  // Note oversampling and digital averaging will affect the offset correction
  // This is taken care of in the IADC_init() function in the emlib
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

  // Enable interrupts on data valid level
  IADC_enableInt(IADC0, IADC_IEN_SINGLEFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}


void init_SoilMoisture(void)
{
  // Initialize GPIO
  initGPIO();

  // Initialize PRS
  initPRS();

//  // Initialize the IADC
  initIADC();
//
  //app_log("ADC Initialized\n\r");
//
//  // Start single
  IADC_command(IADC0, iadcCmdStartSingle);
}

/**************************************************************************//**
 * @brief  ADC Handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{

  GPIO_PinOutClear(SENSOR_POWER_PORT, SENSOR_POWER_PIN);
  // Read data from the FIFO, 16-bit result
  sample = IADC_pullSingleFifoResult(IADC0);

  IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL);

  sl_bt_external_signal(ADC_INT_IRQ);
}




void Measure_SoilMoisture(void)
{

  //The GPIO will power the sensor, and it will be disbaled on the ADC IRQ
  GPIO_PinOutSet(SENSOR_POWER_PORT, SENSOR_POWER_PIN);
  //It triggers the ADC for reading
  IADC_command(IADC0,iadcCmdStartSingle);

  //app_log("triggering ADC \n\r");

}

uint32_t Get_SensMoistData(void)
{
  return sample.data;
}
