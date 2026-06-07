/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

// =========================================================================
// ТУМБЛЕР ЗАДНЬОГО ХОДУ — підключення до правого роз'єму
// =========================================================================
// PB11 (RX правого роз'єму) -> один контакт тумблера
// GND  (правого роз'єму)    -> другий контакт тумблера
//
// Внутрішня підтяжка до 3.3V (GPIO_PULLUP):
//   Тумблер РОЗІМКНЕНИЙ = PB11 HIGH = їзда ВПЕРЕД
//   Тумблер ЗАМКНЕНИЙ   = PB11 LOW  = їзда НАЗАД
//
// ВАЖЛИВО: SUPPORT_BUTTONS_RIGHT має бути ВИМКНЕНО в config.h,
// щоб HAL не переініціалізував PB11 як кнопку.
// DEBUG_SERIAL_USART3 використовує PB10 (TX) — конфлікту немає.
// =========================================================================
#define REVERSE_PORT   GPIOB
#define REVERSE_PIN    GPIO_PIN_11

// =========================================================================
// НАПРЯМОК ОБЕРТАННЯ — визначено тут, не через #define в config.h
// =========================================================================
// Лівий мотор:  pwml > 0 = ВПЕРЕД (без інверсії)
// Правий мотор: pwmr > 0 = НАЗАД  (потрібна інверсія знаку)
//
// Якщо після прошивки одне колесо крутиться не в той бік — змінити
// відповідне значення на протилежне: 1 -> -1 або -1 -> 1
// =========================================================================
#define DIR_LEFT   ( 1)   //  1 = без інверсії,  -1 = з інверсією
#define DIR_RIGHT  (-1)   //  1 = без інверсії,  -1 = з інверсією

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  extern LCD_PCF8574_HandleTypeDef lcd;
  extern uint8_t LCDerrorFlag;
#endif

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

volatile uint8_t uart_buf[200];

extern P    rtP_Left;
extern P    rtP_Right;
extern ExtY rtY_Left;
extern ExtY rtY_Right;
extern ExtU rtU_Left;
extern ExtU rtU_Right;

extern uint8_t     inIdx;
extern uint8_t     inIdx_prev;
extern InputStruct input1[];
extern InputStruct input2[];

extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern volatile uint32_t timeoutCntGen;
extern volatile uint8_t  timeoutFlgGen;
extern uint8_t timeoutFlgADC;
extern uint8_t timeoutFlgSerial;

extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;

#if defined(SIDEBOARD_SERIAL_USART2)
extern SerialSideboard Sideboard_L;
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
extern SerialSideboard Sideboard_R;
#endif
#if (defined(CONTROL_PPM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PPM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif
#if (defined(CONTROL_PWM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PWM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t pwm_captured_ch1_value;
extern volatile uint16_t pwm_captured_ch2_value;
#endif

//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;
int16_t board_temp_deg_c;
int16_t left_dc_curr;
int16_t right_dc_curr;
int16_t dc_curr;
int16_t cmdL;
int16_t cmdR;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

#ifdef VARIANT_TRANSPOTTER
  uint8_t  nunchuk_connected;
  extern float    setDistance;
  static uint8_t  checkRemote = 0;
  static uint16_t distance;
  static float    steering;
  static int      distanceErr;
  static int      lastDistance = 0;
  static uint16_t transpotter_counter = 0;
#endif

static int16_t    speed;
#ifndef VARIANT_TRANSPOTTER
  static int16_t  steer;
  static int16_t  steerRateFixdt;
  static int16_t  speedRateFixdt;
  static int32_t  steerFixdt;
  static int32_t  speedFixdt;
#endif

static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;

static uint16_t rate = RATE;

// =========================================================================
// Стан тумблера заднього ходу
// 0 = вперед (PB11 HIGH, тумблер розімкнений)
// 1 = назад  (PB11 LOW,  тумблер замкнений на GND)
// =========================================================================
static uint8_t reverseActive = 0;

#ifdef MULTI_MODE_DRIVE
  static uint8_t drive_mode;
  static uint16_t max_speed;
#endif


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
  Input_Lim_Init();
  Input_Init();

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  // =========================================================================
  // Ініціалізація GPIO для тумблера заднього ходу (PB11)
  // Виконується ПІСЛЯ MX_GPIO_Init(), щоб перевизначити пін якщо потрібно.
  // GPIO_PULLUP — зовнішній резистор не потрібен.
  // =========================================================================
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = REVERSE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(REVERSE_PORT, &GPIO_InitStruct);
  }

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  #ifdef MULTI_MODE_DRIVE
    if (adc_buffer.l_tx2 > input1[0].min + 50 && adc_buffer.l_rx2 > input2[0].min + 50) {
      drive_mode = 2;
      max_speed = MULTI_MODE_DRIVE_M3_MAX;
      rate = MULTI_MODE_DRIVE_M3_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M3_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M3_I_MOT_MAX * A2BIT_CONV) << 4;
    } else if (adc_buffer.l_tx2 > input1[0].min + 50) {
      drive_mode = 1;
      max_speed = MULTI_MODE_DRIVE_M2_MAX;
      rate = MULTI_MODE_DRIVE_M2_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M2_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M2_I_MOT_MAX * A2BIT_CONV) << 4;
    } else {
      drive_mode = 0;
      max_speed = MULTI_MODE_DRIVE_M1_MAX;
      rate = MULTI_MODE_DRIVE_M1_RATE;
      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M1_N_MOT_MAX << 4;
      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M1_I_MOT_MAX * A2BIT_CONV) << 4;
    }
    printf("Drive mode %i selected: max_speed:%i acc_rate:%i \r\n", drive_mode, max_speed, rate);
  #endif

  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  #ifdef MULTI_MODE_DRIVE
    int iTimeout = 0;
    while((adc_buffer.l_rx2 + adc_buffer.l_tx2) >= (input1[0].min + input2[0].min) && iTimeout++ < 300) {
      HAL_Delay(10);
    }
  #endif

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {

    readCommand();
    calcAvgSpeed();

    // =========================================================================
    // ЧИТАННЯ ТУМБЛЕРА ЗАДНЬОГО ХОДУ
    // Перемикання дозволено тільки при швидкості < 30 rpm (авто майже стоїть).
    // PB11 LOW  (замкнений на GND)  -> задній хід
    // PB11 HIGH (розімкнений)       -> передній хід
    // =========================================================================
    if (speedAvgAbs < 30) {
      reverseActive = (HAL_GPIO_ReadPin(REVERSE_PORT, REVERSE_PIN) == GPIO_PIN_RESET) ? 1 : 0;
    }

    #ifndef VARIANT_TRANSPOTTER
      // ####### MOTOR ENABLING #######
      if (enable == 0 && !rtY_Left.z_errCode && !rtY_Right.z_errCode &&
          ABS(input1[inIdx].cmd) < 50 && ABS(input2[inIdx].cmd) < 50) {
        beepShort(6);
        beepShort(4); HAL_Delay(100);
        steerFixdt = speedFixdt = 0;
        enable = 1;
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }

      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50);
      #endif

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold();
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {
        if (speedAvgAbs < 60) {
          multipleTapDet(input1[inIdx].cmd, HAL_GetTick(), &MultipleTapBrake);
        }
        if (input1[inIdx].cmd > 30) {
          input2[inIdx].cmd = (int16_t)((input2[inIdx].cmd * speedBlend) >> 15);
          cruiseControl((uint8_t)rtP_Left.b_cruiseCtrlEna);
        }
      }
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {
        if (speedAvg > 0) {
          input1[inIdx].cmd = (int16_t)((-input1[inIdx].cmd * speedBlend) >> 15);
        } else {
          input1[inIdx].cmd = (int16_t)(( input1[inIdx].cmd * speedBlend) >> 15);
        }
      }
      #endif

      #ifdef VARIANT_SKATEBOARD
        if (input2[inIdx].cmd < 0) {
          if (speedAvg > 0) {
            input2[inIdx].cmd  = (int16_t)(( input2[inIdx].cmd * speedBlend) >> 15);
          } else {
            input2[inIdx].cmd  = (int16_t)((-input2[inIdx].cmd * speedBlend) >> 15);
          }
        }
      #endif

      // ####### LOW-PASS FILTER #######
      rateLimiter16(input1[inIdx].cmd, rate, &steerRateFixdt);
      rateLimiter16(input2[inIdx].cmd, rate, &speedRateFixdt);
      filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      steer = (int16_t)(steerFixdt >> 16);
      speed = (int16_t)(speedFixdt >> 16);

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {
        #ifdef MULTI_MODE_DRIVE
        if (speed >= max_speed) { speed = max_speed; }
        #endif
        if (!MultipleTapBrake.b_multipleTap) {
          speed = steer + speed;
        } else {
          speed = steer - speed;
        }
        steer = 0;
      }
      #endif

      // ####### MIXER / TANK STEERING #######
      #if defined(TANK_STEERING) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_SKATEBOARD)
        cmdL = steer;
        cmdR = speed;
      #else
        mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);
      #endif

      // =========================================================================
      // ЗАСТОСУВАННЯ РЕВЕРСУ
      // Інвертуємо обидві команди — обидва мотори крутяться у зворотний бік.
      // Виконується після фільтрів і міксера — плавно і безпечно.
      // =========================================================================
      if (reverseActive) {
        cmdL = -cmdL;
        cmdR = -cmdR;
      }

      // =========================================================================
      // ВИВІД НА ШІМ
      // DIR_LEFT / DIR_RIGHT враховують фізичну орієнтацію моторів на рамі.
      // Якщо колесо крутиться не в той бік — змінити відповідну константу:
      //   DIR_LEFT  = 1  або -1
      //   DIR_RIGHT = 1  або -1
      // =========================================================================
      pwml = DIR_LEFT  * cmdL;
      pwmr = DIR_RIGHT * cmdR;

    #endif  // #ifndef VARIANT_TRANSPOTTER

    #ifdef VARIANT_TRANSPOTTER
      distance    = CLAMP(input1[inIdx].cmd - 180, 0, 4095);
      steering    = (input2[inIdx].cmd - 2048) / 2048.0;
      distanceErr = distance - (int)(setDistance * 1345);

      if (nunchuk_connected == 0) {
        cmdL = cmdL * 0.8f + (CLAMP(distanceErr + (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        cmdR = cmdR * 0.8f + (CLAMP(distanceErr - (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        if (distanceErr > 0) { enable = 1; }
        if (distanceErr > -300) {
          pwml = DIR_LEFT  * cmdL;
          pwmr = DIR_RIGHT * cmdR;
          if (checkRemote) {
            if (!HAL_GPIO_ReadPin(LED_PORT, LED_PIN)) {
              // enable = 1;
            } else {
              enable = 0;
            }
          }
        } else {
          enable = 0;
        }
        timeoutCntGen = 0;
        timeoutFlgGen = 0;
      }

      if (timeoutFlgGen) {
        pwml = 0;
        pwmr = 0;
        enable = 0;
        #ifdef SUPPORT_LCD
          LCD_SetLocation(&lcd,  0, 0); LCD_WriteString(&lcd, "Len:");
          LCD_SetLocation(&lcd,  8, 0); LCD_WriteString(&lcd, "m(");
          LCD_SetLocation(&lcd, 14, 0); LCD_WriteString(&lcd, "m)");
        #endif
        HAL_Delay(1000);
        nunchuk_connected = 0;
      }

      if ((distance / 1345.0) - setDistance > 0.5 && (lastDistance / 1345.0) - setDistance > 0.5) {
        enable = 0;
        beepLong(5);
        #ifdef SUPPORT_LCD
          LCD_ClearDisplay(&lcd);
          HAL_Delay(5);
          LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Emergency Off!");
          LCD_SetLocation(&lcd, 0, 1); LCD_WriteString(&lcd, "Keeper too fast.");
        #endif
        poweroff();
      }

      #ifdef SUPPORT_NUNCHUK
        if (transpotter_counter % 500 == 0) {
          if (nunchuk_connected == 0 && enable == 0) {
            if(Nunchuk_Read() == NUNCHUK_CONNECTED) {
              #ifdef SUPPORT_LCD
                LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Nunchuk Control");
              #endif
              nunchuk_connected = 1;
            }
          } else {
            nunchuk_connected = 0;
          }
        }
      #endif

      #ifdef SUPPORT_LCD
        if (transpotter_counter % 100 == 0) {
          if (LCDerrorFlag == 1 && enable == 0) {
          } else {
            if (nunchuk_connected == 0) {
              LCD_SetLocation(&lcd,  4, 0); LCD_WriteFloat(&lcd,distance/1345.0,2);
              LCD_SetLocation(&lcd, 10, 0); LCD_WriteFloat(&lcd,setDistance,2);
            }
            LCD_SetLocation(&lcd,  4, 1); LCD_WriteFloat(&lcd,batVoltage, 1);
          }
        }
      #endif
      transpotter_counter++;
    #endif  // VARIANT_TRANSPOTTER

    // ####### SIDEBOARDS HANDLING #######
    #if defined(SIDEBOARD_SERIAL_USART2)
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
    #endif

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt = (int16_t)(board_temp_adcFixdt >> 16);
    board_temp_deg_c   = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### CALC DC LINK CURRENT #######
    left_dc_curr  = -(rtU_Left.i_DCLink  * 100) / A2BIT_CONV;
    right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;
    dc_curr       = left_dc_curr + right_dc_curr;

    // ####### DEBUG SERIAL OUT #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (main_loop_counter % 25 == 0) {
        #if defined(DEBUG_SERIAL_PROTOCOL)
          process_debug();
        #else
          printf("in1:%i in2:%i cmdL:%i cmdR:%i pwmL:%i pwmR:%i BatADC:%i BatV:%i Temp:%i Rev:%i\r\n",
            input1[inIdx].raw,
            input2[inIdx].raw,
            cmdL,
            cmdR,
            (int)pwml,
            (int)pwmr,
            adc_buffer.batt1,
            batVoltageCalib,
            board_temp_deg_c,
            (int)reverseActive);
        #endif
      }
    #endif

    // ####### FEEDBACK SERIAL OUT #######
    #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      if (main_loop_counter % 2 == 0) {
        Feedback.start       = (uint16_t)SERIAL_START_FRAME;
        Feedback.cmd1        = (int16_t)input1[inIdx].cmd;
        Feedback.cmd2        = (int16_t)input2[inIdx].cmd;
        Feedback.speedR_meas = (int16_t)rtY_Right.n_mot;
        Feedback.speedL_meas = (int16_t)rtY_Left.n_mot;
        Feedback.batVoltage  = (int16_t)batVoltageCalib;
        Feedback.boardTemp   = (int16_t)board_temp_deg_c;

        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            Feedback.cmdLed   = (uint16_t)sideboard_leds_L;
            Feedback.checksum = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^
                                           Feedback.speedR_meas ^ Feedback.speedL_meas ^
                                           Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            Feedback.cmdLed   = (uint16_t)sideboard_leds_R;
            Feedback.checksum = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^
                                           Feedback.speedR_meas ^ Feedback.speedL_meas ^
                                           Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
      }
    #endif

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, temperature is too high\r\n");
      #endif
      poweroff();
    } else if (BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, battery voltage is too low\r\n");
      #endif
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {   // 1 гудок: помилка мотора
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                // 2 гудки: обрив потенціометра
      beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                             // 3 гудки: serial timeout
      beepCount(3, 24, 1);
    } else if (timeoutFlgGen) {                                // 4 гудки: загальний timeout
      beepCount(4, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) { // 5 гудків: перегрів
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {    // швидке пікання: акум розряджений
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {    // повільне пікання: акум слабкий
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && reverseActive && speedAvgAbs > 10) { // пікання при їзді назад
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }

    inactivity_timeout_counter++;

    // ####### INACTIVITY TIMEOUT #######
    if (abs(cmdL) > 50 || abs(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    }

    #if defined(CRUISE_CONTROL_SUPPORT) || defined(STANDSTILL_HOLD_ENABLE)
      if ((abs(rtP_Left.n_cruiseMotTgt)  > 50 && rtP_Left.b_cruiseCtrlEna) ||
          (abs(rtP_Right.n_cruiseMotTgt) > 50 && rtP_Right.b_cruiseCtrlEna)) {
        inactivity_timeout_counter = 0;
      }
    #endif

    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, wheels were inactive for too long\r\n");
      #endif
      poweroff();
    }

    inIdx_prev = inIdx;
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
    }
  }
}


// ===========================================================
/** System Clock Configuration */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}