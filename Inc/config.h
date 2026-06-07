// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### VARIANT SELECTION ###############################
#if !defined(PLATFORMIO)
  #define VARIANT_ADC
  //#define VARIANT_USART
  //#define VARIANT_NUNCHUK
  //#define VARIANT_PPM
  //#define VARIANT_PWM
  //#define VARIANT_IBUS
  //#define VARIANT_HOVERCAR
  //#define VARIANT_HOVERBOARD
  //#define VARIANT_TRANSPOTTER
  //#define VARIANT_SKATEBOARD
#endif
// ########################### END OF VARIANT SELECTION ############################


// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000
#define DEAD_TIME              48
#ifdef VARIANT_TRANSPOTTER
  #define DELAY_IN_MAIN_LOOP    2
#else
  #define DELAY_IN_MAIN_LOOP    5
#endif
#define TIMEOUT                20
#define A2BIT_CONV             50

#define ADC_CONV_TIME_1C5       (14)
#define ADC_CONV_TIME_7C5       (20)
#define ADC_CONV_TIME_13C5      (26)
#define ADC_CONV_TIME_28C5      (41)
#define ADC_CONV_TIME_41C5      (54)
#define ADC_CONV_TIME_55C5      (68)
#define ADC_CONV_TIME_71C5      (84)
#define ADC_CONV_TIME_239C5     (252)

#define ADC_CONV_CLOCK_CYCLES   (ADC_CONV_TIME_7C5)
#define ADC_CLOCK_DIV           (4)
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)
// ########################### END OF DO-NOT-TOUCH SETTINGS ############################


// ############################### BOARD VARIANT ###############################
#define BOARD_VARIANT           0
// ######################## END OF BOARD VARIANT ###############################


// ############################### BATTERY ###############################
// УВАГА: акумулятор зношений, реальна напруга ~39В при "повному" заряді.
// BAT_CALIB_REAL_VOLTAGE і BAT_CALIB_ADC — замінити після калібрування!
// Поки що виставлено під 39В акум щоб не вимикався раніше часу.
//
// Як калібрувати:
//   1. Увімкни DEBUG_SERIAL_USART3 в VARIANT_ADC нижче
//   2. Підключи UART-USB адаптер до PB10 (TX правого роз'єму)
//   3. Заряди акум до максимуму, зчитай BatADC і виміряй реальну напругу мультиметром
//   4. Постав: BAT_CALIB_REAL_VOLTAGE = реальна_напруга * 100
//              BAT_CALIB_ADC          = значення BatADC з монітора
// =========================================================================
#define BAT_FILT_COEF           655
// =========================================================================
// КАЛІБРОВКА БАТАРЕЇ — зношений акум, ~38В без навантаження
// =========================================================================
// BAT_CALIB_REAL_VOLTAGE = реальна напруга * 100  (виміряй мультиметром)
// BAT_CALIB_ADC          = значення BatADC з debug виводу в той же момент
//
// Поточні значення виставлені приблизно під 38В акум.
// Після підключення debug (розкоментуй DEBUG_SERIAL_USART3 в VARIANT_ADC)
// — уточни обидва значення для точного відображення напруги.
// =========================================================================
#define BAT_CALIB_REAL_VOLTAGE  3800      // ~38.00В * 100
#define BAT_CALIB_ADC           1415      // приблизне значення під 38В
#define BAT_CELLS               10

// =========================================================================
// ПОРОГИ ЗАХИСТУ — знижені під зношений акум 38В без навантаження
// =========================================================================
// При зношеному акумі напруга під навантаженням просідає на 1-3В.
// Якщо без навантаження 38В, то під навантаженням може бути 35-36В —
// це вже близько до стандартного порогу DEAD (33.7В на 10S).
//
// Тому пороги суттєво знижено:
//   BAT_LVL1 = 32.0В (пікання-попередження) — без навантаження ~35В
//   BAT_DEAD = 30.0В (примусове вимкнення)  — без навантаження ~33В
//
// УВАГА: нижче 30В на 10S акум може отримати незворотні пошкодження.
// Якщо акум сильно гріється або просідає до 28В — одразу зупинись.
// =========================================================================
#define BAT_LVL2_ENABLE         0
#define BAT_LVL1_ENABLE         1         // пікання при слабкому акумі
#define BAT_DEAD_ENABLE         1         // захист від повного розряду
#define BAT_BLINK_INTERVAL      80

#define BAT_LVL5  (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL4  (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL3  (355 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL2  (340 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL1  (320 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE  // 32.0В — пікання
#define BAT_DEAD  (300 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE  // 30.0В — вимкнення
// ######################## END OF BATTERY ###############################


// ############################### TEMPERATURE ###############################
#define TEMP_FILT_COEF          655
#define TEMP_CAL_LOW_ADC        1655
#define TEMP_CAL_LOW_DEG_C      358
#define TEMP_CAL_HIGH_ADC       1588
#define TEMP_CAL_HIGH_DEG_C     489
#define TEMP_WARNING_ENABLE     0
#define TEMP_WARNING            600
#define TEMP_POWEROFF_ENABLE    0
#define TEMP_POWEROFF           650
// ######################## END OF TEMPERATURE ###############################


// ############################### MOTOR CONTROL ###############################
#define COM_CTRL    0
#define SIN_CTRL    1
#define FOC_CTRL    2
#define OPEN_MODE   0
#define VLT_MODE    1
#define SPD_MODE    2
#define TRQ_MODE    3

#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA

// FOC + VOLTAGE MODE: найкращий варіант для зношеного акума —
// контролер сам компенсує просідання напруги при навантаженні.
#define CTRL_TYP_SEL    FOC_CTRL
#define CTRL_MOD_REQ    VLT_MODE
#define DIAG_ENA        1

// =========================================================================
// СТРУМ ТА ШВИДКІСТЬ — збільшено для зношеного акума (39В)
// =========================================================================
// Проблема: при 39В замість 42В мотори отримують на ~7% менше напруги.
// Щоб компенсувати — збільшуємо ліміт струму (I_MOT_MAX) і знімаємо
// обмеження поля (FIELD_WEAK), що дає більший крутний момент на старті.
//
// I_MOT_MAX  = 20А  (було 15А) — більше тяги на старті і підйомах
// I_DC_MAX   = 22А  (було 17А) — відповідно збільшено DC ліміт
// N_MOT_MAX  = 1000 — максимальна швидкість без змін
//
// УВАГА: якщо мотори або контролер сильно гріються — знизити I_MOT_MAX
// назад до 17-18А.
// =========================================================================
#define I_MOT_MAX       20        // А, збільшено з 15 -> більше тяги
#define I_DC_MAX        22        // А, збільшено з 17
#define N_MOT_MAX       1000

// =========================================================================
// FIELD WEAKENING — увімкнено для кращого старту на зношеному акумі
// =========================================================================
// При низькій напрузі акума FOC-контролер швидше виходить на межу напруги.
// Field weakening дозволяє підтримувати момент навіть коли напруга просідає.
// FIELD_WEAK_MAX = 3 (обережно, не перегріти)
// =========================================================================
#define FIELD_WEAK_ENA  1         // увімкнено (було 0)
#define FIELD_WEAK_MAX  3         // обережно з зношеним акумом (було 5)
#define PHASE_ADV_MAX   25
#define FIELD_WEAK_HI   900       // трохи знижено (було 1000) під 39В акум
#define FIELD_WEAK_LO   650       // трохи знижено (було 750) під 39В акум

// Утримання на місці при зупинці
#define STANDSTILL_HOLD_ENABLE

// =========================================================================
// ЕЛЕКТРОГАЛЬМО
// =========================================================================
// MAX=200 (було 300) — трохи слабше гальмо, бо при зношеному акумі
// рекуперація може перезаряджати і без того нестабільний акум.
// THRES=50 — без змін.
// =========================================================================
#define ELECTRIC_BRAKE_ENABLE
#define ELECTRIC_BRAKE_MAX    200       // знижено з 300 (захист акума)
#define ELECTRIC_BRAKE_THRES   50
// ########################### END OF MOTOR CONTROL ############################


// ############################## DEFAULT SETTINGS ##############################
#define INACTIVITY_TIMEOUT        8

#define BEEPS_BACKWARD            1

// ADC Protection
#define ADC_MARGIN                100
#define ADC_PROTECT_TIMEOUT       30
#define ADC_PROTECT_THRESH        200

#define AUTO_CALIBRATION_ENA

// =========================================================================
// RATE — швидкість зміни команди (плавність розгону)
// =========================================================================
// Для зношеного акума ВАЖЛИВО мати плавний старт — різкий розгін дає
// великий кидок струму, що просаджує напругу і може вимкнути контролер.
//
// 120 = плавний розгін (~3 сек від 0 до макс.) — компроміс між
//       динамікою і захистом акума від просідання напруги.
// Якщо хочеш ще плавніше — збільш до 160-200.
// Якщо хочеш динамічніше — зменш до 80 (але акум буде просідати).
// =========================================================================
#define DEFAULT_RATE              120   // плавний старт для зношеного акума

// FILTER = згладжування вхідного сигналу
#define DEFAULT_FILTER            3276

#define DEFAULT_SPEED_COEFFICIENT 16384
#define DEFAULT_STEER_COEFFICIENT 0
// ######################### END OF DEFAULT SETTINGS ############################


// ############################## CRUISE CONTROL ############################
// Вимкнено
// #define CRUISE_CONTROL_SUPPORT
// ########################## END OF CRUISE CONTROL ##########################


// ############################### DEBUG SERIAL ###############################
// Підключи UART-USB 3.3V до правого роз'єму для калібрування:
//   GND -> GND правого роз'єму
//   RX адаптера -> TX плати (PB10 правого роз'єму)
//   115200 baud, 8N1
// УВАГА: DEBUG_SERIAL_USART3 і SUPPORT_BUTTONS_RIGHT — взаємовиключні!
// ########################### END OF DEBUG SERIAL ############################


// ############################### BUZZER ###############################
#define BUZZER_ENABLED
// ########################### END OF BUZZER ############################


// ################################# VARIANT_ADC SETTINGS ############################
#ifdef VARIANT_ADC
/*
 * ============================================================
 * СХЕМА ПІДКЛЮЧЕННЯ (лівий роз'єм):
 * ============================================================
 *  GND          ------- Крайній вивід 1 потенціометра газу
 *  PA3 (ADC2)   ------- Середній вивід (сигнал газу)
 *  3.3V         ------- Крайній вивід 2 потенціометра газу
 *
 *  PA2 (ADC1) — не підключено (INPUT1 вимкнено)
 *
 * ============================================================
 * ТУМБЛЕР РЕВЕРСУ (правий роз'єм):
 * ============================================================
 *  GND        -> один контакт тумблера
 *  PB11 (RX)  -> другий контакт тумблера
 *  Розімкнений = ВПЕРЕД, Замкнений на GND = НАЗАД
 *
 *  SUPPORT_BUTTONS_RIGHT — ВИМКНЕНО (тумблер керується напряму з main.c)
 *  Це дозволяє одночасно мати і тумблер реверсу, і DEBUG на PB10 (TX).
 * ============================================================
 */

  // Напрямок моторів керується через DIR_LEFT / DIR_RIGHT в main.c
  // #define INVERT_R_DIRECTION  -- НЕ використовувати, може конфліктувати

  #define CONTROL_ADC           0

  #define FLASH_WRITE_KEY       0x1013    // збільшено -> скидає стару калібровку flash

  // INPUT1 (PA2/ADC1) — ВИМКНЕНО
  // INPUT2 (PA3/ADC2) — ГАЗ
  //   MIN=50   -> ADC Protection спрацює при обриві
  //   MAX=3900 -> трохи нижче максимуму
  //   DEADBAND=100 -> авто стоїть поки педаль не натиснута
  #define PRI_INPUT1            0,    0, 0,    0,   0   // ВИМКНЕНО
  #define PRI_INPUT2            1,   50, 0, 3900, 100   // ГАЗ: Normal Pot

  // Обидва мотори — однакова команда (без рульового)
  #define TANK_STEERING

  // SUPPORT_BUTTONS_RIGHT — ВИМКНЕНО!
  // Тумблер реверсу підключений до PB11 і керується напряму з main.c.
  // Якщо розкоментувати — HAL перепише налаштування PB11 і тумблер не працюватиме.
  // #define SUPPORT_BUTTONS_RIGHT

  // Debug UART (PB10/TX правого роз'єму) — можна увімкнути одночасно з тумблером!
  // Розкоментуй для калібрування батареї та перевірки реверсу:
  // #define DEBUG_SERIAL_USART3

#endif
// ############################# END OF VARIANT_ADC SETTINGS #########################


// ############################ VARIANT_USART SETTINGS ############################
#ifdef VARIANT_USART
  #define CONTROL_SERIAL_USART2  0
  #define FEEDBACK_SERIAL_USART2
  #define PRI_INPUT1             3, -1000, 0, 1000, 0
  #define PRI_INPUT2             3, -1000, 0, 1000, 0
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY      0x1102
    #define SIDEBOARD_SERIAL_USART3 1
    #define AUX_INPUT1           3, -1000, 0, 1000, 0
    #define AUX_INPUT2           3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY      0x1002
  #endif
#endif
// ######################## END OF VARIANT_USART SETTINGS #########################


// ################################# VARIANT_NUNCHUK SETTINGS ############################
#ifdef VARIANT_NUNCHUK
  #define CONTROL_NUNCHUK         0
  #define PRI_INPUT1              2, -1024, 0, 1024, 0
  #define PRI_INPUT2              2, -1024, 0, 1024, 0
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1103
    #define CONTROL_SERIAL_USART2 1
    #define FEEDBACK_SERIAL_USART2
    #define AUX_INPUT1            3, -1000, 0, 1000, 0
    #define AUX_INPUT2            3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY       0x1003
    #define DEBUG_SERIAL_USART2
  #endif
  #define FILTER                  3276
  #define SPEED_COEFFICIENT       8192
  #define STEER_COEFFICIENT       62259
#endif
// ############################# END OF VARIANT_NUNCHUK SETTINGS #########################


// ################################# VARIANT_PPM SETTINGS ##############################
#ifdef VARIANT_PPM
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1104
    #define CONTROL_ADC           0
    #define CONTROL_PPM_RIGHT     1
    #define PRI_INPUT1            3,     0, 0, 4095,   0
    #define PRI_INPUT2            3,     0, 0, 4095,   0
    #define AUX_INPUT1            3, -1000, 0, 1000, 100
    #define AUX_INPUT2            3, -1000, 0, 1000, 100
  #else
    #define FLASH_WRITE_KEY       0x1004
    #define CONTROL_PPM_RIGHT     0
    #define PRI_INPUT1            3, -1000, 0, 1000, 100
    #define PRI_INPUT2            3, -1000, 0, 1000, 100
  #endif
  #define PPM_NUM_CHANNELS        6
  #if defined(CONTROL_PPM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(CONTROL_PPM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_PPM SETTINGS ############################


// ################################# VARIANT_PWM SETTINGS ##############################
#ifdef VARIANT_PWM
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1105
    #define CONTROL_ADC           0
    #define CONTROL_PWM_RIGHT     1
    #define PRI_INPUT1            3,     0, 0, 4095,   0
    #define PRI_INPUT2            3,     0, 0, 4095,   0
    #define AUX_INPUT1            3, -1000, 0, 1000, 100
    #define AUX_INPUT2            3, -1000, 0, 1000, 100
  #else
    #define FLASH_WRITE_KEY       0x1005
    #define CONTROL_PWM_RIGHT     0
    #define PRI_INPUT1            3, -1000, 0, 1000, 100
    #define PRI_INPUT2            3, -1000, 0, 1000, 100
  #endif
  #define FILTER                  6553
  #define SPEED_COEFFICIENT       16384
  #define STEER_COEFFICIENT       16384
  #if defined(CONTROL_PWM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(CONTROL_PWM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_PWM SETTINGS ############################


// ################################# VARIANT_IBUS SETTINGS ##############################
#ifdef VARIANT_IBUS
  #define CONTROL_IBUS
  #define IBUS_NUM_CHANNELS       14
  #define IBUS_LENGTH             0x20
  #define IBUS_COMMAND            0x40
  #define USART3_BAUD             115200
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1106
    #define CONTROL_ADC           0
    #define CONTROL_SERIAL_USART3 1
    #define FEEDBACK_SERIAL_USART3
    #define PRI_INPUT1            3,     0, 0, 4095, 0
    #define PRI_INPUT2            3,     0, 0, 4095, 0
    #define AUX_INPUT1            3, -1000, 0, 1000, 0
    #define AUX_INPUT2            3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY       0x1006
    #define CONTROL_SERIAL_USART3 0
    #define FEEDBACK_SERIAL_USART3
    #define PRI_INPUT1            3, -1000, 0, 1000, 0
    #define PRI_INPUT2            3, -1000, 0, 1000, 0
  #endif
  #if defined(CONTROL_SERIAL_USART3) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(DEBUG_SERIAL_USART2) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_IBUS SETTINGS ############################


// ############################ VARIANT_HOVERCAR SETTINGS ############################
#ifdef VARIANT_HOVERCAR
  #define FLASH_WRITE_KEY         0x1107
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ            VLT_MODE
  #define CONTROL_ADC             0
  #define SIDEBOARD_SERIAL_USART3 1
  #define FEEDBACK_SERIAL_USART3
  #define DUAL_INPUTS
  #define PRI_INPUT1              1,  1000, 0, 2500, 0
  #define PRI_INPUT2              1,   500, 0, 2200, 0
  #define AUX_INPUT1              2, -1000, 0, 1000, 0
  #define AUX_INPUT2              2, -1000, 0, 1000, 0
  #define SPEED_COEFFICIENT       16384
  #define STEER_COEFFICIENT       8192
  #define MULTI_MODE_DRIVE
  #ifdef MULTI_MODE_DRIVE
    #define MULTI_MODE_DRIVE_M1_MAX   175
    #define MULTI_MODE_DRIVE_M1_RATE  250
    #define MULTI_MODE_M1_I_MOT_MAX   4
    #define MULTI_MODE_M1_N_MOT_MAX   30
    #define MULTI_MODE_DRIVE_M2_MAX   500
    #define MULTI_MODE_DRIVE_M2_RATE  300
    #define MULTI_MODE_M2_I_MOT_MAX   8
    #define MULTI_MODE_M2_N_MOT_MAX   80
    #define MULTI_MODE_DRIVE_M3_MAX   1000
    #define MULTI_MODE_DRIVE_M3_RATE  450
    #define MULTI_MODE_M3_I_MOT_MAX   I_MOT_MAX
    #define MULTI_MODE_M3_N_MOT_MAX   N_MOT_MAX
  #endif
#endif
#define MULTIPLE_TAP_NR           2 * 2
#define MULTIPLE_TAP_HI           600
#define MULTIPLE_TAP_LO           200
#define MULTIPLE_TAP_TIMEOUT      2000
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################


// ############################ VARIANT_HOVERBOARD SETTINGS ############################
#ifdef VARIANT_HOVERBOARD
  #define FLASH_WRITE_KEY     0x1008
  #define SIDEBOARD_SERIAL_USART2 1
  #define FEEDBACK_SERIAL_USART2
  #define SIDEBOARD_SERIAL_USART3 0
  #define FEEDBACK_SERIAL_USART3
  #define PRI_INPUT1          3, -1000, 0, 1000, 0
  #define PRI_INPUT2          3, -1000, 0, 1000, 0
  #define AUX_INPUT1          3, -1000, 0, 1000, 0
  #define AUX_INPUT2          3, -1000, 0, 1000, 0
#endif
// ######################## END OF VARIANT_HOVERBOARD SETTINGS #########################


// ################################# VARIANT_TRANSPOTTER SETTINGS ############################
#ifdef VARIANT_TRANSPOTTER
  #define FLASH_WRITE_KEY     0x1009
  #define CONTROL_GAMETRAK
  #define SUPPORT_LCD
  #define GAMETRAK_CONNECTION_NORMAL
  #define ROT_P               1.2
  #define SPEED_COEFFICIENT   14746
  #define STEER_COEFFICIENT   8192
  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
  #define PRI_INPUT1          2, -1000, 0, 1000, 0
  #define PRI_INPUT2          2, -1000, 0, 1000, 0
#endif
// ############################# END OF VARIANT_TRANSPOTTER SETTINGS ########################


// ################################# VARIANT_SKATEBOARD SETTINGS ##############################
#ifdef VARIANT_SKATEBOARD
  #define FLASH_WRITE_KEY     0x1010
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ        TRQ_MODE
  #define CONTROL_PWM_RIGHT   0
  #define PRI_INPUT1          0, -1000, 0, 1000,   0
  #define PRI_INPUT2          2,  -800, 0,  700, 100
  #define INPUT_BRK           -400
  #define FILTER              6553
  #define SPEED_COEFFICIENT   16384
  #define STEER_COEFFICIENT   0
  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
  #ifdef CONTROL_PWM_RIGHT
    #define DEBUG_SERIAL_USART2
  #else
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_SKATEBOARD SETTINGS ############################


// ########################### UART SETTINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD
  #define SERIAL_BUFFER_SIZE      64
  #define SERIAL_TIMEOUT          160
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           115200
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B
#endif
#if defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #ifndef USART3_BAUD
    #define USART3_BAUD           115200
  #endif
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B
#endif
// ########################### END OF UART SETTINGS ############################


// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
  #define INPUTS_NR               2
#else
  #define INPUTS_NR               1
#endif
// ########################### END OF APPLY DEFAULT SETTINGS ############################


// ############################### VALIDATE SETTINGS ###############################
#if !defined(VARIANT_ADC) && !defined(VARIANT_USART) && !defined(VARIANT_NUNCHUK) && !defined(VARIANT_PPM) && !defined(VARIANT_PWM) && \
    !defined(VARIANT_IBUS) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER) && !defined(VARIANT_SKATEBOARD)
  #error Variant not defined! Please check platformio.ini or Inc/config.h for available variants.
#endif
#if defined(CONTROL_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART2)
  #error CONTROL_SERIAL_USART2 and SIDEBOARD_SERIAL_USART2 not allowed, choose one.
#endif
#if defined(CONTROL_SERIAL_USART3) && defined(SIDEBOARD_SERIAL_USART3)
  #error CONTROL_SERIAL_USART3 and SIDEBOARD_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART2)
  #error DEBUG_SERIAL_USART2 and FEEDBACK_SERIAL_USART2 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART3) && defined(FEEDBACK_SERIAL_USART3)
  #error DEBUG_SERIAL_USART3 and FEEDBACK_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART2) && defined(DEBUG_SERIAL_USART3)
  #error DEBUG_SERIAL_USART2 and DEBUG_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PPM_RIGHT)
  #error CONTROL_PPM_LEFT and CONTROL_PPM_RIGHT not allowed, choose one.
#endif
#if defined(CONTROL_PWM_LEFT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PWM_LEFT and CONTROL_PWM_RIGHT not allowed, choose one.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && defined(SUPPORT_BUTTONS_RIGHT)
  #error SUPPORT_BUTTONS_LEFT and SUPPORT_BUTTONS_RIGHT not allowed, choose one.
#endif
#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PPM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PWM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PWM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error SUPPORT_BUTTONS_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_ADC) || defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error SUPPORT_BUTTONS_LEFT and (CONTROL_ADC or CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_ADC) && (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error CONTROL_ADC and (CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PWM_LEFT)
  #error CONTROL_PPM_LEFT and CONTROL_PWM_LEFT not allowed. It is on the same cable.
#endif
#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_NUNCHUK and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PPM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PWM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PWM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error SUPPORT_BUTTONS_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error SUPPORT_BUTTONS_RIGHT and (CONTROL_NUNCHUK or CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error CONTROL_NUNCHUK and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif
#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT))
  #error DEBUG_I2C_LCD and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_RIGHT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PPM_RIGHT and CONTROL_PWM_RIGHT not allowed. It is on the same cable.
#endif
#if (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)) && !defined(PPM_NUM_CHANNELS)
  #error Total number of PPM channels needs to be set
#endif
// ############################# END OF VALIDATE SETTINGS ############################

#endif