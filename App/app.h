/*
 * app.h
 *
 *  Created on: Oct 29, 2024
 *      Author: joaopedro
 */

#ifndef APP_H_
#define APP_H_

#include "main.h"

#define Clock_Frequency 32768
#define PWM_VALUE 2048
#define BUTTON_DELAY 100

typedef uint8_t bool;

typedef struct {
    uint8_t seconds_units;
    uint8_t seconds_tens;
    uint8_t minutes_units;
    uint8_t minutes_tens;
    uint8_t hours_units;
    uint8_t hours_tens;
} TimeStruct;

typedef enum {
    MODE_CONFIG_CLOCK,
    MODE_CONFIG_ALARM,
    MODE_RUNNING,
} ConfigMode;


// main functions
void setup();
void loop();


// clock
void incrementSeconds_units();
void incrementSeconds_decs();
void incrementMinutes_units();
void incrementMinutes_decs();
void incrementHours_units();
void incrementHours_decs();


// alarm
void incrementAlarmMinutes_units();
void incrementAlarmMinutes_decs();
void incrementAlarmHours_units();
void incrementAlarmHours_decs();


// auxiliary functions
void updateDisplay(TimeStruct time);
void playAlarm(TIM_HandleTypeDef *htimPWM);
void readButtons();


// TLC5947 functions
void __TLC5947_SendGSData(uint32_t pwmval);
void TLC5947_SendData(uint8_t *powerBuffer);


#endif /* APP_H_ */
