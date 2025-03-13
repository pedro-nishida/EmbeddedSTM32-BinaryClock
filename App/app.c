/*
 * app.c
 *
 *  Created on: Oct 29, 2024
 *      Author: joaopedro
 */

#include "app.h"

TimeStruct currentTime = {0, 0, 0, 0, 0, 0}; // initializes currentTime at 00:00:00
TimeStruct alarmTime = {0, 0, 0, 0, 0, 0};
ConfigMode currentMode = MODE_RUNNING;
bool anyButtonPressed;

void setup(TIM_HandleTypeDef *htimPWM) {
	HAL_TIM_PWM_Init(htimPWM);				// initializes timer for PWM
}

void loop(TIM_HandleTypeDef *htimPWM) {
	anyButtonPressed = 0;
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(SEC_UNID_GPIO_Port, SEC_UNID_Pin);
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(SEC_DEZ_GPIO_Port, SEC_DEZ_Pin);
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(MIN_UNID_GPIO_Port, MIN_UNID_Pin);
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(MIN_DEZ_GPIO_Port, MIN_DEZ_Pin);
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(HOUR_UNID_GPIO_Port, HOUR_UNID_Pin);
	anyButtonPressed = anyButtonPressed || HAL_GPIO_ReadPin(HOUR_DEZ_GPIO_Port, HOUR_DEZ_Pin);

	if (anyButtonPressed) readButtons();

	if (currentMode == MODE_RUNNING) updateDisplay(currentTime);
	playAlarm(htimPWM);						// VERIFICAR: horario congelado durante alarme
											// VERIFICAR: tentar rodar o stm no clock de baixa velocidade
}


// seconds
void incrementSeconds_units() {
    currentTime.seconds_units = (currentTime.seconds_units + 1) % 10;
    HAL_Delay(BUTTON_DELAY);
}

void incrementSeconds_tens() {
    currentTime.seconds_tens = (currentTime.seconds_tens + 1) % 6;
    HAL_Delay(BUTTON_DELAY);
}


// minutes
void incrementMinutes_units() {
    currentTime.minutes_units = (currentTime.minutes_units + 1) % 10;
    HAL_Delay(BUTTON_DELAY);
}

void incrementMinutes_tens() {
    currentTime.minutes_tens = (currentTime.minutes_tens + 1) % 6;
    HAL_Delay(BUTTON_DELAY);
}


// hours
void incrementHours_units() {
	if (currentTime.hours_tens == 2)
		currentTime.hours_units = (currentTime.hours_units + 1) % 4;  // Limite de 24 horas
	else
		currentTime.hours_units = (currentTime.hours_units + 1) % 10;
	HAL_Delay(BUTTON_DELAY);
}

void incrementHours_tens() {
    currentTime.hours_tens = (currentTime.hours_tens + 1) % 3;  // Limite de 24 horas
    HAL_Delay(BUTTON_DELAY);
}


// minutes (alarm)
void incrementAlarmMinutes_units() {
	alarmTime.minutes_units = (alarmTime.minutes_units + 1) % 10;
	HAL_Delay(BUTTON_DELAY);
}

void incrementAlarmMinutes_tens() {
	alarmTime.minutes_tens = (alarmTime.minutes_tens + 1) % 6;
	HAL_Delay(BUTTON_DELAY);
}


// hours (alarm)
void incrementAlarmHours_units() {
	if (alarmTime.hours_tens == 2)
		alarmTime.hours_units = (alarmTime.hours_units + 1) % 4;  // Limite de 24 horas
	else
		alarmTime.hours_units = (alarmTime.hours_units + 1) % 10;
	HAL_Delay(BUTTON_DELAY);
}

void incrementAlarmHours_tens() {
	alarmTime.hours_tens = (alarmTime.hours_tens + 1) % 3;  // Limite de 24 horas
	HAL_Delay(BUTTON_DELAY);
}




void updateDisplay(TimeStruct time) {
	uint8_t ledBuffer[20];

	// hours
	ledBuffer[0]  = time.hours_tens >> 1;
	ledBuffer[1]  = time.hours_tens & ~(1 << 1);

	ledBuffer[2]  = time.hours_units >> 3;
	ledBuffer[3]  = (time.hours_units & ~(0b1 << 3)) >> 2;
	ledBuffer[4]  = (time.hours_units & ~(0b11 << 2)) >> 1;
	ledBuffer[5]  = time.hours_units & ~(0b111 << 1);


	// minutes
	ledBuffer[6]  = time.minutes_tens >> 2;
	ledBuffer[7]  = (time.minutes_tens & ~(0b1 << 2)) >> 1;
	ledBuffer[8]  = (time.minutes_tens & ~(0b11 << 1));

	ledBuffer[9]  = time.minutes_units >> 3;
	ledBuffer[10] = (time.minutes_units & ~(0b1 << 3)) >> 2;
	ledBuffer[11] = (time.minutes_units & ~(0b11 << 2)) >> 1;
	ledBuffer[12] = time.minutes_units & ~(0b111 << 1);


	// seconds
	ledBuffer[13] = time.minutes_tens >> 2;
	ledBuffer[14] = (time.minutes_tens & ~(0b1 << 2)) >> 1;
	ledBuffer[15] = (time.minutes_tens & ~(0b11 << 1));

	ledBuffer[16] = time.minutes_units >> 3;
	ledBuffer[17] = (time.minutes_units & ~(0b1 << 3)) >> 2;
	ledBuffer[18] = (time.minutes_units & ~(0b11 << 2)) >> 1;
	ledBuffer[19] = time.minutes_units & ~(0b111 << 1);

	TLC5947_SendData(ledBuffer);
}



void playAlarm(TIM_HandleTypeDef *htimPWM) {
	if (currentTime.hours_tens == alarmTime.hours_tens && currentTime.hours_units == alarmTime.hours_units &&
	    currentTime.minutes_tens == alarmTime.minutes_tens && currentTime.minutes_units == alarmTime.minutes_units) {

		HAL_TIM_PWM_Start(htimPWM, TIM_CHANNEL_1);	// alarm started
	    HAL_Delay(60000);							// alarm sound duration
	    HAL_TIM_PWM_Stop(htimPWM, TIM_CHANNEL_1);	// alarm ended
	}
}



void readButtons() {
	currentMode = HAL_GPIO_ReadPin(SEL_MODE_GPIO_Port, SEL_MODE_Pin) ? MODE_CONFIG_ALARM : MODE_CONFIG_CLOCK;

	if (currentMode == MODE_CONFIG_CLOCK) {
		while (1) {
			if (HAL_GPIO_ReadPin(SEC_UNID_GPIO_Port, SEC_UNID_Pin)) incrementSeconds_units();
			if (HAL_GPIO_ReadPin(SEC_DEZ_GPIO_Port, SEC_DEZ_Pin)) incrementSeconds_tens();
			if (HAL_GPIO_ReadPin(MIN_UNID_GPIO_Port, MIN_UNID_Pin)) incrementMinutes_units();
			if (HAL_GPIO_ReadPin(MIN_DEZ_GPIO_Port, MIN_DEZ_Pin)) incrementMinutes_tens();
			if (HAL_GPIO_ReadPin(HOUR_UNID_GPIO_Port, HOUR_UNID_Pin)) incrementHours_units();
			if (HAL_GPIO_ReadPin(HOUR_DEZ_GPIO_Port, HOUR_DEZ_Pin)) incrementHours_tens();
			updateDisplay(currentTime);

            if (HAL_GPIO_ReadPin(CONFIRM_GPIO_Port, CONFIRM_Pin)) break;
        }
    } else if (currentMode == MODE_CONFIG_ALARM) {
        while (1) {
            if (HAL_GPIO_ReadPin(MIN_UNID_GPIO_Port, MIN_UNID_Pin)) incrementAlarmMinutes_units();
            if (HAL_GPIO_ReadPin(MIN_DEZ_GPIO_Port, MIN_DEZ_Pin)) incrementAlarmMinutes_tens();
            if (HAL_GPIO_ReadPin(HOUR_UNID_GPIO_Port, HOUR_UNID_Pin)) incrementAlarmHours_units();
            if (HAL_GPIO_ReadPin(HOUR_DEZ_GPIO_Port, HOUR_DEZ_Pin)) incrementAlarmHours_tens();
            updateDisplay(alarmTime);

            if (HAL_GPIO_ReadPin(CONFIRM_GPIO_Port, CONFIRM_Pin)) break;
        }
    }

    currentMode = MODE_RUNNING;
}



void __TLC5947_SendGSData(uint32_t pwmval) {
	if (pwmval >= (1 << 24)) return;							// error (overflow)

	HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);


	for (int i = 11; i >= 0; i--) {
		HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, pwmval / (1 << i));
		pwmval &= ~(1 << i);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
	}
}


void TLC5947_SendData(uint8_t *ledBuffer) {
	HAL_GPIO_WritePin(XLAT_GPIO_Port, XLAT_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 4; i++) __TLC5947_SendGSData(0);
	for (int i = 19; i >= 0; i--) {
		if (ledBuffer[i] > 1) return;							// error (undesired value)
		__TLC5947_SendGSData(PWM_VALUE * ledBuffer[i]);
	}
	HAL_GPIO_WritePin(XLAT_GPIO_Port, XLAT_Pin, GPIO_PIN_SET);
}

