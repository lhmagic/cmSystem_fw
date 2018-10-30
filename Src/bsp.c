#include "bsp.h"

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;

/***************************************************************************
function : 		hc595_init
description:	initialization hc595 controlled led structrue.
input:				none
output:				none
return:				u_led union structure.
 ***************************************************************************/
u_led hc595_init(void) {
static u_led led;
	
	HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, GPIO_PIN_RESET);
	
	led.val = 0;
	hc595_write(led.val);
	
	return led;
}

/***************************************************************************
function : 		get_ac_state
description:	get ac input on/off state.
input:				none
output:				none
return:				11 channel ac on/off state
 ***************************************************************************/
void hc595_write(uint16_t val) {
	HAL_GPIO_WritePin(LED_ST_GPIO_Port, LED_ST_Pin, GPIO_PIN_RESET);
	for(uint16_t i=0x8000; i!=0x0000; i>>=1) {
		HAL_GPIO_WritePin(LED_SH_GPIO_Port, LED_SH_Pin, GPIO_PIN_RESET);
		if(val&i) {
			HAL_GPIO_WritePin(LED_DS_GPIO_Port, LED_DS_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LED_SH_GPIO_Port, LED_DS_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(LED_SH_GPIO_Port, LED_SH_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(LED_ST_GPIO_Port, LED_ST_Pin, GPIO_PIN_SET);
}

uint8_t get_rs485_addr(void) {
uint8_t addr=0;
	
	addr |= (HAL_GPIO_ReadPin(DIP_SW0_GPIO_Port, DIP_SW3_Pin) == GPIO_PIN_RESET) ? 0x01 : 0x00;
	addr |= (HAL_GPIO_ReadPin(DIP_SW1_GPIO_Port, DIP_SW2_Pin) == GPIO_PIN_RESET) ? 0x02 : 0x00;
	addr |= (HAL_GPIO_ReadPin(DIP_SW2_GPIO_Port, DIP_SW1_Pin) == GPIO_PIN_RESET) ? 0x04 : 0x00;
	addr |= (HAL_GPIO_ReadPin(DIP_SW3_GPIO_Port, DIP_SW0_Pin) == GPIO_PIN_RESET) ? 0x08 : 0x00;
	return addr;
}

/***************************************************************************
function : 		get_ac_state
description:	get ac input on/off state.
input:				none
output:				none
return:				11 channel ac on/off state

			| LSB	| ... |	SW3 | SW2 | SW1 | P3_START | P3_STOP | P2_START | P2_STOP | P2_FAIL | P1_START | P1_STOP | P1_FAIL |
			------------| b10 | b9  | b8  | b7			 | b6			 | b5			  | b4      | b3      | b2       | b1      | b0      |
 ***************************************************************************/
u_ac_state get_ac_state(void) {
static u_ac_state state;
	
	state.val = 0;
	state.pump1_fail = (HAL_GPIO_ReadPin(PUMP1_FAIL_GPIO_Port, PUMP1_FAIL_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump1_stop = (HAL_GPIO_ReadPin(PUMP1_STOP_GPIO_Port, PUMP1_STOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump1_start = (HAL_GPIO_ReadPin(PUMP1_START_GPIO_Port, PUMP1_START_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump2_fail = (HAL_GPIO_ReadPin(PUMP2_FAIL_GPIO_Port, PUMP2_FAIL_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump2_stop = (HAL_GPIO_ReadPin(PUMP2_STOP_GPIO_Port, PUMP2_STOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump2_start = (HAL_GPIO_ReadPin(PUMP2_START_GPIO_Port, PUMP2_START_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump3_stop = (HAL_GPIO_ReadPin(PUMP3_STOP_GPIO_Port, PUMP3_STOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.pump3_start = (HAL_GPIO_ReadPin(PUMP3_START_GPIO_Port, PUMP3_START_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.ac_sw1 = (HAL_GPIO_ReadPin(DI_SW1_GPIO_Port, DI_SW1_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.ac_sw2 = (HAL_GPIO_ReadPin(DI_SW2_GPIO_Port, DI_SW2_Pin) == GPIO_PIN_RESET)  ? 1 : 0;
	state.ac_sw3 = (HAL_GPIO_ReadPin(DI_SW3_GPIO_Port, DI_SW3_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	
	return state;
}

u_dc_state get_dc_state(void) {
static u_dc_state state;
	
	state.val = 0;	
	state.di_ext_in1 = (HAL_GPIO_ReadPin(DI_EXT_IN1_GPIO_Port, DI_EXT_IN1_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	state.di_ext_in2 = (HAL_GPIO_ReadPin(DI_EXT_IN2_GPIO_Port, DI_EXT_IN2_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	
	return state;
}

uint16_t mb_crc16(const uint8_t *buff, uint8_t len) {
uint16_t i, j;
uint16_t c, crc = 0xFFFF;
		for (i = 0; i < len; i++) {
			c = buff[i];
			crc ^= c;
			for (j = 0; j < 8; j++) {
				if ((crc & 0x0001) != 0) {
					crc >>= 1;
					crc ^= 0xA001;
				} else
					crc >>= 1;
			}
		}
		return (crc);
}

