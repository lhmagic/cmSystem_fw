#ifndef		__BSP_H__
#define		__BSP_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

#define		HW_VER									"A.0"
#define		FW_VER									"1.2.3"

#define		DEBUG_BUFF_MAX					32
#define		RS485_BUFF_MAX					32
#define		FILTER_BUFF_SIZE				32

#define		parama_save_addr				0xFF00

typedef	struct {
	int16_t pres1_k;
	int16_t pres1_b;
	int16_t pres1_max;
	int16_t pres1_min;
	int16_t pres2_k;
	int16_t pres2_b;	
	int16_t pres2_max;
	int16_t pres2_min;
	int8_t 	cs1_switch;
	int8_t 	cs2_switch;
} s_sys_para;

typedef union {
	uint16_t val;
	struct {
		uint8_t sys_err			: 1;	
		uint8_t pump_pwr		: 1;
		uint8_t pump1_red		: 1;
		uint8_t pump1_green	: 1;
		uint8_t pump2_red		: 1;
		uint8_t pump2_green	: 1;
		uint8_t pump3_red		: 1;
		uint8_t pump3_green	: 1;
		uint8_t volt_in2		: 1;
		uint8_t volt_in1		: 1;
		uint8_t curr_in2		: 1;
		uint8_t curr_in1		: 1;
		uint8_t 						: 4;
	};
} u_led;

typedef union {
	uint16_t val;
	struct {
		uint8_t ac_pwr			: 1;
		uint8_t ac_sw1			: 1;
		uint8_t ac_sw2			: 1;
		uint8_t ac_sw3			: 1;
		uint8_t pump1_start	: 1;
		uint8_t pump1_stop	: 1;
		uint8_t pump1_fail	: 1;
		uint8_t pump2_start	: 1;
		uint8_t pump2_stop	: 1;
		uint8_t pump2_fail	: 1;		
		uint8_t pump3_start	: 1;	
		uint8_t pump3_stop	: 1;	
		uint8_t 						: 4;
	};
} u_ac_state;

typedef union {
	uint16_t val;
	struct {
		uint8_t di_ext_in1	: 1;
		uint8_t di_ext_in2	: 1;
		uint8_t 						: 6;
	};
} u_dc_state;

u_led hc595_init(void);
void hc595_write(uint16_t val);
uint8_t get_rs485_addr(void);
u_ac_state get_ac_state(void);
u_dc_state get_dc_state(void);
uint16_t mb_crc16(const uint8_t *buff, uint8_t len);

void flash_write(uint32_t addr, uint8_t *data, uint16_t len) ;
void set_rop(void);

#endif	// __BSP_H__

