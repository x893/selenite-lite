#ifndef BOARD_H_
#define BOARD_H_

#include "stm32f4xx_hal.h"

#define LED_PIN		0
#define LED_PORT	0

#define BUTTON_PIN	0
#define BUTTON_PORT	0
#define BUTTON_STATE_ACTIVE	0
#define LED_STATE_ON		0

#ifdef __cplusplus
 extern "C" {
#endif

typedef void*	UART_HandleTypeDef;

static inline void board_clock_init(void) { }
static inline void board_vbus_sense_init(void) { }

#ifdef __cplusplus
 }
#endif

#endif /* BOARD_H_ */
