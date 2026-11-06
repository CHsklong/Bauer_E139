#ifndef GPIO_H
#define GPIO_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "mcdrv.h"

#if (BOARD_TYPE == BOARD_E139 ||BOARD_TYPE == BOARD_E884)
#define LED_PIN_NUM 9 // PA09
#define SYS_LED_HIGH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM, 1);
#define SYS_LED_LOW() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM, 0);
#define SYS_LED_TOG() gpio_toggle_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM);
#define ABS_CAL_START() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 25, 1);
#define ABS_CAL_FINISH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 25, 0);
#elif (BOARD_TYPE == BOARD_E914)
#define LED_PIN_NUM 11 // PB11
#define SYS_LED_HIGH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOB, LED_PIN_NUM, 1);
#define SYS_LED_LOW() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOB, LED_PIN_NUM, 0);
#define SYS_LED_TOG() gpio_toggle_pin(HPM_GPIO0, GPIO_DO_GPIOB, LED_PIN_NUM);
#define ABS_CAL_START() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 30, 1);
#define ABS_CAL_FINISH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 30, 0);
#elif (BOARD_TYPE == BOARD_E145)
#define LED_PIN_NUM 10 // PA10
#define SYS_LED_HIGH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM, 1);
#define SYS_LED_LOW() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM, 0);
#define SYS_LED_TOG() gpio_toggle_pin(HPM_GPIO0, GPIO_DO_GPIOA, LED_PIN_NUM);
#define ABS_CAL_START() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 02, 1);
#define ABS_CAL_FINISH() gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 02, 0);

#define BOARD_E145_GET_VER()  (1)
#endif

/*
#define  UPIN_NUM 23
#define  VPIN_NUM 24
#define  WPIN_NUM 25


//#define     ENCODE_U()
//PB01
#define		ZL_OUT_ON()		gpio_write_pin(HPM_GPIO0 ,GPIO_DO_GPIOB, 1,1);
#define		ZL_OUT_OFF()		gpio_write_pin(HPM_GPIO0 ,GPIO_DO_GPIOB, 1,0);

//PA25


//PB00
#define 	ZL_ALARM			(gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOB, 0))
*/
extern int8_t over_current_flag;

void can_gpio_init(void);
void led_gpio_init(void);
void uvw_init(void);
void io_gpio_init(void);
void adc_pins_init_analog(void);
void spi_gpio_init(void);

#endif