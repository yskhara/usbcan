#ifndef _LED_H
#define _LED_H

#define GPIO_LED0 GPIOB

#define GPIO_BSRR_BS_LED0 GPIO_BSRR_BS_5
#define GPIO_BSRR_BR_LED0 GPIO_BSRR_BR_5

#define GPIO_LED1 GPIOB

#define GPIO_BSRR_BS_LED1 GPIO_BSRR_BS_6
#define GPIO_BSRR_BR_LED1 GPIO_BSRR_BR_6

#define GPIO_LED2 GPIOB

#define GPIO_BSRR_BS_LED2 GPIO_BSRR_BS_7
#define GPIO_BSRR_BR_LED2 GPIO_BSRR_BR_7

#define GPIO_LEDCAN GPIOB

#define GPIO_BSRR_BS_LEDCAN GPIO_BSRR_BS_4
#define GPIO_BSRR_BR_LEDCAN GPIO_BSRR_BR_4


#ifdef __cplusplus
 extern "C" {
#endif

#define LED_DURATION 25

#define LED_STAT_ON_DUR 1900
#define LED_STAT_OFF_DUR 100

void led_on(void);
void led_process(void);

#ifdef __cplusplus
}
#endif

#endif
