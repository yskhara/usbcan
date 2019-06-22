//
// LED: Handles blinking of status light
//

#include "main.h"
#include "led.h"

static uint32_t led_laston = 0;
static uint32_t led_lastoff = 0;

static uint32_t led_laston_stat = 0;
static uint32_t led_lastoff_stat = 0;

// Attempt to turn on status LED
void led_on(void)
{
	// Make sure the LED has been off for at least LED_DURATION before turning on again
	// This prevents a solid status LED on a busy canbus
	if(led_laston == 0 && HAL_GetTick() - led_lastoff > LED_DURATION)
	{
        GPIO_LEDCAN->BSRR = GPIO_BSRR_BS_LEDCAN;
        //GPIOC->BSRR = GPIO_BSRR_BS13;
		led_laston = HAL_GetTick();
	}
}


// Process time-based LED events
void led_process(void)
{
	// If LED has been on for long enough, turn it off
	if(led_laston > 0 && HAL_GetTick() - led_laston > LED_DURATION)
	{
	    GPIO_LEDCAN->BSRR = GPIO_BSRR_BR_LEDCAN;
        //GPIOC->BSRR = GPIO_BSRR_BR13;
		led_laston = 0;
		led_lastoff = HAL_GetTick();
	}

    if(led_laston_stat > 0 && HAL_GetTick() - led_laston_stat > LED_STAT_ON_DUR)
    {
        GPIO_LED0->BSRR = GPIO_BSRR_BR_LED0;
        //GPIOB->BSRR = GPIO_BSRR_BR0;
        led_laston_stat = 0;
        led_lastoff_stat = HAL_GetTick();
    }

    if(led_laston_stat == 0 && HAL_GetTick() - led_lastoff_stat > LED_STAT_OFF_DUR)
    {
        GPIO_LED0->BSRR = GPIO_BSRR_BS_LED0;
        //GPIOB->BSRR = GPIO_BSRR_BS0;
        led_laston_stat = HAL_GetTick();
    }
}

