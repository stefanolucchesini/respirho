#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"

#define LED 11

void TIMER0_IRQHandler()
{
    if (NRF_TIMER0->EVENTS_COMPARE[0] != 0)
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
			  nrf_gpio_pin_toggle(LED);
    }
}

void timer_config(void)
{
    NRF_TIMER0->TASKS_STOP = 1; // Stop timer
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // taken from Nordic dev zone
    NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER0->PRESCALER = 8; // 1us resolution
    NRF_TIMER0->TASKS_CLEAR = 1; // Clear timer
    NRF_TIMER0->CC[0] = 62500;
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; // taken from Nordic dev zone
    NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    //attachInterrupt(TIMER1_IRQn, TIMER1_Interrupt); // also used in variant.cpp to configure the RTC1
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START = 1; // Start TIMER
}

void setup() 
{
    timer_config();
}

int main(void)
{
    nrf_gpio_cfg_output(LED);
		nrf_gpio_pin_clear(LED);
	  setup();
    while (1) nrf_pwr_mgmt_run();;
}