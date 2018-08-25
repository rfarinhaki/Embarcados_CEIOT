#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "ti-lib.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "lpm.h"
#include <stdio.h>

#define POT (ADC_COMPB_IN_AUXIO0)
//#define LED (IOID_21)
#define LED (IOID_6)
#define BUTTON_RIGHT (IOID_13)
#define BUTTON_LEFT (IOID_14)

#define BUTTON_UP_EVENT (51)
#define BUTTON_DOWN_EVENT (52)

#define PRESSED 1

static struct etimer et_button;

PROCESS(pwm_process, "PWM Process");
PROCESS(button_process, "Button Process");

AUTOSTART_PROCESSES(&pwm_process, &button_process);

uint8_t pwm_request_max_pm(void)
{
    return LPM_MODE_DEEP_SLEEP;
}
void sleep_enter(void)
{
    //leds_on(LEDS_RED);
}
void sleep_leave(void)
{
    //leds_off(LEDS_RED);
}
LPM_MODULE(pwmdrive_module, pwm_request_max_pm,
           sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

int16_t pwminit(int32_t freq){
    uint32_t load = 0;
    ti_lib_ioc_pin_type_gpio_output(LED);
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);

    /* Enable GPT0 clocks under active, sleep, deep sleep */
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);

    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* Register with LPM. This will keep the PERIPH PD powered on
     * during deep sleep, allowing the pwm to keep working while the chip is
     * being power-cycled */
    lpm_register_module(&pwmdrive_module);

    ti_lib_ioc_port_configure_set(LED, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

    ti_lib_timer_configure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    /* Stop the timers */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if(freq > 0) {
        load = (GET_MCU_CLOCK / freq);
        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load-1);
        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;

}


PROCESS_THREAD(pwm_process, ev, data)
{
    static int16_t current_duty = 100;
    static int16_t loadvalue;
    static int ticks;

    PROCESS_BEGIN();
    loadvalue = pwminit(5000);

    ticks = (current_duty * loadvalue) / 100;
    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
    while(1){
        PROCESS_WAIT_EVENT();

        if(ev == BUTTON_UP_EVENT){

            current_duty += 10;
            if(current_duty>100)
                current_duty=100;
            printf("Up [%d]\n", current_duty);
            ticks = (current_duty * loadvalue) / 100;
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
        }

        if(ev == BUTTON_DOWN_EVENT){
            current_duty -= 10;
            if(current_duty<=0)
                current_duty=1;
            printf("Down [%d]\n", current_duty);
            ticks = (current_duty * loadvalue) / 100;
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(button_process, ev, data)
{
    PROCESS_BEGIN();
    //IOCPinTypeGpioInput(BUTTON_RIGHT);
    //IOCIOPortPullSet(BUTTON_RIGHT, IOC_IOPULL_DOWN);
    //IOCPinTypeGpioInput(BUTTON_LEFT);
    //IOCIOPortPullSet(BUTTON_LEFT, IOC_IOPULL_DOWN);

    //etimer_set(&et_button, 1*CLOCK_SECOND/100);

    while(1){
        PROCESS_YIELD();

        //if(GPIO_readDio(BUTTON_LEFT) == PRESSED){
        //    process_post(&pwm_process, BUTTON_UP_EVENT, (void*)(&button_process));
        //}
        //if(GPIO_readDio(BUTTON_LEFT) == PRESSED){
        //    process_post(&pwm_process, BUTTON_DOWN_EVENT, (void*)(&button_process));
        //}

        if(ev == sensors_event){
            if(data == &button_left_sensor)
                process_post(&pwm_process, BUTTON_UP_EVENT, (void*)(&button_process));
            if(data == &button_right_sensor)
                process_post(&pwm_process, BUTTON_DOWN_EVENT, (void*)(&button_process));
        }


        //etimer_reset(&et_button);

    }
    PROCESS_END();
}
