#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "ti-lib.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "lpm.h"
#include <stdio.h>

//GPIO
#define POT (ADC_COMPB_IN_AUXIO0)
#define PER (ADC_COMPB_IN_AUXIO1)
#define RED_LED (IOID_6)
#define GREEN_LED (IOID_7)
#define BUTTON_RIGHT (IOID_13)
#define BUTTON_LEFT (IOID_14)

//Events
#define BUTTON_UP_EVENT (51)
#define BUTTON_DOWN_EVENT (52)
#define BRIGHTNESS_CHANGE_EVENT (53)
#define PERIOD_CHANGE_EVENT (54)

// Constants
#define PRESSED 1

//Timers
static struct etimer et_adc;
static struct etimer et_period;
static struct etimer et_led;


PROCESS(pwm_process, "PWM Process");
PROCESS(button_process, "Button Process");
PROCESS(readAdc_process, "adc Process");
PROCESS(readPeriod_process, "read period process");
PROCESS(green_led_process, "Green Led process");


AUTOSTART_PROCESSES(&pwm_process, &button_process, &readAdc_process, &readPeriod_process, &green_led_process);

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
    ti_lib_ioc_pin_type_gpio_output(RED_LED);
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

    ti_lib_ioc_port_configure_set(RED_LED, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

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
    loadvalue = pwminit(4000);

    ticks = (current_duty * loadvalue) / 100;
    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
    while(1){
        PROCESS_WAIT_EVENT();

        if(ev == BRIGHTNESS_CHANGE_EVENT){
            current_duty = (int) data;

            if(current_duty >100)
                current_duty = 100;
            if(current_duty<1)
                current_duty = 1;

            ticks = (current_duty * loadvalue) / 100;
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
        }
    }

    PROCESS_END();
}

PROCESS_THREAD(readAdc_process, ev, data)
{
    static int oldAdcValue=0, adcValue = 0;
    static struct sensors_sensor *sensor;
    sensor = sensors_find(ADC_SENSOR);

    PROCESS_BEGIN();

    etimer_set(&et_adc, CLOCK_SECOND/10);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            SENSORS_ACTIVATE(*sensor);
            sensor->configure(ADC_SENSOR_SET_CHANNEL, POT);
            adcValue = (sensor -> value(ADC_SENSOR_VALUE)>>15);
            SENSORS_DEACTIVATE(*sensor);
            if(adcValue != oldAdcValue){
                process_post(&pwm_process, BRIGHTNESS_CHANGE_EVENT, (void*)adcValue);
                printf("Value Bri: %d\n", adcValue);
            }
            oldAdcValue = adcValue;
            etimer_reset(&et_adc);
        }
    }
    PROCESS_END();
}


PROCESS_THREAD(green_led_process, ev, data)
{
    static uint8_t count=0;
    static int period = 1;
    static int but = 1;
    PROCESS_BEGIN();


    etimer_set(&et_led, 1*CLOCK_SECOND);
    IOCPinTypeGpioOutput(GREEN_LED);

    while(1){
        PROCESS_WAIT_EVENT();

        if(ev == BUTTON_UP_EVENT){
            but = 1;
            printf("Liga led verde\n");
        }

        if(ev == BUTTON_DOWN_EVENT){
            but = 0;
            printf("Desliga led verde\n");
        }

        if(ev == PERIOD_CHANGE_EVENT){
            period = (int)data/10;
            if(period < 1)
                period = 1;
        }

        if(ev == PROCESS_EVENT_TIMER){
            GPIO_writeDio(GREEN_LED, ( (count++ & (1<<0) ) == 0x1) && but == 1);
            etimer_set(&et_led, 1*CLOCK_SECOND/period);
        }

    }
    PROCESS_END();
}

PROCESS_THREAD(readPeriod_process, ev, data)
{
    static int oldAdcValue=0, adcValue = 0;
    static struct sensors_sensor *sensor;
    sensor = sensors_find(ADC_SENSOR);

    PROCESS_BEGIN();

    etimer_set(&et_period, CLOCK_SECOND/10);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            SENSORS_ACTIVATE(*sensor);
            sensor->configure(ADC_SENSOR_SET_CHANNEL, PER);
            adcValue = (sensor -> value(ADC_SENSOR_VALUE)>>15);
            SENSORS_DEACTIVATE(*sensor);
            if(adcValue != oldAdcValue){
                process_post(&green_led_process, PERIOD_CHANGE_EVENT, (void*)adcValue);
                printf("Value Per: %d\n", adcValue);
            }
            oldAdcValue = adcValue;
            etimer_reset(&et_period);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(button_process, ev, data)
{
    PROCESS_BEGIN();

    while(1){
        PROCESS_YIELD();

        if(ev == sensors_event){
            if(data == &button_left_sensor)
                process_post(&green_led_process, BUTTON_UP_EVENT, (void*)(&button_process));
            if(data == &button_right_sensor)
                process_post(&green_led_process, BUTTON_DOWN_EVENT, (void*)(&button_process));
        }

    }
    PROCESS_END();
}




