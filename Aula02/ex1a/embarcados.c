#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
//#include "ioc.h"
#include <stdio.h>

#define BUTTON_EVENT  (45)
#define LED1 (IOID_4)
#define LED2 (IOID_5)

static struct etimer et_count;

PROCESS(count_process, "Count Process");
PROCESS(read_button_process, "Button Process");

AUTOSTART_PROCESSES(&count_process, &read_button_process);

PROCESS_THREAD(count_process, ev, data)
{
    static uint8_t counter = 3;

    PROCESS_BEGIN();
    //IOCPortConfigureSet()


    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            if(counter == 1 || counter == 3)
                leds_on(LEDS_GREEN);
            else
                leds_off(LEDS_GREEN);

            if(counter == 2 || counter == 3)
                leds_on(LEDS_RED);
            else
                leds_off(LEDS_RED);

            counter++;
            if(counter == 4)
                counter = 0;

            etimer_reset(&et_count);

        }

        if(ev == BUTTON_EVENT){
            etimer_set(&et_count, 1*CLOCK_SECOND);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();

    while(1){
        PROCESS_YIELD();

        if(ev == sensors_event){
            if(data == &button_left_sensor){
                process_post(&count_process, BUTTON_EVENT, (void*)(&read_button_process));
            }

        }

    }
    PROCESS_END();
}

