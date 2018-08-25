#include "contiki.h"
#include "driverlib/ioc.h"
#include <stdio.h>

#define BUTTON_EVENT  (45)
#define LED1 (IOID_14)
#define LED2 (IOID_15)

static struct etimer et_count;

PROCESS(count_process, "Count Process");
PROCESS(read_button_process, "Button Process");

AUTOSTART_PROCESSES(&count_process, &read_button_process);

PROCESS_THREAD(count_process, ev, data)
{
    static uint8_t counter = 0;

    PROCESS_BEGIN();
    IOCPinTypeGpioOutput(LED1);
    IOCPinTypeGpioOutput(LED2);


    etimer_set(&et_count, 1*CLOCK_SECOND);
    while(1){
        PROCESS_WAIT_EVENT();

        if(ev == PROCESS_EVENT_TIMER){
            if(counter == 1 || counter == 3)
                GPIO_setDio(LED1);
            else
                GPIO_clearDio(LED1);

            if(counter == 2 || counter == 3)
                GPIO_setDio(LED2);
            else
                GPIO_clearDio(LED2);

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

        //if(ev == sensors_event){
         //   if(data == &button_left_sensor){
         //       process_post(&count_process, BUTTON_EVENT, (void*)(&read_button_process));
         //   }

        //}

    }
    PROCESS_END();
}

