#include "contiki.h"
#include "driverlib/ioc.h"
#include <stdio.h>

#define BUTTON_EVENT  (45)
#define LED1 (IOID_6)
#define LED2 (IOID_7)
#define BUTTON (IOID_13)
#define PRESSED 0
#define UP 1
#define DOWN 0

static struct etimer et_count;
static struct etimer et_button;

static uint8_t mode;

PROCESS(count_process, "Count Process");
PROCESS(read_button_process, "Button Process");

AUTOSTART_PROCESSES(&count_process, &read_button_process);

PROCESS_THREAD(count_process, ev, data)
{
    static int8_t counter = 0;

    PROCESS_BEGIN();
    IOCPinTypeGpioOutput(LED1);
    IOCPinTypeGpioOutput(LED2);

    etimer_set(&et_count, 1*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();

        if(ev == PROCESS_EVENT_TIMER){

            GPIO_writeDio(LED1, (counter & (1<<0)) == 0x1);
            GPIO_writeDio(LED2, (counter & (1<<1)) == 0x2);

            //if(counter == 1 || counter == 3)
            //    GPIO_setDio(LED1);
            //else
            //    GPIO_clearDio(LED1);

            //if( (counter &(1<<1)) == 0x2)
            //if(counter == 2 || counter == 3)
            //    GPIO_setDio(LED2);
            //else
            //    GPIO_clearDio(LED2);

            if(mode == UP)
                counter++;
            else
                counter--;

            if(counter > 3)
                counter = 0;

            if(counter < 0)
                counter = 3;

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
    IOCPinTypeGpioInput(BUTTON);
    etimer_set(&et_button, 1*CLOCK_SECOND/1000);

    while(1){
        PROCESS_YIELD();

        if(GPIO_readDio(BUTTON) == PRESSED){
            //process_post(&count_process, BUTTON_EVENT, (void*)(&read_button_process));
            mode = DOWN;
        }
        else
            mode = UP;

        etimer_reset(&et_button);

    }
    PROCESS_END();
}

