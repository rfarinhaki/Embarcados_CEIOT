#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
static struct etimer et_hello;
static struct etimer et_blink;
static struct etimer et_proc3;

PROCESS(hello_world_process, "Hello World Process");
PROCESS(blink_process, "Blink Process");
PROCESS(proc3_process, "Proc3 Process");

AUTOSTART_PROCESSES(&hello_world_process, &blink_process, &proc3_process);

PROCESS_THREAD(hello_world_process, ev, data)
{
    PROCESS_BEGIN();

    etimer_set(&et_hello, 4*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            printf("Hello World!\n");
            etimer_reset(&et_hello);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(blink_process, ev, data)
{
    PROCESS_BEGIN();

    etimer_set(&et_blink, 2*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            leds_toggle(LEDS_GREEN);
            etimer_reset(&et_blink);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(proc3_process, ev, data)
{
    PROCESS_BEGIN();

    etimer_set(&et_proc3, 10*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            printf("Proc3 Message!\n");
            etimer_reset(&et_proc3);
        }
    }
    PROCESS_END();
}
