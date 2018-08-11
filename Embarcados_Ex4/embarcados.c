#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include <stdio.h>

#define LED_PING_EVENT  (44)
#define LED_PONG_EVENT  (45)

static struct etimer et_hello;
static struct etimer et_blink;
static struct etimer et_proc3;

PROCESS(hello_world_process, "Hello World Process");
PROCESS(blink_process, "Blink Process");
PROCESS(proc3_process, "Proc3 Process");
PROCESS(pong_process, "Pong Process");
PROCESS(read_button_process, "Button Process");

AUTOSTART_PROCESSES(&hello_world_process, &blink_process, &proc3_process, &pong_process, &read_button_process);

PROCESS_THREAD(hello_world_process, ev, data)
{
    PROCESS_BEGIN();
    static int id = 1;
    etimer_set(&et_hello, 4*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            printf("%d: Hello World!\n", id);
            process_post(&pong_process, LED_PING_EVENT, (void*)(&hello_world_process));
            etimer_reset(&et_hello);
        }
        if(ev == LED_PONG_EVENT){
            printf("%d: Recebido PONG por %s\n", id, hello_world_process.name);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(blink_process, ev, data)
{
    PROCESS_BEGIN();
    static int id = 2;
    etimer_set(&et_blink, 2*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            leds_toggle(LEDS_GREEN);
            process_post(&pong_process, LED_PING_EVENT, (void*)(&blink_process));
            etimer_reset(&et_blink);
        }

        if(ev == LED_PONG_EVENT){
            printf("%d: Recebido PONG por %s\n", id, blink_process.name);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(proc3_process, ev, data)
{
    PROCESS_BEGIN();
    static int id = 3;
    etimer_set(&et_proc3, 10*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            printf("%d: Proc3 Message!\n", id);
            process_post(&pong_process, LED_PING_EVENT, (void*)(&proc3_process));
            etimer_reset(&et_proc3);
        }
        if(ev == LED_PONG_EVENT){
            printf("%d: Recebido PONG por %s\n", id, proc3_process.name);
        }
    }
    PROCESS_END();
}

PROCESS_THREAD(pong_process, ev, data)
{
    PROCESS_BEGIN();
    static int id = 4;
    while(1){
        //PROCESS_WAIT_EVENT();
        PROCESS_YIELD();
        if(ev == LED_PING_EVENT){
            printf("%d: Recebido PING de %s\n", id, ((struct process*)data)->name);
            process_post((struct process*)data, LED_PONG_EVENT, NULL);
        }
    }

    PROCESS_END();

}

PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();
    static int id = 5;

    while(1){
        PROCESS_YIELD();

        if(ev == sensors_event){
            if(data == &button_left_sensor){
                printf("%d: Left Button!\n", id);
                leds_toggle(LEDS_RED);
                process_post(&pong_process, LED_PING_EVENT, (void*)(&read_button_process));
            }
            else if(data == &button_right_sensor){
                leds_toggle(LEDS_RED);
                printf("%d: Right Button!\n", id);
                process_post(&pong_process, LED_PING_EVENT, (void*)(&read_button_process));
            }
        }

        if(ev == LED_PONG_EVENT){
            printf("%d: Recebido PONG por %s\n", id, read_button_process.name);
        }
    }
    PROCESS_END();
}
