/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include "dev/leds.h"
#include "ti-lib.h"
#include "dev/adc-sensor.h"
#include "dev/button-sensor.h"
#include "lib/sensors.h"
#include "lpm.h"

#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define SEND_INTERVAL        1 * CLOCK_SECOND//alterado para 5 segundos original 15
#define MAX_PAYLOAD_LEN        40
#define CONN_PORT     8802
#define MDNS 0 // 0 configura para zerar o MDNS


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


#define LED_TOGGLE_REQUEST_MSG  (0x79)
#define LED_SET_STATE_MSG  (0x7A)
#define LED_GET_STATE_MSG  (0x7B)
#define LED_STATE_MSG  (0x7C)
#define LED_PWM_VALUE_MSG (0x7D)
#define LED_PER_VALUE_MSG (0x7E)
#define ADC_PWM_VALUE_MSG (0x7F)
#define ADC_PER_VALUE_MSG (0x80)
#define BUTTON_LEFT_MSG (0x81)
#define BUTTON_RIGHT_MSG (0x82)
#define GREEN_LED_ON (0x83)
#define GREEN_LED_OFF (0x84)

#define OP_REQUEST (0X6E)
#define OP_RESULT (0X6F)

static char buf[MAX_PAYLOAD_LEN];

static struct uip_udp_conn *client_conn;

#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(pwm_process, "PWM Process");
PROCESS(button_process, "Button Process");
PROCESS(readAdc_process, "adc Process");
PROCESS(readPeriod_process, "read period process");
PROCESS(green_led_process, "Green Led process");

AUTOSTART_PROCESSES(&resolv_process,&udp_client_process, &pwm_process, &button_process, &readAdc_process, &readPeriod_process, &green_led_process);
/*---------------------------------------------------------------------------*/



static void
tcpip_handler(void)
{
    char i=0;
#define SEND_ECHO (0xBA)
    if(uip_newdata()) //verifica se novos dados foram recebidos
    {
        char* dados = ((char*)uip_appdata); //este buffer é padrão do contiki
        int pwmValue = 1, perValue = 0;
        PRINTF("Recebidos %d bytes\n", uip_datalen());
        switch (dados[0])
        {
        case LED_GET_STATE_MSG:

            buf[0] = LED_STATE_MSG;
            buf[1] = leds_get();

            uip_ipaddr_copy(&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            client_conn->rport = UIP_UDP_BUF->destport;
            uip_udp_packet_send(client_conn, buf, uip_datalen());
            PRINTF("Enviando para [");
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]:%u\n", UIP_HTONS(client_conn->rport));
            break;


        case LED_SET_STATE_MSG:


            leds_off(LEDS_ALL);
            leds_on(dados[1]);

            buf[0] = LED_STATE_MSG;
            buf[1] = leds_get();

            uip_ipaddr_copy(&client_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            client_conn->rport = UIP_UDP_BUF->destport;
            uip_udp_packet_send(client_conn, buf, uip_datalen());
            PRINTF("Enviando para [");
            PRINT6ADDR(&client_conn->ripaddr);
            PRINTF("]:%u\n", UIP_HTONS(client_conn->rport));
            break;

        case LED_PWM_VALUE_MSG:
            pwmValue = dados[1];
            PRINTF("Recebido PWM: %d", pwmValue);
            process_post(&pwm_process, BRIGHTNESS_CHANGE_EVENT, (void*)pwmValue);
            break;

        case LED_PER_VALUE_MSG:
            perValue = dados[1];
            PRINTF("Recebido periodo: %d", perValue);
            process_post(&green_led_process, PERIOD_CHANGE_EVENT, (void*)perValue);
            break;

        case GREEN_LED_OFF:
            process_post(&green_led_process, BUTTON_DOWN_EVENT, (void*)(&button_process));
            break;

        case GREEN_LED_ON:
            process_post(&green_led_process, BUTTON_UP_EVENT, (void*)(&button_process));
            break;

        default:

            PRINTF("Comando Invalido: ");
            for(i=0;i<uip_datalen();i++)
            {
                PRINTF("0x%02X ",dados[i]);
            }
            PRINTF("\n");
            break;

        }
    }
    return;
}

/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
    char payload = 0;

    buf[0] = LED_TOGGLE_REQUEST_MSG;//
    buf[1] = 0;
    if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        PRINTF("Aguardando auto-configuracao de IP\n");
        return;
    }

    PRINTF("Cliente para ["); //add ao codigo conforme exercicio sala
    PRINT6ADDR(&client_conn->ripaddr);//add ao codigo conforme exercicio sala
    PRINTF("]:%u\n", UIP_HTONS(client_conn->rport));//add ao codigo conforme exercicio sala

    // uip_udp_packet_send(client_conn, buf, strlen(buf));

    uip_udp_packet_send(client_conn, buf, 2);


}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
    int i;
    uint8_t state;

    PRINTF("Client IPv6 addresses: ");
    for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
        state = uip_ds6_if.addr_list[i].state;
        if(uip_ds6_if.addr_list[i].isused &&
                (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
            PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
            PRINTF("\n");
        }
    }
}
/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
static void
set_global_address(void)
{
    uip_ipaddr_t ipaddr;
    //uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

    uip_ip6addr(&ipaddr, 0Xfd00, 0, 0, 0, 0x0212, 0x4b00, 0x0aff, 0x6b01);
    //uip_ip6addr(&ipaddr, 0Xfe80, 0, 0, 0, 0x0212, 0x4b00, 0x07b9, 0x5e8d);

    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/


PROCESS_THREAD(udp_client_process, ev, data)
{
    static struct etimer et;
    uip_ipaddr_t ipaddr;

    PROCESS_BEGIN();
    PRINTF("UDP client process started\n");

#if UIP_CONF_ROUTER
    //set_global_address();
#endif

    etimer_set(&et, 2*CLOCK_SECOND);
    while(uip_ds6_get_global(ADDR_PREFERRED) == NULL)
    {
        PROCESS_WAIT_EVENT();
        if(etimer_expired(&et))
        {
            PRINTF("Aguardando auto-configuracao de IP\n");
            etimer_set(&et, 2*CLOCK_SECOND);
        }
    }

    print_local_addresses();


    //c_onfigures the destination IPv6 address
    // uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x215, 0x2000, 0x0002, 0x2145);


    //IPv6 do servidor Python
    uip_ip6addr(&ipaddr, 0X2804, 0x14c, 0x8786, 0x8166, 0x9ccd, 0xb3cd, 0xa46b, 0x9978);


    /* new connection with remote host */
    client_conn = udp_new(&ipaddr, UIP_HTONS(CONN_PORT), NULL);
    udp_bind(client_conn, UIP_HTONS(CONN_PORT));



    //client_conn = udp_new(&ipaddr, UIP_HTONS(8802), NULL);
    //udp_bind(client_conn, UIP_HTONS(8802));



    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF(" local/remote port %u/%u\n",
           UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

    etimer_set(&et, SEND_INTERVAL);
    while(1) {
        PROCESS_YIELD();
        if(etimer_expired(&et)) {
            timeout_handler();
            etimer_restart(&et);
        } else if(ev == tcpip_event) {
            tcpip_handler();
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/


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
                buf[0] = ADC_PWM_VALUE_MSG;
                buf[1] = (char)adcValue;
                uip_udp_packet_send(client_conn, buf, 2);

                //process_post(&pwm_process, BRIGHTNESS_CHANGE_EVENT, (void*)adcValue);
                //printf("Value Bri: %d\n", adcValue);
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
                buf[0] = ADC_PER_VALUE_MSG;
                buf[1] = (char)adcValue;
                uip_udp_packet_send(client_conn, buf, 2);
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
            if(data == &button_left_sensor){
                buf[0] = BUTTON_LEFT_MSG;
                buf[1] = 0;
                uip_udp_packet_send(client_conn, buf, 2);
                //process_post(&green_led_process, BUTTON_UP_EVENT, (void*)(&button_process));
            }
            if(data == &button_right_sensor){
                buf[0] = BUTTON_RIGHT_MSG;
                buf[1] = 0;
                uip_udp_packet_send(client_conn, buf, 2);
                //process_post(&green_led_process, BUTTON_DOWN_EVENT, (void*)(&button_process));
            }
        }

    }
    PROCESS_END();
}

