/* 
 * Le o o potenciometro em ADC0 e imprime o valor na serial
 */
#include "contiki.h"
#include "driverlib/ioc.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include <stdio.h>

#define POT (ADC_COMPB_IN_AUXIO0)

static struct etimer et_adc;

PROCESS(readAdc_process, "adc Process");

AUTOSTART_PROCESSES(&readAdc_process);

PROCESS_THREAD(readAdc_process, ev, data)
{
    static int adcValue = 0;
    static struct sensors_sensor *sensor;
    sensor = sensors_find(ADC_SENSOR);

    PROCESS_BEGIN();

    etimer_set(&et_adc, 2*CLOCK_SECOND);

    while(1){
        PROCESS_WAIT_EVENT();
        if(ev == PROCESS_EVENT_TIMER){
            SENSORS_ACTIVATE(*sensor);
            sensor->configure(ADC_SENSOR_SET_CHANNEL, POT);
            adcValue = sensor -> value(ADC_SENSOR_VALUE);
            SENSORS_DEACTIVATE(*sensor);

            printf("Value: %d\n", adcValue);
            etimer_reset(&et_adc);
        }
    }
    PROCESS_END();
}
