#include <stdio.h>
#include <string.h>
#include <math.h>
#include "contiki.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "sys/log.h"


#define CLOCKING CLOCK_SECOND
static struct etimer timer;
int count=0;



/* Declare our "main" process, the basestation_process */
PROCESS(basestation_process, "Clicker basestation");
/* Process to turn off leds after 10 secs of inactivity */
PROCESS(led_process, "LED handling process");

/* The processes should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&basestation_process, &led_process);

/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
*/
static void recv(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest) {
    count++; 
    char pack[1];
    memcpy(&pack, data, sizeof(count));
    puts(pack);
    
    
    if ((pack[0] == '1')){     //turn on LED 1 if accelerometer is shaken 
        //printf("LED 1\n");
        leds_off(LEDS_ALL);
        leds_on(0b0001);
    }
    if ((pack[0] == '2')){  //turn on LED 2 if butto is pressed is shaken 
        //printf("LED 2\n");
        leds_off(LEDS_ALL);
        leds_on(0b0010);
    }
    if ((pack[0] == '3')){  //turn on LED 1 and 2 if accelerometer is shaken and button is pressed at the same time 
        //printf("LED 3\n");
        leds_off(LEDS_ALL);
        leds_on(0b0111);
    }
    
/* Polling the led process everytime a packet is received*/
    process_poll(&led_process);

}

/* Main process. */
PROCESS_THREAD(basestation_process, ev, data) {
    PROCESS_BEGIN();
    /* Initializing recv callback function */
    nullnet_set_input_callback(recv);
    PROCESS_END();
}


/* Led process. */
PROCESS_THREAD(led_process, ev, data) {    
    PROCESS_BEGIN();
    while(1){
        /* setting timer with a period of 10 secs */
        etimer_set(&timer, CLOCK_SECOND * 10);
        /* Wait for timer to expire or packet to arrive which polls this process*/
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer) || ev == PROCESS_EVENT_POLL);
        if (etimer_expired(&timer)){ //if timer expires, turn off the LEDs
            leds_off(LEDS_ALL);
        }      
        else                        //else just reset the timer
            etimer_reset(&timer);   
    }
    PROCESS_END();
}


