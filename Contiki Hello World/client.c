#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "dev/adxl345.h"

/* Declare our "main" process, the client process*/
PROCESS(client_process, "Clicker client");

/* The client process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&client_process);


static int payload_accel;
static char payload;
static struct etimer timer1;
int x = 0;

/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
 * As the client does not need to receive, the function does not do anything
 */
static void recv(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest) {
}

/* Our main process. */
PROCESS_THREAD(client_process, ev, data) {

	/* Activate the button sensor. */
	SENSORS_ACTIVATE(button_sensor);
	    

	PROCESS_BEGIN();

	/* Loop forever. */
	while (1) {
		/*set timer to 10Hz freq*/        
		etimer_set(&timer1, CLOCK_SECOND/10);

		/*wait for the timer to expire or button to be pressed*/
		PROCESS_WAIT_EVENT_UNTIL((etimer_expired(&timer1)) || (ev == sensors_event));
        
        x = 0;
		payload = '0';  
        

		/*read data from accelerometer and store in payload*/
		payload_accel = accm_read_axis(X_AXIS);

		if (abs(payload_accel) > 50){
			x = 1;
			payload = '1';
			leds_off(LEDS_ALL);
    		leds_on(LEDS_RED);	
		}
		
		if (ev == sensors_event && data == &button_sensor){
			payload = '2';
			leds_off(LEDS_ALL);
			leds_toggle(LEDS_GREEN);	
		}
		
		if (payload !='0'){				
		/* Initialize NullNet */
		nullnet_buf = (uint8_t *)&payload;
		nullnet_len = sizeof(payload);
		nullnet_set_input_callback(recv);

		/* Copy the payload into the packet buffer. */
		memcpy(nullnet_buf, &payload, sizeof(payload));
    	nullnet_len = sizeof(payload); 

		/* Send the content of the packet buffer using the
		 * broadcast handle. */
		NETSTACK_NETWORK.output(NULL);
	}
}

	
	
	PROCESS_END();
}

