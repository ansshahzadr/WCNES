#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "dev/adxl345.h"

#define CLOCKING CLOCK_SECOND/100

/* Declare our "main" process, the client process*/
PROCESS(client_process, "Clicker client");

/* The client process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&client_process);

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
	static int payload;
	static struct etimer et;
	int temp = 0;
	int error = 100;

	PROCESS_BEGIN();

	/* Loop forever. */
	while (1) {
		/*read data from accelerometer and store in payload*/
		payload = accm_read_axis(X_AXIS);
		
		/* Initialize NullNet */
		nullnet_buf = (uint8_t *)&payload;
		nullnet_len = sizeof(payload);
		nullnet_set_input_callback(recv);

		/*set timer to 100Hz freq*/
        etimer_set(&et, CLOCKING);
        /*wait for the timer to expire*/
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et)); 
		if (payload - temp > error)
		{
			printf(" inside if %d\n", payload);
			if (payload > 0)
				leds_toggle(LEDS_RED);		
			else 
				leds_toggle(LEDS_GREEN);
		/* Copy the payload into the packet buffer. */
		memcpy(nullnet_buf, &payload, sizeof(payload));
    	nullnet_len = sizeof(payload); 

		/* Send the content of the packet buffer using the
		 * broadcast handle. */
		NETSTACK_NETWORK.output(NULL);
		temp = payload;
		}
	}
	
	PROCESS_END();
}
