#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <misc/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <misc/byteorder.h>

#include "relay_packet.h"
#include "relay_reporting.h"

#ifdef __V3_REPAY_WITH_AMP__
// repeater V3
#include <gpio.h>
#define PORT				"GPIO_0"
#define RF_MODE				17
#define RF_RXEN				18

struct device *gpio_dev;

void init_amp()
{
	gpio_dev = device_get_binding(PORT);
	if (gpio_dev == NULL) 
	{
		printk("GPIO system init failed\n");
		return;
	}
	
	gpio_pin_configure(gpio_dev, RF_MODE, GPIO_DIR_OUT);
	gpio_pin_configure(gpio_dev, RF_RXEN, GPIO_DIR_OUT);
	
	gpio_pin_write(gpio_dev, RF_MODE, 0);
	gpio_pin_write(gpio_dev, RF_RXEN, 1);
}
					    
#else
void init_amp()
{

}
#endif

void main(void)
{
	printk("Start Relay Packet Services ...\n");
	
	// update boot
	boot_count_update();
	
	// init amp
	init_amp();
	
	// relay
	start_relay_packet();	

		
	// init system reporting
	start_reporting();
}
