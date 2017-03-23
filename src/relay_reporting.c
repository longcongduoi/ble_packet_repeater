#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <flash.h>
#include <device.h>
#include <misc/printk.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <misc/byteorder.h>

/* reboot status */
struct device *flash_dev;
static uint32_t boot_count;

#define FLASH_BOOT_OFFSET 0x3fc00 //0x3f800
#define FLASH_PAGE_SIZE   1024
#define FLASH_COMPANY_ID  0xFF0F


/* BLE reporting */
#define DEVICE_NAME "iot_relay"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static 
uint8_t rp_data[] = {0xff, 0x0f, /* VNG */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define BT_LE_ADV_LOWPOWER BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, 0x640, 0x640)
#define BT_LE_ADV_CONN2 BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
				       BT_GAP_ADV_FAST_INT_MIN_2, \
				       BT_GAP_ADV_FAST_INT_MAX_2)
				       
#define htons(x) sys_cpu_to_be16(x)
#define htonl(x) sys_cpu_to_be32(x)

/* delayed stop adv */
void stop_send_report_packet_handler(struct k_work *item) 
{
	bt_le_adv_stop();
}
struct k_delayed_work stop_send_delay_work;

void send_report_packet_handler(struct k_work *work)
{
	int err;
	
	// rp values
	uint16_t* deviceType 	= (uint16_t*) &rp_data[2];
	uint32_t* token			= (uint32_t*) &rp_data[2+2];
	uint8_t* energy			= (uint8_t*) &rp_data[2+2+4];
	uint32_t* resets		= (uint32_t*) &rp_data[2+2+4+1];
   
	// reboot count + name
	char buffer[20];
	sprintf(buffer, "%s_%d", DEVICE_NAME, boot_count);
	
	struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, buffer, strlen(buffer)),
	};
	
	*deviceType	= htons(0x11);
	*token = htonl(0xffffffff);
	*energy = 33;
	*resets = htonl(boot_count);
	
	// BT_LE_AD_NO_BREDR
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, rp_data, sizeof(rp_data)),
	};
		
	// test adv
	err = bt_le_adv_start(BT_LE_ADV_CONN2, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	
	k_delayed_work_submit(&stop_send_delay_work, 200); // 200 ms
    
}

struct k_work send_report_work = K_WORK_INITIALIZER(send_report_packet_handler);

void report_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&send_report_work);
}

K_TIMER_DEFINE(report_timer, report_timer_handler, NULL);

void start_reporting()
{
	k_delayed_work_init(&stop_send_delay_work, stop_send_report_packet_handler);
	
	/* start periodic timer that expires once every second */
	k_timer_start(&report_timer, K_SECONDS(10), K_SECONDS(10));
}

void stop_reporting()
{
	bt_le_adv_stop();
}

void boot_count_update()
{
	uint32_t buf_word[2];
	uint32_t offset;
	int16_t pagesize = NRF_FICR->CODEPAGESIZE;
	
	flash_dev = device_get_binding("NRF5_FLASH");

	printk("Flash Page Size: %d\n", pagesize);
	
	if (!flash_dev) {
		printk("Nordic nRF5 flash driver was not found!\n");
		return;
	}
	
	offset = FLASH_BOOT_OFFSET;
	if (flash_read(flash_dev, offset, &buf_word[0],
					2*sizeof(uint32_t)) != 0) {
		printk("Flash read failed!\n");
		return;
	}
	
	/* Valid flash data */
	if (buf_word[0] == FLASH_COMPANY_ID)
	{	
		flash_write_protection_set(flash_dev, false);
			
		boot_count = buf_word[1];
		printk("Sig: %04X, Bootcount = %d, buff = %d\n",buf_word[0], boot_count, buf_word[1]);
		
		
		if (flash_erase(flash_dev, offset , pagesize) != 0) {
			printk("   Flash erase failed!\n");
		} else {
			printk("   Flash erase succeeded!\n");
		}		
		
		flash_write_protection_set(flash_dev, false);
		buf_word[1] = boot_count+1;
		if (flash_write(flash_dev, offset, &buf_word[0],
					2*sizeof(uint32_t)) != 0) {
			printk("Flash write boot count failed!\n");
			return;
		}
		
		if (flash_write(flash_dev, offset, &buf_word[0],
					2*sizeof(uint32_t)) != 0) {
			printk("Flash write boot count failed!\n");
			return;
		}
		flash_write_protection_set(flash_dev, true);
		printk("Update bootcount, last value = %d\n", boot_count);
	}
	else
	{
		flash_write_protection_set(flash_dev, false);
		if (flash_erase(flash_dev, offset, pagesize) != 0) {
			printk("   Flash erase failed!\n");
		} else {
			printk("   Flash erase succeeded!\n");
		}
		
		// init boot data
		printk("Init boot count data...\n");
		flash_write_protection_set(flash_dev, false);
		
		buf_word[0] = FLASH_COMPANY_ID;
		buf_word[1] = boot_count = 1;
		if (flash_write(flash_dev, offset, &buf_word[0],
					2*sizeof(uint32_t)) != 0) {
			printk("Flash write boot flag failed!\n");
			return;
		}		
		
		flash_write_protection_set(flash_dev, true);		
	}
}
