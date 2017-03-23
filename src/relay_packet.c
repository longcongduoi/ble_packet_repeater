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
#define DEVICE_NAME "iot_repeater"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BT_LE_ADV_LOWPOWER BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, 0x640, 0x640)
#define BT_LE_ADV_CONN2 BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
				       BT_GAP_ADV_FAST_INT_MIN_2, \
				       BT_GAP_ADV_FAST_INT_MAX_2)
				       
#define htons(x) sys_cpu_to_be16(x)
#define htonl(x) sys_cpu_to_be32(x)

#define BLE_MAX_ADV_SIZE 31

static 
bool compare_address(const bt_addr_t* addr, uint8_t* le_addr)
{
	if (
		addr->val[5] == le_addr[0] && 
			addr->val[4] == le_addr[1] && 
				addr->val[3] == le_addr[2] && 
					addr->val[2] == le_addr[3] &&
						addr->val[1] == le_addr[4] &&
							addr->val[0] == le_addr[5] 
							) return true;
							
	return false;						
}

static   
bool get_data_from_advertise_bytype(
	uint8_t* adv_data,
	uint8_t adv_data_len,
	const uint8_t eir_type, 
	uint8_t** data,
	uint8_t* data_len)
{
    bool retval = false;
    
    if  (NULL != adv_data)
    {
        while (adv_data_len > 1)
        {
            uint8_t len = adv_data[0];
            uint8_t type = adv_data[1];

            /* Check for early termination */
            if ((len == 0) || ((len + 1) > adv_data_len)) {
                break;
            }

            if (type == eir_type)
            {
                if (len >= BLE_MAX_ADV_SIZE)
                {
					// fail
                    break;
                }
                
                *data = &adv_data[2];
                *data_len = len - 1;
                retval = true;
                
                break;
            }

            adv_data_len -= len + 1;
            adv_data += len + 1;
        }       
                
    }
    
    return retval;
}  

static uint16_t last_seq_id = 0;
#define ntohs(x) sys_be16_to_cpu(x)

struct msg_packet_header {
	uint16_t company_id;
	uint16_t seq_id;
	uint16_t command_id;
	uint8_t from_gateway;
	
} __packed;

struct msg_packet {
	struct msg_packet_header 	header;
	
	union {
		uint8_t detail[17];
		
		/* value from gateway, 0x22 */
		struct light_dimmer {
			uint8_t target_address[6];
			uint8_t dim_value; 
			} dimmmer;
		
		/* 0x21 */	
		struct light_onoff {
			uint8_t target_address[6];
			uint8_t dim_value; 
			} onoff;
		
		/* 0x11 */	
		struct door_open {
			uint8_t target_address[6];
			uint8_t open_value;
			} dooropen;	
		
		/* continues */		
	}
	
} __packed;


struct msg_packet_raw
{
	uint8_t data[BLE_MAX_ADV_SIZE];
	
} __packed;

K_MSGQ_DEFINE(raw_msgq, sizeof(struct msg_packet_raw), 5, 4);

// delay send
#define MAX_PKG_SIZE 25
static uint8_t raw_mfg_data[MAX_PKG_SIZE];
static uint8_t raw_packet_size;
struct k_delayed_work packet_send_delay_work;
struct k_delayed_work packet_stop_send_delay_work;
#define PACKET_DELAY_TIME 	50 // 50ms
#define PACKET_COUNT		4
#define PACKET_NUMBER 		5

static 
void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	int err = 0;
	
	if (ad->len < 20)
		return;
	
	static char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("*");
	
	struct msg_packet_raw packet;
	uint8_t* adv_data = ad->data;
	uint8_t adv_data_len = ad->len;
	
	/* pls check */
	memcpy(&packet.data[0], adv_data, adv_data_len);
	
	/* dont care response */
	err = k_msgq_put(&raw_msgq, &packet, K_NO_WAIT);
	
	if (err == -ENOMSG)
		printk("*ENOMSG");
	if (err == -EAGAIN)
		printk("*EAGAIN");			
		
}

#define DWORD uint32_t
#define WORD uint16_t
#define BYTE uint8_t

#define MAKEWORD(high, low) \
((WORD)((((WORD)(high)) << 8) | ((BYTE)(low))))

#define MAKEDWORD(a,b,c,d) ((((DWORD)(a)) << 24) | (((DWORD)(b)) << 16) | (((DWORD)(c)) << 8) | (((DWORD)(a))))

#define STACK_SIZE 1024
static char __stack __noinit stack[STACK_SIZE];
void packet_process_thread(void *id, void *unused1, void *unused2)
{	
	struct msg_packet_raw packet_raw;
	struct msg_packet packet;
	
	
	while (1) {
		/* get a data item */
        k_msgq_get(&raw_msgq, &packet_raw, K_FOREVER);
		printk(".");
		
		/* process here */		
		uint8_t mngr_len = 0;
		uint8_t* mngr_data = 0;
		
		if (get_data_from_advertise_bytype( 
				&packet_raw.data[0], BLE_MAX_ADV_SIZE, BT_DATA_MANUFACTURER_DATA, &mngr_data, &mngr_len) )
		{
			
			if (mngr_len < sizeof(struct msg_packet_header)) {
				// LOG error
				break;
			}
			
			
			packet.header.company_id 	= (uint16_t) MAKEWORD(mngr_data[0], mngr_data[1]);
			packet.header.seq_id  		= (uint16_t) MAKEWORD(mngr_data[2], mngr_data[3]);
			packet.header.command_id  	= (uint16_t) MAKEWORD(mngr_data[4], mngr_data[5]);
			packet.header.from_gateway 	= (int8_t) mngr_data[6];
			
#if __DEBUG_PACKET__						
			printk("Company ID: %04X\n", 	packet.header.company_id );
			printk("Seq ID: %d\n", 			packet.header.seq_id);
			printk("Command ID: 0x%02X\n", 	packet.header.command_id);
			printk("Gateway ID: %d\n", 		packet.header.from_gateway);
#endif			
			/* next */
			if (packet.header.company_id != 0xFF0F) {
				// LOG ERROR
				continue;
			}
			
			if (packet.header.seq_id == last_seq_id) {
				// LOG DOUBLE
				//printk("Double packages\n");
				continue;
			}
			
			// update sequence
			last_seq_id = packet.header.seq_id;
			
			// relay packet here
			memcpy((void*)raw_mfg_data, mngr_data, mngr_len);
			raw_packet_size = mngr_len;
			
			printk(">");
			k_delayed_work_submit(&packet_send_delay_work, PACKET_DELAY_TIME);
			
		}		
		k_yield();
	}
}

/* delayed send */
const char* relay_name = "relay";
#define BT_LE_ADV_RELAY_PACKET BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
				       BT_GAP_ADV_FAST_INT_MIN_1, \
				       BT_GAP_ADV_FAST_INT_MAX_1
				       
void packet_send_handler(struct k_work *item) 
{
	int err;
	
	// BT_LE_AD_NO_BREDR
	struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_MANUFACTURER_DATA, raw_mfg_data, raw_packet_size),
	};
	
	struct bt_data sd[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, relay_name, strlen(relay_name)),
	};
	
	printk("~");
	bt_le_adv_stop();
		
	// test adv
	err = bt_le_adv_start(BT_LE_ADV_CONN2, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	// send stop
	k_delayed_work_submit(&packet_stop_send_delay_work, PACKET_DELAY_TIME*4); // 120ms
}

/* delayed stop adv */
void packet_stop_adv_handler(struct k_work *item) 
{
	printk("#");
	bt_le_adv_stop();
}
	
#define BT_GAP_SCAN_FAST_INTERVAL1               0x050
#define BT_GAP_SCAN_FAST_WINDOW1                 0x025


#define BT_LE_MY_SCAN_PASSIVE BT_LE_SCAN_PARAM(BT_HCI_LE_SCAN_PASSIVE, \
					    BT_HCI_LE_SCAN_FILTER_DUP_ENABLE, \
					    BT_GAP_SCAN_FAST_INTERVAL1, \
					    BT_GAP_SCAN_FAST_WINDOW1)

/**
 * Main packet process 
 */
void init_packet_process()
{
	int err;
	
	// init bluetooth receiver
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized\n");

	
	// main work
	k_thread_spawn(&stack[0], STACK_SIZE,
			       packet_process_thread, (void *) 0, NULL, NULL, 0, 0, 0);	
			       
	// delay send 
	k_delayed_work_init(&packet_send_delay_work, packet_send_handler);
	
	// delay stop adv
	k_delayed_work_init(&packet_stop_send_delay_work, packet_stop_adv_handler);
	
			       
	err = bt_le_scan_start(BT_LE_MY_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}
	printk("Scanning successfully started\n");
}
	

void start_relay_packet()
{
	init_packet_process();	
}

void stop_relay()
{
	bt_le_adv_stop();
}

