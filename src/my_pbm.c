/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief PBM Service sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "my_pbm.h"
#include "my_pbm_service_table.h"
#include "hardware.h"
#include "dac8831.h"
//-----------------------------Threads------------------------------------------------
#define ADC_THREAD_STACK_SIZE 			1024
#define BLE_THREAD_STACK_SIZE 			1024
#define DAC_UPDATE_THREAD_STACK_SIZE 	1024

#define ADC_THREAD_PRIORITY 5
#define BLE_THREAD_PRIORITY 5
#define DAC_UPDATE_THREAD_PRIORITY 4

K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ble_thread_stack, BLE_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(dac_update_thread_stack, DAC_UPDATE_THREAD_STACK_SIZE);

struct k_thread adc_thread_data;
struct k_thread ble_thread_data;
struct k_thread dac_update_thread_data;


struct k_mutex ring_buffer_mutex;
struct k_sem data_ready_sem;
struct k_timer adc_timer;
struct k_sem adc_sample_sem;
struct k_sem dac_update_sem;
struct k_sem dac_adc_sem;
//--------------------------Thread entry functions prototypes-------------------------
void adc_thread(void *p1, void *p2, void *p3);
void ble_thread(void *p1, void *p2, void *p3);
void dac_update_thread(void *p1, void *p2, void *p3);

//-----------------------------Configurations struct and helper functions for configuration update-----------------------------------------
volatile pbm_config_t g_cfg;
static struct k_mutex g_cfg_mutex;
static void cfg_from_buffer(const uint8_t buf[16], pbm_config_t *cfg) {
	cfg->e_ampl = buf[2];
	cfg->e_base_raw = sys_get_le16(&buf[3]);
	cfg->e_end_raw = sys_get_le16(&buf[5]);
	cfg->period_f = buf[7];
	cfg->delta_e = buf[8];
	cfg->avg_num = buf[9];
	cfg->sampling_rate = sys_get_le16(&buf[10]);
	cfg->sign_byte = buf[12];
}
typedef enum {
    PULSE_PHASE_IDLE = 0,
    PULSE_PHASE_START,
    PULSE_PHASE_MID
} pulse_phase_t;

static volatile pulse_phase_t pulse_phase = PULSE_PHASE_IDLE;
static volatile uint8_t timer_counter;
static volatile uint8_t pulse_cycle_count;

static bool e_base_is_signed(const pbm_config_t *cfg)
{
    return (cfg->sign_byte == 1U) || (cfg->sign_byte == 3U);
}

static bool e_end_is_signed(const pbm_config_t *cfg)
{
    return (cfg->sign_byte == 2U) || (cfg->sign_byte == 3U);
}

/* Return as int32_t so unsigned 0..65535 can still be represented safely */
static int32_t e_base_value(const pbm_config_t *cfg)
{
		//dac8831_set_voltage(-500.0); // debug; 
         int32_t mag = (int32_t)cfg->e_base_raw;   // two's complement decode
		 return e_base_is_signed(cfg) ? -mag : mag;
    
}

static int32_t e_end_value(const pbm_config_t *cfg)
{
		//dac8831_set_voltage(-500.0); // debug; 
         int32_t mag = (int32_t)cfg->e_end_raw;   // two's complement decode
		 return e_end_is_signed(cfg) ? -mag : mag;
}



//-----------------------------Constants----------------------------------------------
// Data packet configuration (ADC config now in hardware.c)

//Buffering 
#define DATAPACKET_SIZE 244
#define BUFFER_COUNT 2 // Double buffering 
#define RING_SIZE 512  // Must be power of 2 for efficiency
#define RING_MASK (RING_SIZE-1)  // For fast modulo operations
#define SAMPLES_PER_PACKET 118  // Based on (244-8)/2 = 118 samples per packet
static uint16_t adc_ring_buffer[RING_SIZE]; // Ring buffer for ADC samples
static volatile uint32_t ring_write_idx = 0;
static volatile uint32_t ring_read_idx = 0;

/* Forward declaration of the GATT service */
extern const struct bt_gatt_service_static my_pbm_svc;

// Sampling parameters 
volatile uint8_t averaging = 1; //Set by the user on the app
volatile uint32_t samplingRate = 1000; // Hz

/* Helper function to decode sample rate from code */
static uint32_t decodeSampleRate(uint8_t code) {
    switch (code) {
        case 0: return 100;
        case 1: return 250;
        case 2: return 500;
        case 3: return 1000;
        case 4: return 2000;
        case 5: return 4000;
        default: return 1000;  // default fallback
    }
}

static bool notify_DATA_enabled;
static bool notify_MESSAGE_enabled;
//static struct my_pbm_cb pbm_cb;
static uint8_t command_buffer[16]; // Buffer to store 16-yybyte commands
static uint8_t config_buffer[16]; // Buffer to store 16-byte configuration data
static uint8_t heartbeat_buffer[8]; // Buffer for heartbeat value
static char message_buffer[120]; // Static buffer for JSON messages
//static char data_buffer[244]; // Buffer for DATA characteristic
// Define the default command array
uint8_t default_command[16] = {0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define CMD_NVS_ID            1
#define CMD_NVS_SECTOR_COUNT  2

static struct nvs_fs cmd_nvs_fs;
static bool cmd_nvs_ready;
// Simple continuous measurement variables
static bool is_measuring = false;



static void   adc_timer_handler(struct k_timer *timer);
static void   packet_work_handler(uint8_t *packet);
static void   prepare_packet_header(uint8_t* packet);
static uint64_t get_timestamp(void);
//-----------------------------Functions----------------------------------------------
LOG_MODULE_DECLARE(Lesson4_Exercise2);

// -----------------Service functions and handlers ---------------------

/** @brief PBM Service callback structure forward declaration. */
 static ssize_t read_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset);
 static ssize_t write_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags);
 static void mypbmbc_ccc_DATA_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
 static void mypbmbc_ccc_message_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
 static ssize_t write_heartbeat(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags);
static void stop_continuous_measurement_timer(void);
static void start_continuous_measurement_timer(uint32_t hold_ms);
static void start_timer_sampling(uint32_t frequency_hz, uint32_t hold_ms);

// nvs functions 
static int command_nvs_init(void)
{
    const struct flash_area *fa;
    int err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
    if (err) {
        LOG_ERR("flash_area_open failed: %d", err);
        return err;
    }

    cmd_nvs_fs.flash_device = fa->fa_dev;
    cmd_nvs_fs.offset = fa->fa_off;
    cmd_nvs_fs.sector_size = fa->fa_size / CMD_NVS_SECTOR_COUNT;
    cmd_nvs_fs.sector_count = CMD_NVS_SECTOR_COUNT;

    err = nvs_mount(&cmd_nvs_fs);
    flash_area_close(fa);

    if (err) {
        LOG_ERR("nvs_mount failed: %d", err);
        return err;
    }

    cmd_nvs_ready = true;
    return 0;
}

static int command_nvs_load(uint8_t *buf, size_t len)
{
    ssize_t rd;

    if (!cmd_nvs_ready) {
        return -EACCES;
    }

    rd = nvs_read(&cmd_nvs_fs, CMD_NVS_ID, buf, len);
    if (rd == (ssize_t)len) {
        return 0;
    }
    if (rd < 0) {
        return (int)rd;
    }
    return -ENOENT;
}

static int command_nvs_save(const uint8_t *buf, size_t len)
{
    ssize_t wr;

    if (!cmd_nvs_ready) {
        return -EACCES;
    }

    wr = nvs_write(&cmd_nvs_fs, CMD_NVS_ID, buf, len);
    if (wr < 0) {
        return (int)wr;
    }
    if (wr != (ssize_t)len) {
        return -EIO;
    }
    return 0;
}


static ssize_t write_heartbeat(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Heartbeat write, handle: %u, conn: %p", attr->handle, (void *)conn);
	
	if (len != 8U) {
		LOG_ERR("Invalid heartbeat length: %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t heartbeat_value = sys_get_be32(buf);
	LOG_INF("Heartbeat value: %u", heartbeat_value);

	// Here you can handle the heartbeat value as needed
	// For now, we'll just log it
    
   //LOG_INF("Heartbeat ignored for testing");
	return len;
}

static void mypbmbc_ccc_message_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_MESSAGE_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("MESSAGE CCCD changed: %u, notifications %s", value, 
		notify_MESSAGE_enabled ? "enabled" : "disabled");
}

static void mypbmbc_ccc_DATA_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_DATA_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("DATA CCCD changed: %u, notifications %s", value, 
		notify_DATA_enabled ? "enabled" : "disabled");
}

static ssize_t write_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	// Make this VERY visible
	printk("\n\n*** WRITE_COMMANDS FUNCTION CALLED ***\n");
	printk("*** THIS SHOULD BE VERY VISIBLE ***\n\n");
	
	LOG_INF("=== WRITE_COMMANDS CALLED ===");
	LOG_INF("Command write, handle: %u, conn: %p", attr->handle, (void *)conn);

	LOG_INF("Data length: %u bytes", len);
	// -----------------------------------------------------------
	// Log the raw bytes received
	const uint8_t *data = (const uint8_t *)buf;
	LOG_INF("Raw data received:");
	for (int i = 0; i < len; i++) {
		printk("%02x", data[i]);
	}
	printk("\n");
	
	// Also log as hex string for easy comparison
	char hex_str[64];
	for (int i = 0; i < len && i < 16; i++) {
		sprintf(&hex_str[i*2], "%02x", data[i]);
	}
	hex_str[len*2] = '\0';
	LOG_INF("Hex string: %s", hex_str);
// -----------------------------------------------------------
	if (len != 16U) {
		LOG_INF("Write command: Incorrect data length, expected 16 bytes, got %u", len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write command: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	// Copy the 16-byte command into our buffer
	memcpy(command_buffer, buf, 16);
	// Switch on the second byte (command type)
	switch (command_buffer[1]) {
		case CMD_STOP_ALL:
			LOG_INF("Command: STOP_ALL");
			// Handle stop all command
			LOG_INF("Command: STOP_MEASUREMENT");
			timer_counter = 0;
			pulse_cycle_count = 0;
			stop_continuous_measurement_timer(); //stop_continuous_measurement();
			//dac8831_set_voltage(-100.0f); 
			break;
			
		case CMD_BATTERY_CHECK:
			LOG_INF("Command: BATTERY_CHECK");
			// debug - want to observe the contents of config buffer which comes from command_buffer in ligh blue
			uint8_t packet_debug[DATAPACKET_SIZE];
			memset(packet_debug, 0, DATAPACKET_SIZE);
			memcpy(packet_debug, config_buffer, 16);
			int err = my_pbm_send_sensor_notify(packet_debug);
			// end of debug 
			// Handle battery check command
			break;
			
		case CMD_READ_CONFIG:
			LOG_INF("Command: READ_CONFIG");
			// Update the second byte of config_buffer with the command type
			/*
			config_buffer[1] = CMD_READ_CONFIG;
			samplingRate = decodeSampleRate(config_buffer[7]);  // Decode sampling rate
			averaging = config_buffer[8];  // Set averaging parameter
			*/
			// Get current timestamp (using k_uptime_get_32() for milliseconds since boot)

			uint32_t current_timestamp = k_uptime_get_32();
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				g_cfg.e_ampl, g_cfg.e_base_raw, g_cfg.e_end_raw, g_cfg.period_f,
				g_cfg.delta_e, g_cfg.avg_num, g_cfg.sampling_rate, g_cfg.sign_byte);	
				LOG_INF("JSON message created: %s", message_buffer);

			/*
			 debug
			config_buffer[2] = 1; config_buffer[3] = 2; config_buffer[4] = 3; config_buffer[7] = 4; 
			config_buffer[8] = 5; config_buffer[9] = 6; config_buffer[10] = 7; config_buffer[11] = 8;
			config_buffer[12] = 9;
			// end debug */

			// Use the static message buffer instead of declaring on stack
			// Format JSON message similar to your C++ version
			/*
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				config_buffer[2], config_buffer[3], config_buffer[4], config_buffer[7],
				config_buffer[8], config_buffer[9], config_buffer[10], config_buffer[11],
				config_buffer[12]);
				
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				1, 2, 3, 4, 5, 6, 7, 8);
			*/
			LOG_INF("JSON message created: %s", message_buffer);
			
			// Check if MESSAGE notifications are enabled
			if (!notify_MESSAGE_enabled) {
				LOG_WRN("MESSAGE notifications not enabled by client");
				// Still return success as the command was processed
			} else {
				// Send notification through MESSAGE characteristic
				// The MESSAGE characteristic value attribute is at index 9 in the service
				LOG_INF("Attempting to send notification, message length: %d", strlen(message_buffer));
				int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[9], message_buffer, strlen(message_buffer));
				if (result == 0) {
					LOG_INF("Message notification sent successfully");
				} else {
					LOG_ERR("Failed to send message notification: %d", result);
					// Add specific error descriptions
					switch (result) {
						case -ENOMEM:
							LOG_ERR("No memory/buffer available for notification");
							break;
						case -ENOTCONN:
							LOG_ERR("Device not connected");
							break;
						case -EINVAL:
							LOG_ERR("Invalid parameters");
							break;
						default:
							LOG_ERR("Unknown error code: %d", result);
							break;
					}
				}
			}
			break;
			
		case CMD_SET_CONFIG:
		/* debug purpose 
			uint8_t packet_debug[DATAPACKET_SIZE];
			memset(packet_debug, 0, DATAPACKET_SIZE);
			packet_work_handler(packet_debug);
			memcpy(&packet_debug[8], command_buffer, 16);
			int err = my_pbm_send_sensor_notify(packet_debug);
		 end debug purpose*/
			LOG_INF("Command: SET_CONFIG");
			memcpy(config_buffer, command_buffer, 16); // overwrite config buffer with command buffer for now
			pbm_config_t tmp;
			cfg_from_buffer(config_buffer, &tmp);
			k_mutex_lock(&g_cfg_mutex, K_FOREVER);
			g_cfg = tmp;
			k_mutex_unlock(&g_cfg_mutex);
			command_nvs_save((const uint8_t *)&g_cfg, sizeof(g_cfg)); // save g_cfg to NVS (non-volatile storage)
			current_timestamp = k_uptime_get_32();
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				g_cfg.e_ampl, g_cfg.e_base_raw, g_cfg.e_end_raw, g_cfg.period_f,
				g_cfg.delta_e, g_cfg.avg_num, g_cfg.sampling_rate, g_cfg.sign_byte);	
				LOG_INF("JSON message created: %s", message_buffer);

			/*
			config_buffer[0] = 0;
			config_buffer[1] = CMD_SET_CONFIG;
			config_buffer[2] = command_buffer[2];
			// Clear bytes 3-6 (4 bytes) 
			memset(&config_buffer[3], 0, 4);
			config_buffer[7] = command_buffer[7];  // Sample Rate
			samplingRate = decodeSampleRate(config_buffer[7]);  // Decode sampling rate
			averaging = config_buffer[8];  // Set averaging parameter
			config_buffer[8] = averaging;
			config_buffer[9] = command_buffer[9];  // Set averaging parameter
			memset(&config_buffer[10], 0, 6); // Clear bytes 9-15
			
			memcpy(config_buffer, command_buffer, 16); // overwrite config buffer with command buffer for now
			current_timestamp = k_uptime_get_32();
			snprintf(message_buffer, sizeof(message_buffer),
				"{\"ts\":%u,\"Config read\":[%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
				current_timestamp,
				config_buffer[2], config_buffer[3], config_buffer[4], config_buffer[7],
				config_buffer[8], config_buffer[9], config_buffer[10], config_buffer[11],
				config_buffer[12]);		
				LOG_INF("JSON message created: %s", message_buffer);

				// save the configuration from the app into config_buffer and then into NVS
			int save_err = command_nvs_save(config_buffer, sizeof(config_buffer));
    		if (save_err) {
        		LOG_WRN("Failed to persist command buffer: %d", save_err);
   			 }
			 */

			// Check if MESSAGE notifications are enabled
			if (!notify_MESSAGE_enabled) {
				LOG_WRN("MESSAGE notifications not enabled by client");
				// Still return success as the command was processed
			} else {
				// Send notification through MESSAGE characteristic
				// The MESSAGE characteristic value attribute is at index 9 in the service
				LOG_INF("Attempting to send notification, message length: %d", strlen(message_buffer));
				int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[9], message_buffer, strlen(message_buffer));
				if (result == 0) {
					LOG_INF("Message notification sent successfully");
				} else {
					LOG_ERR("Failed to send message notification: %d", result);
					// Add specific error descriptions
					switch (result) {
						case -ENOMEM:
							LOG_ERR("No memory/buffer available for notification");
							break;
						case -ENOTCONN:
							LOG_ERR("Device not connected");
							break;
						case -EINVAL:
							LOG_ERR("Invalid parameters");
							break;
						default:
							LOG_ERR("Unknown error code: %d", result);
							break;
					}
				}
			}
			break;
			
		case CMD_STOP_MEASUREMENT:
			//LOG_INF("Command: STOP_MEASUREMENT");
			//stop_continuous_measurement();
			break;
		case CMD_START_SINGLE:
			LOG_INF("Command: START_SINGLE");

			/*pbm_config_t local_cfg; 
			k_mutex_lock(&g_cfg_mutex, K_FOREVER);
			local_cfg = g_cfg; // Copy the current configuration to a local variable
			k_mutex_unlock(&g_cfg_mutex);
			float v = (float)e_base_value(&local_cfg);
			dac8831_set_voltage(v); */
			/*adc_setup(SENSOR_PIN); // set up the ADC
			// Send a single data packet
			if (notify_DATA_enabled) {
				memset(data_buffer, 0, sizeof(data_buffer)); // filling the buffer with zeros
				// Prepare the data packet with the current timestamp and ADC values
				//prepare_data_packet((uint8_t*)data_buffer);
				int result = bt_gatt_notify(NULL, &my_pbm_svc.attrs[5], data_buffer, DATAPACKET_SIZE);
				if (result == 0) {
					LOG_INF("Single data packet sent (%d bytes)", DATAPACKET_SIZE);
				} else {
					LOG_ERR("Failed to send single data packet: %d", result);
				}
			} else {
				LOG_WRN("DATA notifications not enabled");
			}
				*/
			break;
		case CMD_START_CONTINUOUS:
			LOG_INF("Command: START_CONTINUOUS");

			if (is_measuring) {
				LOG_WRN("Already measuring");
				break;
			}

			const uint32_t hold_ms = 3000U;

			k_mutex_init(&ring_buffer_mutex);
			k_sem_init(&data_ready_sem, 0, 1);

			pbm_config_t local_cfg;
			k_mutex_lock(&g_cfg_mutex, K_FOREVER);
			local_cfg = g_cfg;
			k_mutex_unlock(&g_cfg_mutex);

			/* 1) Force DAC to e_base immediately */
			dac8831_set_voltage((float)e_base_value(&local_cfg));

			/* 2) Reset sweep state before anything starts */
			timer_counter = 0;
			pulse_cycle_count = 1;         /* first cycle uses base + 0*delta */
			pulse_phase = PULSE_PHASE_IDLE;

			/* 3) Mark running before thread create so loops do not exit early */
			//is_measuring = true;

			/* 4) Create worker threads */
			k_thread_create(&adc_thread_data, adc_thread_stack, ADC_THREAD_STACK_SIZE,
							adc_thread, NULL, NULL, NULL, ADC_THREAD_PRIORITY, 0, K_NO_WAIT);

			k_thread_create(&ble_thread_data, ble_thread_stack, BLE_THREAD_STACK_SIZE,
							ble_thread, NULL, NULL, NULL, BLE_THREAD_PRIORITY, 0, K_NO_WAIT);

			k_thread_create(&dac_update_thread_data, dac_update_thread_stack, DAC_UPDATE_THREAD_STACK_SIZE,
							dac_update_thread, NULL, NULL, NULL, DAC_UPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);

			/* 5) Start timer with initial hold delay, then periodic sampling */
			start_continuous_measurement_timer(hold_ms);
			break;			
			
		default:
			LOG_WRN("Unknown command type: 0x%02x", command_buffer[1]);
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}


static ssize_t read_commands(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset)
{
	// get a pointer to command_buffer (16-byte array) which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const uint8_t *command_data = (const uint8_t *)attr->user_data;

	LOG_INF("Command read, handle: %u, conn: %p", attr->handle, (void *)conn);
	
	// Log what we're about to send back to the client
	LOG_INF("Sending command buffer content:");
	for (int i = 0; i < 16; i++) {
		printk("%02x", command_data[i]);
	}
	printk("\n");

	// Return the content of command_buffer (16 bytes)
	return bt_gatt_attr_read(conn, attr, buf, len, offset, command_data, 16);
}

//----------------Ring Buffer Management Functions--------------------
static inline bool ring_buffer_put(uint16_t sample)
{
	uint32_t next_write  = (ring_write_idx + 1) & RING_MASK;	
	if (next_write == ring_read_idx) {
		// Buffer is full
		return false;
	}
	adc_ring_buffer[ring_write_idx] = sample;	
	ring_write_idx = next_write;

	return true;
}

static inline bool ring_buffer_get(uint16_t *sample){
	if (ring_read_idx == ring_write_idx) {
		// Buffer is empty
		return false;
	}
	*sample = adc_ring_buffer[ring_read_idx];
	ring_read_idx = (ring_read_idx + 1) & RING_MASK;
	return true;
}
static inline uint32_t ring_buffer_count(void){
	return (ring_write_idx - ring_read_idx) & RING_MASK;
}
static inline bool ring_buffer_is_full(void){
	return ((ring_write_idx + 1) & RING_MASK) == ring_read_idx;
}
static inline bool ring_buffer_is_empty(void){
	return ring_write_idx == ring_read_idx;
}
static void ring_buffer_reset(void){
	ring_write_idx = 0;
	ring_read_idx = 0;
}

// ADC functions moved to hardware.c
	
static void adc_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    uint8_t period_in_counts = g_cfg.sampling_rate / g_cfg.period_f;

    if (timer_counter == 0) {
        pulse_phase = PULSE_PHASE_START;
        k_sem_give(&dac_update_sem);
    } else if (timer_counter == period_in_counts / 2) {
        pulse_phase = PULSE_PHASE_MID;
        k_sem_give(&dac_update_sem);
    } else if (timer_counter >= period_in_counts) {
        timer_counter = 0;
        pulse_cycle_count++;
        pulse_phase = PULSE_PHASE_START;
        k_sem_give(&dac_update_sem);
    }

    k_sem_give(&adc_sample_sem);
    timer_counter++;

}
/*
static void adc_work_handler(struct k_work*work)
{
	if (!is_measuring) return;
	uint16_t sample;
	if(averaging > 1){
		sample = read_adc_averaged(averaging);
	}
	else {
		sample = read_adc_single();
		LOG_INF("Average of 1");
	}

	if (!ring_buffer_put(sample)){
		LOG_WRN("Ring buffer full, sample lost");
	}

	if (ring_buffer_count() >= SAMPLES_PER_PACKET){
		if (!k_work_is_pending(&packet_work)){
			k_work_submit(&packet_work);
		}
	}
}
	*/
static void packet_work_handler(uint8_t *packet)
{
	//if (!is_measuring) return;
	if(ring_buffer_count() < SAMPLES_PER_PACKET) return;
	memset(packet, 0, DATAPACKET_SIZE);

	prepare_packet_header(packet);

	for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
		uint16_t sample;
		if (ring_buffer_get(&sample)) {
			packet[8 + (i * 2)] = sample & 0xFF;
			packet[9 + (i * 2)] = (sample >> 8) & 0xFF;
		} else {	
			LOG_ERR("Unexpected ring buffer underflow");
			break;
		}
	}
}

static void prepare_packet_header(uint8_t* packet){
	uint64_t unix_time = get_timestamp();
	for (int i = 0; i < 6; i++) {
		packet[i] = (unix_time >> (i * 8)) & 0xFF;
	}
	const uint8_t num_channels = 1;
	const uint8_t precision = 1;
	const uint8_t channel_index = (1 << 0) << 4;
	uint8_t bytes_per_sample = precision + 1;
	uint8_t encoded_num_channels = num_channels - 1;
	uint8_t data_format = (encoded_num_channels & 0x03)
		| ((precision & 0x03) << 2)
		| (channel_index & 0xF0);
	packet[6] = data_format;
	packet[7] =  (DATAPACKET_SIZE - 8) / (num_channels * bytes_per_sample);
}

// adc_configure_channel() moved to hardware.c

void adc_thread(void *p1, void *p2, void *p3) {
    while (is_measuring) {
		//k_sem_take(&dac_adc_sem, K_FOREVER);
        k_sem_take(&adc_sample_sem, K_FOREVER);
        // Take ADC sample, lock mutex, put in buffer, unlock mutex, etc.
        // If enough samples, k_sem_give(&data_ready_sem);
        uint16_t sample;
        if (g_cfg.avg_num > 1) {
            sample = read_adc_averaged(g_cfg.avg_num);
        } else {
            sample = read_adc_single();
        }
        k_mutex_lock(&ring_buffer_mutex, K_FOREVER);
        ring_buffer_put(sample);
        k_mutex_unlock(&ring_buffer_mutex);

        if (ring_buffer_count() >= SAMPLES_PER_PACKET) {
            k_sem_give(&data_ready_sem);
        }
    }		
}

void ble_thread(void *p1, void *p2, void *p3) {
	 uint8_t packet[DATAPACKET_SIZE];
    while (is_measuring) {
        k_sem_take(&data_ready_sem, K_FOREVER);
        k_mutex_lock(&ring_buffer_mutex, K_FOREVER);// using mutex to protect ring buffer access
		packet_work_handler(packet);
        k_mutex_unlock(&ring_buffer_mutex);
		// send over BLE
			if (notify_DATA_enabled) {
		int err = my_pbm_send_sensor_notify(packet);
		if (err) {
			LOG_ERR("Notify DATA failed (err %d)", err);
		}  else if(err == -ENOMEM){
			LOG_WRN("BLE buffer full, packet dropped");
		}
		else if (err == 0) {
			LOG_DBG("Notify DATA sent");
		}
		else {
			LOG_WRN("DATA notifications not enabled");
			}
	  	}
	}
}
void dac_update_thread(void *p1, void *p2, void *p3) { //needs to be updated 
	    while (is_measuring) {
        k_sem_take(&dac_update_sem, K_FOREVER);
		//dac8831_set_voltage(-300.0f); // debug. 
        pbm_config_t local_cfg;
        k_mutex_lock(&g_cfg_mutex, K_FOREVER);
        local_cfg = g_cfg;
        k_mutex_unlock(&g_cfg_mutex);

        float current_base =
            (float)e_base_value(&local_cfg) +
            local_cfg.delta_e * (pulse_cycle_count - 1);

        if (current_base >= (float)e_end_value(&local_cfg)) {
            is_measuring = false;
            stop_continuous_measurement_timer();
            break;
        }

        if (pulse_phase == PULSE_PHASE_START) {
            dac8831_set_voltage(current_base + local_cfg.e_ampl);
        } else if (pulse_phase == PULSE_PHASE_MID) {
            dac8831_set_voltage(current_base - local_cfg.e_ampl);
        }
    }
}

static void start_timer_sampling(uint32_t frequency_hz, uint32_t hold_ms){
	ring_buffer_reset();
	uint32_t period_us  = 1000000 / frequency_hz;
	k_timer_start(&adc_timer,K_MSEC(hold_ms),K_USEC(period_us));
	is_measuring = true;
	LOG_INF("Started timer sampling at %d Hz (%d μs period)",frequency_hz, period_us);
}
static void stop_timer_sampling(void){
	/*
	k_work_cancel(&adc_work);
	k_work_cancel(&packet_work);
	*/
	is_measuring = false;
	k_timer_stop(&adc_timer);
	k_sem_give(&adc_sample_sem);
	k_sem_give(&data_ready_sem);
	k_sem_give(&dac_update_sem);
	
	ring_buffer_reset();
	LOG_INF("Stopped timer sampling");
}

static void start_continuous_measurement_timer(uint32_t hold_ms){
	
	if (is_measuring) {
		LOG_WRN("Already measuring");
		return;
	}
	LOG_INF("Starting timer-based measurement at %d Hz", g_cfg.sampling_rate);
	k_sem_init(&adc_sample_sem, 0, 1);
	k_sem_init(&dac_update_sem, 0, 1);
	k_sem_init(&dac_adc_sem,0,1);
	k_timer_init(&adc_timer, adc_timer_handler, NULL);
	//k_timer_start(&adc_timer, K_MSEC(1), K_MSEC(1));
	//start_timer_sampling(samplingRate);
	start_timer_sampling(g_cfg.sampling_rate, hold_ms);
}

static void stop_continuous_measurement_timer(void){
	LOG_INF("Stopping timer-based measurement");
	stop_timer_sampling();
}

// read_adc_averaged() moved to hardware.c

static uint64_t get_timestamp(void)
{
	return k_uptime_get();
}

// ...existing code for message, CCCD, and BLE handlers...
#include "my_pbm_service_table.h"

// -------------------------------BLE related code---------------------------------
// PIBiomed (PBM) Service Declaration
BT_GATT_SERVICE_DEFINE(
    my_pbm_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_PBM),
    // Command Characteristic with descriptive name
    BT_GATT_CHARACTERISTIC(BT_UUID_PBM_COMMAND, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ, read_commands, write_commands, &command_buffer),
    BT_GATT_CUD("COMMAND", BT_GATT_PERM_READ),
    // Data Characteristic with descriptive name
    BT_GATT_CHARACTERISTIC(BT_UUID_PBM_DATA, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CUD("DATA", BT_GATT_PERM_READ),
    BT_GATT_CCC(mypbmbc_ccc_DATA_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    // Message Characteristic with descriptive name
    BT_GATT_CHARACTERISTIC(BT_UUID_PBM_MESSAGE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CUD("MESSAGE", BT_GATT_PERM_READ),
    BT_GATT_CCC(mypbmbc_ccc_message_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    // HeartBeat Characteristic with descriptive name
    BT_GATT_CHARACTERISTIC(BT_UUID_PBM_HEARTBEAT, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE, NULL, write_heartbeat, &heartbeat_buffer),
    BT_GATT_CUD("HEARTBEAT", BT_GATT_PERM_READ)
);

int my_pbm_init(void)
{
    int err;

    LOG_INF("PBM service initialization started");
 	k_mutex_init(&g_cfg_mutex);
	pbm_config_t tmp;

	err = command_nvs_init();
    if (!err && !command_nvs_load((uint8_t *)&tmp, 14)) { // instead of 14 i used to have sizeof(tmp)
        LOG_INF("Command buffer restored from NVS");
		k_mutex_lock(&g_cfg_mutex, K_FOREVER);
		g_cfg = tmp;
		k_mutex_unlock(&g_cfg_mutex);
    } else {
        memcpy(config_buffer, default_command, sizeof(config_buffer));
        LOG_INF("Command buffer initialized with default command");
    }

    // Initialize ring buffer
    ring_buffer_reset();

    LOG_INF("Service has %d attributes", my_pbm_svc.attr_count);
    for (int i = 0; i < my_pbm_svc.attr_count; i++) {
        LOG_INF("Attr[%d]: UUID type %d, read=%p, write=%p",
            i, my_pbm_svc.attrs[i].uuid->type,
            (void*)my_pbm_svc.attrs[i].read,
            (void*)my_pbm_svc.attrs[i].write);
    }
    LOG_INF("PBM service initialization completed");
    return 0;
}

int my_pbm_send_sensor_notify(uint8_t *sensor_value)
{
	if (!notify_DATA_enabled) {
		return -EACCES;
	}
	return bt_gatt_notify(NULL, &my_pbm_svc.attrs[5], sensor_value, DATAPACKET_SIZE);
}
// -----------LED Control Functions--------------------
// Wrapper functions that call hardware layer
void set_led1(bool state) {
	led1_set(state);
}

void set_led2(bool state) {
	led2_set(state);
}

void toggle_led1(void) {
	led1_toggle();
}

void toggle_led2(void) {
	led2_toggle();
}
