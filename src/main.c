/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/gpio.h>
#include "my_pbm.h"
#include "my_pbm_service_table.h"
#include "hardware.h"


static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	800, /* Min Advertising Interval 500ms (800*0.625ms) */
	801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
	NULL); /* Set to NULL for undirected advertising */

LOG_MODULE_REGISTER(Lesson4_Exercise2, LOG_LEVEL_DBG);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 100
//#define NOTIFY_INTERVAL 500


void toggle_led2(void);
void toggle_led1(void);
void set_led2(bool state);
void set_led1(bool state);

static bool app_button_state;
static struct k_work adv_work;
static bool is_ble_connected = false;

static uint32_t app_sensor_value = 100;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	//BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PBM_VAL),  // Main service only
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PBM_ADVERTISING_VAL),  // Advertising service in scan response
};

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/* STEP 16 - Define a function to simulate the data 
static void simulate_data(void)
{
	app_sensor_value++;
	if (app_sensor_value == 200) {
		app_sensor_value = 100;
	}
}
	*/


/* STEP 18.1 - Define the thread function  
void send_data_thread(void)
{
	while (1) {
		/* Simulate data 
		simulate_data();
		/* Send notification, the function sends notifications only if a client is subscribed 
		//my_pbm_send_sensor_notify(app_sensor_value);

		k_sleep(K_MSEC(NOTIFY_INTERVAL));
	}
}
*/

/*static struct my_pbm_cb app_callbacks = {
	.led_cb = app_led_cb,
	.button_cb = app_button_cb,
};
*/
/*
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;
		app_button_state = user_button_state ? true : false;
		// Button just updates the button state for the button characteristic
		// No streaming control here - that's done via BLE commands
	}
}
	*/
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
	
	// Log connection parameters
	struct bt_conn_info info;
	int ret = bt_conn_get_info(conn, &info);
	if (ret == 0) {
		double connection_interval = info.le.interval * 1.25; // Convert to ms
		uint16_t supervision_timeout = info.le.timeout * 10;   // Convert to ms
		printk("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms\n", 
			connection_interval, info.le.latency, supervision_timeout);
	}
	
	printk("Use your BLE app to:\n");
	printk("1. Enable DATA characteristic notifications\n");
	printk("2. Write [0x00, 0x11, 0x00...] to COMMAND characteristic to START streaming\n");
	printk("3. Write [0x00, 0x12, 0x00...] to COMMAND characteristic to STOP streaming\n");

	is_ble_connected = true;
	set_led1(false); // Turn off LED0 (P1.08) blinking
	set_led2(true); // Turn on LED1 (P0.24) when connected
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	dk_set_led_off(CON_STATUS_LED);
	is_ble_connected = false;
	set_led2(false); // Turn off LED1 (P0.24) when disconnected
	
	/* Restart advertising after disconnection */
	advertising_start();
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};



int main(void)
{
	//int blink_status = 0;
	int err;

	LOG_INF("Starting Lesson 4 - Exercise 2 \n");


	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	bt_conn_cb_register(&connection_callbacks);

	err = hw_init_all();
	if (err) {
		LOG_ERR("Hardware init failed (err %d)\n", err);
		return -1;
	}
	err = my_pbm_init();
	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return -1;
	}
	LOG_INF("Bluetooth initialized\n");
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();
	
	// Main loop - blink system status LED and handle LED0 for advertising
	for (;;) {
		
		// Handle LED0 blinking for advertising (when not connected)
		if (!is_ble_connected) {
			static int adv_blink_counter = 0;
			set_led1((++adv_blink_counter) % 2); // Blink LED1 for advertising
		}
		
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
/* STEP 18.2 - Define and initialize a thread to send data periodically */
//K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
