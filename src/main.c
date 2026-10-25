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
	4800, /* Min Advertising Interval 500ms (800*0.625ms) */
	4808, /* Max Advertising Interval 500.625ms (801*0.625ms) */
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

#define RUN_LED_BLINK_INTERVAL 1000
//#define NOTIFY_INTERVAL 500


void toggle_led2(void);
void toggle_led1(void);
void set_led2(bool state);
void set_led1(bool state);

static struct k_work adv_work;
static bool is_ble_connected = false;


static struct k_timer adv_led_timer;

static void adv_led_timer_handler(struct k_timer *timer_id)
{
    static bool on;
    if (!is_ble_connected) {
        on = !on;
        set_led1(on);
    }
}


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

	k_timer_start(&adv_led_timer, K_NO_WAIT, K_MSEC(RUN_LED_BLINK_INTERVAL));

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}


static void on_connected(struct bt_conn *conn, uint8_t err)
{
	 if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");

	my_pbm_set_active_conn(conn);
	(void)my_pbm_request_slow_conn_params();
   

    is_ble_connected = true;
	k_timer_stop(&adv_led_timer);
    set_led1(false);
    set_led2(false);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	is_ble_connected = false;
	set_led1(false);
	k_timer_start(&adv_led_timer, K_NO_WAIT, K_MSEC(RUN_LED_BLINK_INTERVAL));
	set_led2(false);
	my_pbm_clear_active_conn();
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
	k_timer_init(&adv_led_timer, adv_led_timer_handler, NULL);


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
		k_sleep(K_FOREVER);
	}
}
/* STEP 18.2 - Define and initialize a thread to send data periodically */
//K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
