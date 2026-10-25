#ifndef MY_PBM_H
#define MY_PBM_H

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>


/** @brief Callback type for when an LED state change is received. */
typedef void (*led_cb_t)(const bool led_state);

/** @brief Callback type for when the button state is pulled. */
typedef bool (*button_cb_t)(void);

/** @brief Callback struct used by the PBM Service. */
struct my_pbm_cb {
	led_cb_t led_cb;
	button_cb_t button_cb;
};

typedef struct{
	int16_t e_ampl_pos;		    // config_buffer[2-3] // make it signed for debug 
	int16_t e_ampl_neg; 		// config_buffer[4-5]
	int16_t e_base;			    // config_buffer[6-7]
	int16_t e_end; 				// config_buffer[8-9]
	uint8_t pulse_freq; 		// config_buffer[10]
	uint8_t pw; 				// config_buffer[11]
	uint8_t delta_e; 			// config_buffer[12]
	uint8_t avg_num; 			// config_buffer[13]
	uint8_t sampling_rate; 		// config_buffer[14]
	uint16_t sampling_rate_hz;
	uint8_t EC_mode; 			// config_buffer[15]
} pbm_config_t;

extern volatile pbm_config_t g_cfg;


enum CommandType {
	CMD_STOP_ALL = 0x00,
    CMD_BATTERY_CHECK = 0x01,
    CMD_READ_CONFIG = 0x04,
    CMD_SET_CONFIG = 0x05,
	CMD_DEV_OFF = 0x07,
	CMD_START_SINGLE = 0x11,
    CMD_START_CONTINUOUS = 0x12,
	CMD_STOP_MEASUREMENT = 0x13,
};

extern uint8_t default_command[16];

int my_pbm_init(void);
int my_pbm_send_button_state_indicate(bool button_state);
int my_pbm_send_button_state_notify(bool button_state);
int my_pbm_send_sensor_notify(uint8_t *sensor_value);

void my_pbm_set_active_conn(struct bt_conn *conn);
void my_pbm_clear_active_conn(void);

int my_pbm_request_fast_conn_params(void);
int my_pbm_request_slow_conn_params(void);


#endif // MY_PBM_H
