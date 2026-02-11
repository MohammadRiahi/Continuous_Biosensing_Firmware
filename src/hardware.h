#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>

// ============= Initialization =============
int hw_init_all(void);
int led_init(void);
int measure_pin_init(void);
int adc_init(void);

// ============= LED Control =============
void led1_on(void);
void led1_off(void);
void led1_set(bool state);
void led1_toggle(void);

void led2_on(void);
void led2_off(void);
void led2_set(bool state);
void led2_toggle(void);

// ============= Measure Pin (oscilloscope timing) =============
void measure_pin_toggle(void);
void measure_pin_set(bool state);

// ============= ADC Control =============
int adc_configure_channel(uint8_t channel);
uint16_t read_adc_single(void);
uint16_t read_adc_averaged(uint8_t num_samples);
uint8_t adc_get_active_channel(void);

#endif // HARDWARE_H
