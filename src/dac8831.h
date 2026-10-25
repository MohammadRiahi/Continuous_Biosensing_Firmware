#ifndef DAC8831_H
#define DAC8831_H

#include <zephyr/kernel.h>

/**
 * @brief Initialize the DAC8831 SPI interface and output 0V.
 *
 * Configures SPI0 (Zephyr spi_write API), CPOL=0, CPHA=1 (Mode 1),
 * MSB first, 1 MHz. CS (P0.08) is managed manually via gpio0.
 * After init, writes 0x8000 to the DAC → 0V bipolar output (VREF=1.8V).
 *
 * Requires spi0 to be enabled in the board DTS with pinctrl:
 *   SCLK → P0.07, MOSI → P0.05, MISO → not connected.
 *
 * @retval 0        Success.
 * @retval -ENODEV  SPI or GPIO device not ready.
 * @retval Other    Zephyr error code.
 */
int dac8831_init(void);

/**
 * @brief Set the DAC8831 output voltage.
 *
 * Converts voltage_mv to a 16-bit bipolar code and sends it over SPI.
 *   Code = ((Vout_V + VREF) / (2 * VREF)) * 65536,  VREF = 1.8 V
 *
 * Input is clamped to [-1800 mV, +1800 mV]. LDAC is tied to GND so the
 * DAC output updates on the rising edge of CS.
 *
 * Examples:
 *   dac8831_set_voltage(   0.0f) → 0x8000 →  0.0 V
 *   dac8831_set_voltage(-400.0f) → 0x6C72 → -0.4 V
 *   dac8831_set_voltage(-500.0f) → 0x6893 → -0.5 V
 *
 * @param voltage_mv  Desired output in millivolts (-1800.0f to +1800.0f).
 */
void dac8831_set_voltage(float voltage_mv);

#endif /* DAC8831_H */

