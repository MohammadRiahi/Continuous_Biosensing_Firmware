/*
 * dac8831.c — Zephyr/NCS driver for the DAC8831 16-bit bipolar DAC
 *
 * SPI wiring:
 *   SCLK  → P0.07
 *   MOSI  → P0.05
 *   CS    → P0.08  (manually controlled via gpio0)
 *   MISO  → not connected (DAC is write-only)
 *
 * Board DTS requirement — add to your board DTS or overlay:
 *
 *   &spi0 {
 *       status = "okay";
 *       pinctrl-0 = <&spi0_default>;
 *       pinctrl-1 = <&spi0_sleep>;
 *       pinctrl-names = "default", "sleep";
 *   };
 *
 * And in pinctrl dtsi:
 *
 *   spi0_default: spi0_default {
 *       group1 {
 *           psels = <NRF_PSEL(SPIM_SCK,  0, 7)>,
 *                   <NRF_PSEL(SPIM_MOSI, 0, 5)>;
 *           nordic,drive-mode = <NRF_DRIVE_H0H1>;
 *       };
 *   };
 *   spi0_sleep: spi0_sleep {
 *       group1 {
 *           psels = <NRF_PSEL(SPIM_SCK,  0, 7)>,
 *                   <NRF_PSEL(SPIM_MOSI, 0, 5)>;
 *           low-power-enable;
 *       };
 *   };
 *
 * Usage example in my_pbm.c:
 *
 *   #include "dac8831.h"
 *
 *   // On startup:
 *   dac8831_init();                    // sets DAC output to 0V (0x8000)
 *
 *   // On CMD_START_CONTINUOUS event:
 *   dac8831_set_voltage(-400.0f);      // output -400 mV
 *
 *   // To output -500 mV:
 *   dac8831_set_voltage(-500.0f);
 */

#include "dac8831.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dac8831, LOG_LEVEL_INF);

/* ── Pin definitions ─────────────────────────────────────────────────────── */
#define DAC_CS_PIN    8U    /* P0.08 — chip select, active low */

/* ── DAC parameters ──────────────────────────────────────────────────────── */
#define DAC_VREF_V    1.8f          /* Reference voltage in volts            */
#define DAC_MIDCODE   0x8000U       /* Bipolar zero: (0 + 1.8) / 3.6 * 65536 */

/* ── Device handles ──────────────────────────────────────────────────────── */
static const struct device *spi_dev;   /* SPI0 controller                   */
static const struct device *cs_dev;    /* GPIO0 controller for manual CS    */

/*
 * SPI configuration:
 *   Mode 0: CPOL=0 (idle clock low), CPHA=0 (sample on rising edge)
 *   DAC8831 clocks SDI in on the rising edge of SCLK → Mode 0.
 *   MSB first, 8-bit words, 1 MHz, master mode, no hardware CS.
 */
static const struct spi_config dac_spi_cfg = {
    .frequency = MHZ(1),
    .operation = SPI_OP_MODE_MASTER |
                 SPI_WORD_SET(8)    |
                 SPI_TRANSFER_MSB,   /* CPOL=0, CPHA=0 = Mode 0 */
    .slave     = 0,
    .cs        = NULL,               /* CS managed manually below */
};

/* ── Internal: write a 16-bit code to the DAC ────────────────────────────── */
static void dac8831_write_code(uint16_t code)
{
    /* Split 16-bit code into two bytes, MSB first */
    uint8_t tx[2] = {
        (uint8_t)(code >> 8),    /* high byte */
        (uint8_t)(code & 0xFF),  /* low byte  */
    };

    struct spi_buf     tx_buf  = { .buf = tx, .len = sizeof(tx) };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    /* Assert CS low to begin transaction */
    gpio_pin_set(cs_dev, DAC_CS_PIN, 0);

    /* Send the two bytes over SPI */
    int ret = spi_write(spi_dev, &dac_spi_cfg, &tx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
    }

    /*
     * Deassert CS high.
     * LDAC is tied to GND, so the DAC register is transferred to the
     * output latch on this rising CS edge.
     */
    gpio_pin_set(cs_dev, DAC_CS_PIN, 1);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int dac8831_init(void)
{
    /* Obtain the SPI0 device handle from devicetree */
    spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI0 not ready — check board DTS pinctrl config");
        return -ENODEV;
    }

    /* Obtain the GPIO0 device handle for manual CS control */
    cs_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(cs_dev)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }

    /* Configure CS pin as push-pull output, start deasserted (high) */
    int ret = gpio_pin_configure(cs_dev, DAC_CS_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure CS pin P0.%02u: %d", DAC_CS_PIN, ret);
        return ret;
    }

    /* Write midcode 0x8000 → 0V output in bipolar mode */
    dac8831_write_code(DAC_MIDCODE);
    LOG_INF("DAC8831 ready — output set to 0V (code 0x%04X)", DAC_MIDCODE);

    return 0;
}

void dac8831_set_voltage(float voltage_mv)
{
    /* Clamp input to the valid bipolar output range */
    if (voltage_mv >  1800.0f) {
        voltage_mv =  1800.0f;
    }
    if (voltage_mv < -1800.0f) {
        voltage_mv = -1800.0f;
    }

    /* Convert millivolts to volts */
    float vout = voltage_mv / 1000.0f;

    /*
     * Bipolar code formula (VREF = 1.8 V):
     *   Code = ((Vout + Vref) / (2 * Vref)) * 65536
     *
     * Examples:
     *   0 mV    → (1.8 / 3.6) * 65536 = 32768 = 0x8000
     *  -400 mV  → (1.4 / 3.6) * 65536 ≈ 25445 = 0x6365
     *  -500 mV  → (1.3 / 3.6) * 65536 ≈ 23662 = 0x5C6E
     */
    float code_f = ((vout + DAC_VREF_V) / (2.0f * DAC_VREF_V)) * 65536.0f;

    /* Guard against floating-point edge cases */
    if (code_f < 0.0f)     { code_f = 0.0f;     }
    if (code_f > 65535.0f) { code_f = 65535.0f; }

    uint16_t code = (uint16_t)code_f;

    LOG_INF("DAC8831: %.1f mV → code 0x%04X", (double)voltage_mv, code);
    dac8831_write_code(code);
}
