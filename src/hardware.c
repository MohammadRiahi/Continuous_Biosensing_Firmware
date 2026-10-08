/*
 * Hardware Abstraction Layer
 * Handles all peripheral initialization and control: LEDs, GPIO, ADC
 */

#include "hardware.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <errno.h>
#include "dac8831.h"

LOG_MODULE_REGISTER(hardware, LOG_LEVEL_INF);

// =============================================================================
// Device Tree Definitions
// =============================================================================

// LED GPIO specs from device tree aliases
#define LED1_NODE DT_ALIAS(led0)
#define LED2_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// Measure pin from zephyr,user node (for oscilloscope timing verification)
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), measure_timing_gpios)
    #define MEASURE_PIN_NODE DT_PATH(zephyr_user)
    static const struct gpio_dt_spec measure_pin = 
        GPIO_DT_SPEC_GET(MEASURE_PIN_NODE, measure_timing_gpios);
    #define HAS_MEASURE_PIN 1
#else
    #define HAS_MEASURE_PIN 0
#endif

// Power_on pin (P0.14) — driven high on startup
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), power_on_gpios)
    static const struct gpio_dt_spec power_on_pin =
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), power_on_gpios);
    #define HAS_POWER_ON_PIN 1
#else
    #define HAS_POWER_ON_PIN 0
#endif
// Neg_LDO pin (P1.0) — driven low on startup
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), neg_ldo_gpios)
    static const struct gpio_dt_spec neg_ldo_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), neg_ldo_gpios);
    #define HAS_NEG_LDO_PIN 1
#else
    #define HAS_NEG_LDO_PIN 0
#endif

// =============================================================================
// ADC Configuration
// =============================================================================

// ADC channel specs from device tree zephyr,user node io-channels
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// ADC device
static const struct device *adc_dev = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER_NODE));

// ADC channels from io-channels property (physical channels 2 and 7)
static const uint8_t adc_channel_ids[] = {
    DT_IO_CHANNELS_INPUT_BY_IDX(ZEPHYR_USER_NODE, 0),  // First entry = channel 2
    DT_IO_CHANNELS_INPUT_BY_IDX(ZEPHYR_USER_NODE, 1),  // Second entry = channel 7
};

// ADC configuration structs initialized from device tree
static struct adc_channel_cfg adc_channel_cfgs[2];

static int16_t adc_sample_buffer;  // Buffer for ADC readings
static struct adc_sequence adc_sequence;

#define NUM_ADC_CHANNELS ARRAY_SIZE(adc_channel_ids)

static uint8_t active_adc_channel = 0;  // Currently active logical channel

// =============================================================================
// LED Functions
// =============================================================================

int led_init(void) {
    int ret;
    
    if (!gpio_is_ready_dt(&led1)) {
        LOG_ERR("LED1 device not ready");
        return -ENODEV;
    }
    
    if (!gpio_is_ready_dt(&led2)) {
        LOG_ERR("LED2 device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED1: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED2: %d", ret);
        return ret;
    }
    
    LOG_INF("LEDs initialized successfully");
    return 0;
}

void led1_on(void) {
    gpio_pin_set_dt(&led1, 1);
}

void led1_off(void) {
    gpio_pin_set_dt(&led1, 0);
}

void led1_set(bool state) {
    gpio_pin_set_dt(&led1, state ? 1 : 0);
}

void led1_toggle(void) {
    gpio_pin_toggle_dt(&led1);
}

void led2_on(void) {
    gpio_pin_set_dt(&led2, 1);
}

void led2_off(void) {
    gpio_pin_set_dt(&led2, 0);
}

void led2_set(bool state) {
    gpio_pin_set_dt(&led2, state ? 1 : 0);
}

void led2_toggle(void) {
    gpio_pin_toggle_dt(&led2);
}

// =============================================================================
// Measure Pin Functions
// =============================================================================

int measure_pin_init(void) {
#if HAS_MEASURE_PIN
    if (!gpio_is_ready_dt(&measure_pin)) {
        LOG_WRN("Measure pin device not ready");
        return -ENODEV;
    }
    
    int ret = gpio_pin_configure_dt(&measure_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure measure pin: %d", ret);
        return ret;
    }
    
    LOG_INF("Measure pin initialized");
    return 0;
#else
    LOG_INF("Measure pin not configured in device tree, skipping");
    return 0;
#endif
}

void measure_pin_toggle(void) {
#if HAS_MEASURE_PIN
    gpio_pin_toggle_dt(&measure_pin);
#endif
}

void measure_pin_set(bool state) {
#if HAS_MEASURE_PIN
    gpio_pin_set_dt(&measure_pin, state ? 1 : 0);
#endif
}

// =============================================================================
// ADC Functions
// =============================================================================

int adc_init(void) {
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }
    
    // Initialize channel configs from device tree
    // Channel 0 (physical channel 2, AIN2 = P0.04)
    adc_channel_cfgs[0].gain = ADC_GAIN_1_3;
    adc_channel_cfgs[0].reference = ADC_REF_INTERNAL;
    adc_channel_cfgs[0].acquisition_time = ADC_ACQ_TIME_DEFAULT;
    adc_channel_cfgs[0].channel_id = adc_channel_ids[0];
    adc_channel_cfgs[0].input_positive = NRF_SAADC_AIN2;  /* P0.04 — must match DTS */
    
    // Channel 1 (physical channel 7, AIN7 = P0.31)
    adc_channel_cfgs[1].gain = ADC_GAIN_1_3;
    adc_channel_cfgs[1].reference = ADC_REF_INTERNAL;
    adc_channel_cfgs[1].acquisition_time = ADC_ACQ_TIME_DEFAULT;
    adc_channel_cfgs[1].channel_id = adc_channel_ids[1];
    adc_channel_cfgs[1].input_positive = NRF_SAADC_AIN7;  /* P0.31 — must match DTS */
    
    LOG_INF("ADC device initialized");
    return 0;
}

int adc_configure_channel(uint8_t channel) {
    // Validate logical channel number (0 or 1)
    if (channel >= NUM_ADC_CHANNELS) {
        LOG_ERR("Invalid logical channel %d (only 0-%d supported)", 
                channel, NUM_ADC_CHANNELS - 1);
        return -EINVAL;
    }
    
    // Check if ADC is ready
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }
    
    // Get the channel config
    struct adc_channel_cfg *cfg = &adc_channel_cfgs[channel];
    
    // Setup the channel
    int ret = adc_channel_setup(adc_dev, cfg);
    if (ret) {
        LOG_ERR("ADC channel %d setup failed with error %d", channel, ret);
        return ret;
    }
    
    // Initialize the ADC sequence for this channel
    adc_sequence.channels = BIT(cfg->channel_id);
    adc_sequence.buffer = &adc_sample_buffer;
    adc_sequence.buffer_size = sizeof(adc_sample_buffer);
    adc_sequence.resolution = 12;  // 12-bit resolution from DT
    
    active_adc_channel = channel;
    
    LOG_INF("ADC logical channel %d configured (physical channel %d)", 
            channel, cfg->channel_id);
    
    return 0;
}

uint16_t read_adc_single(void) {
    measure_pin_toggle();  // Toggle for oscilloscope timing
    
    int ret = adc_read(adc_dev, &adc_sequence);
    if (ret == 0) {
        return (uint16_t)adc_sample_buffer;  // Return raw value
    } else {
        LOG_ERR("ADC read failed with error %d", ret);
        return 0;
    }
}

uint16_t read_adc_averaged(uint8_t num_samples) {
    if (num_samples == 0) {
        LOG_WRN("Invalid num_samples (0), returning single read");
        return read_adc_single();
    }
    
    int32_t sum = 0;
    uint8_t successful_reads = 0;
    
    for (uint8_t i = 0; i < num_samples; i++) {
        int ret = adc_read(adc_dev, &adc_sequence);
        if (ret == 0) {
            sum += adc_sample_buffer;
            successful_reads++;
            LOG_DBG("Sample %d: %d, Running sum: %d", i, adc_sample_buffer, sum);
        } else {
            LOG_WRN("ADC read %d/%d failed", i + 1, num_samples);
        }
    }
    
    if (successful_reads == 0) {
        LOG_ERR("All ADC reads failed");
        return 0;
    }
    
    uint16_t average = (uint16_t)(sum / successful_reads);
    LOG_INF("ADC Sum: %d, Samples: %d, Average: %d", 
            sum, successful_reads, average);
    
    return average;
}

uint8_t adc_get_active_channel(void) {
    return active_adc_channel;
}

// =============================================================================
// Combined Initialization
// =============================================================================

int hw_init_all(void) {
    int ret;
    
    LOG_INF("=== Hardware Initialization Starting ===");
    
    // Initialize LEDs
    ret = led_init();
    if (ret < 0) {
        LOG_ERR("LED initialization failed: %d", ret);
        return ret;
    }
    
    // Initialize measure pin (optional)
    ret = measure_pin_init();
    if (ret < 0) {
        LOG_WRN("Measure pin init failed: %d (non-critical)", ret);
    }
    
    // Initialize ADC
    ret = adc_init();
    if (ret < 0) {
        LOG_ERR("ADC initialization failed: %d", ret);
        return ret;
    }
    
    // Configure default ADC channel (logical channel 0 = physical channel 2)
    ret = adc_configure_channel(0);
    if (ret < 0) {
        LOG_ERR("ADC channel 0 configuration failed: %d", ret);
        return ret;
    }

    // Drive P0.14 (Power_on) high on startup
#if HAS_POWER_ON_PIN
    if (!gpio_is_ready_dt(&power_on_pin)) {
        LOG_ERR("Power_on pin device not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&power_on_pin, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure Power_on pin: %d", ret);
        return ret;
    }
    LOG_INF("Power_on pin (P0.14) set high");
#endif
#if HAS_NEG_LDO_PIN
    if (!gpio_is_ready_dt(&neg_ldo_pin)) {
        LOG_ERR("Neg_LDO pin device not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&neg_ldo_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure Neg_LDO pin: %d", ret);
        return ret;
    }
    LOG_INF("Neg_LDO pin (1.0) set low");
#endif


    dac8831_init();                    // sets DAC output to 0V (0x8000) and initialized it. 

    LOG_INF("=== Hardware Initialization Complete ===");
    LOG_INF("Logical CH0 = Physical ADC channel %d", adc_channel_ids[0]);
    LOG_INF("Logical CH1 = Physical ADC channel %d", adc_channel_ids[1]);
    
    return 0;
}
