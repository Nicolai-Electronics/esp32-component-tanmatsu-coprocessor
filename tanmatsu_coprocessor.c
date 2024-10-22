// Tanmatsu coprocessor interface component
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "tanmatsu_coprocessor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"

// Registers
#define TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_0          0  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_1          1  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_0            2  // 9 bytes
#define TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT     11
#define TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT    12
#define TANMATSU_COPROCESSOR_I2C_REG_INTERRUPT             13
#define TANMATSU_COPROCESSOR_I2C_REG_RESERVED_0            14
#define TANMATSU_COPROCESSOR_I2C_REG_INPUT                 15
#define TANMATSU_COPROCESSOR_I2C_REG_OUTPUT                16
#define TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL         17
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_0           18  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_1           19
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_2           20
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_3           21  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_BACKUP_0              22  // 84 bytes
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_COMM_FAULT       106
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_FAULT            107
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_CONTROL      108
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBAT_0       109  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBAT_1       110  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VSYS_0       111  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VSYS_1       112  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_TS_0         113  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_TS_1         114  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBUS_0       115  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBUS_1       116  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_ICHGR_0      117  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_ICHGR_1      118  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_CHARGING_CONTROL 119
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_CHARGING_STATUS  120
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_OTG_CONTROL      121

typedef struct tanmatsu_coprocessor {
    i2c_master_dev_handle_t dev_handle;           /// I2C device handle
    tanmatsu_coprocessor_config_t configuration;  /// Copy of the configuration struct provided during initialization
    TaskHandle_t interrupt_handler_thread;        /// Task handle for the interrupt handling thread
    SemaphoreHandle_t interrupt_semaphore;        /// Semaphore for triggering the interrupt thread
} tanmatsu_coprocessor_t;

static char const TAG[] = "Tanmatsu coprocessor";

static void tanmatsu_coprocessor_interrupt_thread_entry(void* pvParameters) {
    tanmatsu_coprocessor_handle_t handle = (tanmatsu_coprocessor_handle_t)pvParameters;

    tanmatsu_coprocessor_keys_t prev_keys = {0};
    tanmatsu_coprocessor_keys_t keys = {0};
    tanmatsu_coprocessor_inputs_t prev_inputs = {0};
    tanmatsu_coprocessor_inputs_t inputs = {0};
    tanmatsu_coprocessor_pmic_faults_t prev_faults = {0};
    tanmatsu_coprocessor_pmic_faults_t faults = {0};

    bool failed = false;

    while (true) {
        // Wait for interrupt
        if (failed) {
            printf("Previous interrupt coprocessor read failed!\r\n");
        }
        xSemaphoreTake(handle->interrupt_semaphore, pdMS_TO_TICKS(1000));
        failed = false;

        // Read interrupt reason
        bool keyboard, input, pmic;
        if (tanmatsu_coprocessor_get_interrupt(handle, &keyboard, &input, &pmic) != ESP_OK) {
            failed = true;
            continue;
        }

        if (keyboard) {
            // Read keyboard state
            if (tanmatsu_coprocessor_get_keyboard_keys(handle, &keys) != ESP_OK) {
                failed = true;
                continue;
            }
        }

        if (input) {
            // Read input state
            if (tanmatsu_coprocessor_get_inputs(handle, &inputs) != ESP_OK) {
                failed = true;
                continue;
            }
        }

        if (pmic) {
            // Read PMIC state
            if (tanmatsu_coprocessor_get_pmic_faults(handle, &faults) != ESP_OK) {
                failed = true;
                continue;
            }
        }

        if (memcmp(&prev_keys, &keys, sizeof(tanmatsu_coprocessor_keys_t))) {
            if (handle->configuration.on_keyboard_change) {
                handle->configuration.on_keyboard_change(handle, &prev_keys, &keys);
            }
        }

        if (memcmp(&prev_inputs, &inputs, sizeof(tanmatsu_coprocessor_inputs_t))) {
            if (handle->configuration.on_input_change) {
                handle->configuration.on_input_change(handle, &prev_inputs, &inputs);
            }
        }

        if (memcmp(&prev_faults, &faults, sizeof(tanmatsu_coprocessor_pmic_faults_t))) {
            // tbd
            printf("PMIC faults changed\r\n");
        }

        memcpy(&prev_keys, &keys, sizeof(tanmatsu_coprocessor_keys_t));
        memcpy(&prev_inputs, &inputs, sizeof(tanmatsu_coprocessor_inputs_t));
        memcpy(&prev_faults, &faults, sizeof(tanmatsu_coprocessor_pmic_faults_t));
    }
}

IRAM_ATTR static void tanmatsu_coprocessor_interrupt_handler(void* pvParameters) {
    tanmatsu_coprocessor_handle_t handle = (tanmatsu_coprocessor_handle_t)pvParameters;
    xSemaphoreGiveFromISR(handle->interrupt_semaphore, NULL);
    portYIELD_FROM_ISR();
}

// Wrapping functions for making ESP-IDF I2C driver thread-safe

static void claim_i2c_bus(tanmatsu_coprocessor_handle_t handle) {
    // Claim I2C bus
    if (handle->configuration.concurrency_semaphore != NULL) {
        xSemaphoreTake(handle->configuration.concurrency_semaphore, portMAX_DELAY);
    } else {
        ESP_LOGW(TAG, "No concurrency semaphore");
    }
}

static void release_i2c_bus(tanmatsu_coprocessor_handle_t handle) {
    // Release I2C bus
    if (handle->configuration.concurrency_semaphore != NULL) {
        xSemaphoreGive(handle->configuration.concurrency_semaphore);
    }
}

static esp_err_t ts_i2c_master_transmit_receive(tanmatsu_coprocessor_handle_t handle, i2c_master_dev_handle_t i2c_dev,
                                                const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer,
                                                size_t read_size, int xfer_timeout_ms) {
    claim_i2c_bus(handle);
    esp_err_t res =
        i2c_master_transmit_receive(i2c_dev, write_buffer, write_size, read_buffer, read_size, xfer_timeout_ms);
    release_i2c_bus(handle);
    return res;
}

static esp_err_t ts_i2c_master_transmit(tanmatsu_coprocessor_handle_t handle, i2c_master_dev_handle_t i2c_dev,
                                        const uint8_t* write_buffer, size_t write_size, int xfer_timeout_ms) {
    claim_i2c_bus(handle);
    esp_err_t res = i2c_master_transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
    release_i2c_bus(handle);
    return res;
}

esp_err_t tanmatsu_coprocessor_initialize(const tanmatsu_coprocessor_config_t* configuration,
                                          tanmatsu_coprocessor_handle_t* out_handle) {
    ESP_RETURN_ON_FALSE(configuration, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    ESP_RETURN_ON_FALSE(configuration->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(configuration->i2c_address, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    tanmatsu_coprocessor_t* handle = heap_caps_calloc(1, sizeof(tanmatsu_coprocessor_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, TAG, "no memory for coprocessor struct");

    memcpy(&handle->configuration, configuration, sizeof(tanmatsu_coprocessor_config_t));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = configuration->i2c_address,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(configuration->i2c_bus, &dev_cfg, &handle->dev_handle));

    handle->interrupt_semaphore = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(handle->interrupt_semaphore, ESP_ERR_NO_MEM, TAG, "no memory for interrupt semaphore");
    xSemaphoreGive(handle->interrupt_semaphore);

    if (configuration->int_io_num >= 0) {
        assert(xTaskCreate(tanmatsu_coprocessor_interrupt_thread_entry, "Tanmatsu coprocessor interrupt task", 4096,
                           (void*)handle, 0, &handle->interrupt_handler_thread) == pdTRUE);

        gpio_config_t int_pin_cfg = {
            .pin_bit_mask = BIT64(configuration->int_io_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = false,
            .pull_down_en = false,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&int_pin_cfg));
        ESP_ERROR_CHECK(
            gpio_isr_handler_add(configuration->int_io_num, tanmatsu_coprocessor_interrupt_handler, (void*)handle));
    }

    *out_handle = handle;
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_firmware_version(tanmatsu_coprocessor_handle_t handle,
                                                    uint16_t* out_firmware_version) {
    uint8_t buffer[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_0}, 1,
                                                       buffer, sizeof(buffer), TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    *out_firmware_version = buffer[0] + (buffer[1] << 8);
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_keyboard_keys(tanmatsu_coprocessor_handle_t handle,
                                                 tanmatsu_coprocessor_keys_t* out_keys) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(
                            handle, handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_0}, 1,
                            (uint8_t*)out_keys, sizeof(tanmatsu_coprocessor_keys_t), TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_display_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t* out_brightness) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT}, 1,
                                                       out_brightness, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_display_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t brightness) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT,
                                                   brightness,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t* out_brightness) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT}, 1,
                                                       out_brightness, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t brightness) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT,
                                                   brightness,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_interrupt(tanmatsu_coprocessor_handle_t handle, bool* out_keyboard, bool* out_input,
                                             bool* out_pmic) {
    uint8_t value;
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit_receive(handle, handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_INTERRUPT},
                                       1, &value, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    if (out_keyboard) {
        *out_keyboard = (value >> 0) & 1;
    }
    if (out_input) {
        *out_input = (value >> 1) & 1;
    }
    if (out_pmic) {
        *out_pmic = (value >> 2) & 1;
    }
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_inputs(tanmatsu_coprocessor_handle_t handle,
                                          tanmatsu_coprocessor_inputs_t* out_inputs) {
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit_receive(handle, handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_INPUT}, 1,
                                       (uint8_t*)out_inputs, sizeof(tanmatsu_coprocessor_inputs_t),
                                       TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_outputs(tanmatsu_coprocessor_handle_t handle,
                                           tanmatsu_coprocessor_outputs_t* out_outputs) {
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit_receive(handle, handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_OUTPUT}, 1,
                                       (uint8_t*)out_outputs, sizeof(tanmatsu_coprocessor_outputs_t),
                                       TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_outputs(tanmatsu_coprocessor_handle_t handle,
                                           tanmatsu_coprocessor_outputs_t* outputs) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_OUTPUT,
                                                   outputs->raw,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_amplifier_enable(tanmatsu_coprocessor_handle_t handle, bool* out_enable) {
    tanmatsu_coprocessor_outputs_t outputs;
    ESP_RETURN_ON_ERROR(tanmatsu_coprocessor_get_outputs(handle, &outputs), TAG, "Communication fault");
    return outputs.amplifier_enable;
}

esp_err_t tanmatsu_coprocessor_set_amplifier_enable(tanmatsu_coprocessor_handle_t handle, bool enable) {
    tanmatsu_coprocessor_outputs_t outputs;
    ESP_RETURN_ON_ERROR(tanmatsu_coprocessor_get_outputs(handle, &outputs), TAG, "Communication fault");
    outputs.amplifier_enable = enable;
    return tanmatsu_coprocessor_set_outputs(handle, &outputs);
}

esp_err_t tanmatsu_coprocessor_get_camera_gpio0(tanmatsu_coprocessor_handle_t handle, bool* out_enable) {
    tanmatsu_coprocessor_outputs_t outputs;
    ESP_RETURN_ON_ERROR(tanmatsu_coprocessor_get_outputs(handle, &outputs), TAG, "Communication fault");
    return outputs.camera_gpio0;
}

esp_err_t tanmatsu_coprocessor_set_camera_gpio0(tanmatsu_coprocessor_handle_t handle, bool enable) {
    tanmatsu_coprocessor_outputs_t outputs;
    ESP_RETURN_ON_ERROR(tanmatsu_coprocessor_get_outputs(handle, &outputs), TAG, "Communication fault");
    outputs.camera_gpio0 = enable;
    return tanmatsu_coprocessor_set_outputs(handle, &outputs);
}

esp_err_t tanmatsu_coprocessor_get_radio_state(tanmatsu_coprocessor_handle_t handle,
                                               tanmatsu_coprocessor_radio_state_t* out_state) {
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL}, 1, (uint8_t*)out_state,
                                       sizeof(tanmatsu_coprocessor_radio_state_t), TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_radio_state(tanmatsu_coprocessor_handle_t handle,
                                               tanmatsu_coprocessor_radio_state_t state) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL,
                                                   (uint8_t)(state),
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    claim_i2c_bus(handle);
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait a bit (workaround for crash)
    release_i2c_bus(handle);
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_radio_disable(tanmatsu_coprocessor_handle_t handle) {
    return tanmatsu_coprocessor_set_radio_state(handle, tanmatsu_coprocessor_radio_state_disabled);
}

esp_err_t tanmatsu_coprocessor_radio_enable_application(tanmatsu_coprocessor_handle_t handle) {
    return tanmatsu_coprocessor_set_radio_state(handle, tanmatsu_coprocessor_radio_state_enabled_application);
}

esp_err_t tanmatsu_coprocessor_radio_enable_bootloader(tanmatsu_coprocessor_handle_t handle) {
    return tanmatsu_coprocessor_set_radio_state(handle, tanmatsu_coprocessor_radio_state_enabled_bootloader);
}

esp_err_t tanmatsu_coprocessor_get_real_time(tanmatsu_coprocessor_handle_t handle, uint32_t* out_value) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_0}, 1,
                                                       (uint8_t*)out_value, 4, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_real_time(tanmatsu_coprocessor_handle_t handle, uint32_t value) {
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_0,
                                                   (uint8_t)((value >> 0) & 0xFF),
                                                   (uint8_t)((value >> 8) & 0xFF),
                                                   (uint8_t)((value >> 16) & 0xFF),
                                                   (uint8_t)((value >> 24) & 0xFF),
                                               },
                                               5, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_backup_registers(tanmatsu_coprocessor_handle_t handle, uint8_t reg,
                                                    uint8_t* out_value, uint8_t length) {
    ESP_RETURN_ON_FALSE(reg >= TANMATSU_COPROCESSOR_BACKUP_NUM_REGS, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(length >= TANMATSU_COPROCESSOR_BACKUP_NUM_REGS - reg, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    ESP_RETURN_ON_FALSE(length < 1, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_BACKUP_0 + reg}, 1,
                                                       out_value, length, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_backup_registers(tanmatsu_coprocessor_handle_t handle, uint8_t reg,
                                                    const uint8_t* value, uint8_t length) {
    ESP_RETURN_ON_FALSE(reg >= TANMATSU_COPROCESSOR_BACKUP_NUM_REGS, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(length >= TANMATSU_COPROCESSOR_BACKUP_NUM_REGS - reg, ESP_ERR_INVALID_ARG, TAG,
                        "invalid argument");
    ESP_RETURN_ON_FALSE(length < 1, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    uint8_t buffer[TANMATSU_COPROCESSOR_BACKUP_NUM_REGS] = {TANMATSU_COPROCESSOR_I2C_REG_BACKUP_0 + reg};
    memcpy(&buffer[1], value, length);
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit(handle, handle->dev_handle, buffer, length + 1, TANMATSU_COPROCESSOR_TIMEOUT_MS), TAG,
        "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_communication_fault(tanmatsu_coprocessor_handle_t handle, bool* out_last,
                                                            bool* out_latch) {
    uint8_t value;
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_COMM_FAULT}, 1,
                                                       &value, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_last) {
        *out_last = value & 1;
    }

    if (out_latch) {
        *out_latch = (value >> 1) & 1;
    }

    return ESP_OK;
}

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t ntc_fault : 3;       // Temperature sensor fault
            uint8_t bat_fault : 1;       // Battery over voltage fault
            uint8_t chrg_fault : 2;      // Charge fault
            uint8_t boost_fault : 1;     // Boost mode fault
            uint8_t watchdog_fault : 1;  // Watchdog fault
        };
    };
} bq25895_reg0C_t;

esp_err_t tanmatsu_coprocessor_get_pmic_faults(tanmatsu_coprocessor_handle_t handle,
                                               tanmatsu_coprocessor_pmic_faults_t* out_faults) {
    bq25895_reg0C_t value;
    ESP_RETURN_ON_ERROR(
        ts_i2c_master_transmit_receive(handle, handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_FAULT},
                                       1, &value.raw, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");

    if (out_faults) {
        out_faults->watchdog = value.watchdog_fault;
        out_faults->boost = value.boost_fault;
        out_faults->chrg_input = value.chrg_fault == 1;
        out_faults->chrg_thermal = value.chrg_fault == 2;
        out_faults->chrg_safety = value.chrg_fault == 3;
        out_faults->batt_ovp = value.bat_fault;
        out_faults->ntc_cold = (value.ntc_fault & 3) == 1;
        out_faults->ntc_hot = (value.ntc_fault & 3) == 2;
        out_faults->ntc_boost = (value.ntc_fault >> 2) & 1;
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_pmic_adc_control(tanmatsu_coprocessor_handle_t handle, bool trigger,
                                                    bool continuous) {
    uint8_t value = 0;
    if (trigger) {
        value |= (1 << 0);
    }
    if (continuous) {
        value |= (1 << 1);
    }
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_CONTROL,
                                                   value,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_vbat(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vbat) {
    uint8_t value[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBAT_0}, 1,
                                                       value, 2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_vbat) {
        *out_vbat = value[0] | (value[1] << 8);
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_vsys(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vsys) {
    uint8_t value[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VSYS_0}, 1,
                                                       value, 2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_vsys) {
        *out_vsys = value[0] | (value[1] << 8);
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_ts(tanmatsu_coprocessor_handle_t handle, uint16_t* out_ts) {
    uint8_t value[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_TS_0}, 1,
                                                       value, 2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_ts) {
        *out_ts = value[0] | (value[1] << 8);
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_vbus(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vbus) {
    uint8_t value[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_VBUS_0}, 1,
                                                       value, 2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_vbus) {
        *out_vbus = value[0] | (value[1] << 8);
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_ichgr(tanmatsu_coprocessor_handle_t handle, uint16_t* out_ichgr) {
    uint8_t value[2];
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_ADC_ICHGR_0}, 1,
                                                       value, 2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_ichgr) {
        *out_ichgr = value[0] | (value[1] << 8);
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_pmic_charging_control(tanmatsu_coprocessor_handle_t handle, bool disable,
                                                         uint8_t speed) {
    uint8_t value = 0;
    if (disable) {
        value |= (1 << 0);
    }
    value |= ((speed & 3) << 1);
    printf("CURRENT: %u\r\n", speed);
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_PMIC_CHARGING_CONTROL,
                                                   value,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_charging_control(tanmatsu_coprocessor_handle_t handle, bool* out_disable,
                                                         uint8_t* out_speed) {
    uint8_t value;
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_CHARGING_CONTROL},
                                                       1, &value, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_disable) {
        *out_disable = (value >> 0) & 1;
    }

    if (out_speed) {
        *out_speed = (value >> 1) & 3;
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_pmic_charging_status(tanmatsu_coprocessor_handle_t handle,
                                                        bool* out_battery_attached, bool* out_usb_attached,
                                                        bool* out_charging_disabled, uint8_t* out_charging_status) {
    uint8_t value;
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit_receive(handle, handle->dev_handle,
                                                       (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_PMIC_CHARGING_STATUS},
                                                       1, &value, 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");

    if (out_battery_attached) {
        *out_battery_attached = (value >> 0) & 1;
    }

    if (out_usb_attached) {
        *out_usb_attached = (value >> 1) & 1;
    }

    if (out_charging_disabled) {
        *out_charging_disabled = (value >> 2) & 1;
    }

    if (out_charging_status) {
        *out_charging_status = (value >> 3) & 3;
    }

    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_pmic_otg_control(tanmatsu_coprocessor_handle_t handle, bool enable) {
    uint8_t value = 0;
    if (enable) {
        value |= (1 << 0);
    }
    ESP_RETURN_ON_ERROR(ts_i2c_master_transmit(handle, handle->dev_handle,
                                               (uint8_t[]){
                                                   TANMATSU_COPROCESSOR_I2C_REG_PMIC_OTG_CONTROL,
                                                   value,
                                               },
                                               2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}
