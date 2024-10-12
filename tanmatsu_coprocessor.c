/*
 * SPDX-FileCopyrightText: 2024 Nicolai Electronics
 *
 * SPDX-License-Identifier: MIT
 */

#include "tanmatsu_coprocessor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Registers
#define TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_0         0   // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_1         1   // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_0           2   // 9 bytes
#define TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT_0  11  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT_1  12  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT_0 13  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT_1 14  // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_INPUT                15
#define TANMATSU_COPROCESSOR_I2C_REG_OUTPUT               16
#define TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL        17
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_0          18  // LSB
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_1          19
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_2          20
#define TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_3          21   // MSB
#define TANMATSU_COPROCESSOR_I2C_REG_BACKUP_0             22   // 84 bytes
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_BATTERY_CONTROL 106  // NOT IMPLEMENTED
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_BATTERY_STATUS  107  // NOT IMPLEMENTED
#define TANMATSU_COPROCESSOR_I2C_REG_PMIC_OTG_CONTROL     108  // NOT IMPLEMENTED

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

    while (true) {
        // Wait for interrupt
        xSemaphoreTake(handle->interrupt_semaphore, portMAX_DELAY);

        // Claim I2C bus
        if (handle->configuration.concurrency_semaphore != NULL) {
            xSemaphoreTake(handle->configuration.concurrency_semaphore, portMAX_DELAY);
        }

        // Read keyboard state
        ESP_ERROR_CHECK(tanmatsu_coprocessor_get_keyboard_keys(handle, &keys));

        // Read input state
        ESP_ERROR_CHECK(tanmatsu_coprocessor_get_inputs(handle, &inputs));

        // Release I2C bus
        if (handle->configuration.concurrency_semaphore != NULL) {
            xSemaphoreGive(handle->configuration.concurrency_semaphore);
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

        memcpy(&prev_keys, &keys, sizeof(tanmatsu_coprocessor_keys_t));
        memcpy(&prev_inputs, &inputs, sizeof(tanmatsu_coprocessor_inputs_t));
    }
}

IRAM_ATTR static void tanmatsu_coprocessor_interrupt_handler(void* pvParameters) {
    tanmatsu_coprocessor_handle_t handle = (tanmatsu_coprocessor_handle_t)pvParameters;
    xSemaphoreGiveFromISR(handle->interrupt_semaphore, NULL);
    portYIELD_FROM_ISR();
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
        assert(xTaskCreate(tanmatsu_coprocessor_interrupt_thread_entry, "Tanmatsu coprocessor interrupt task", 2048,
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
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_0}, 1,
                                    buffer, sizeof(buffer), TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    *out_firmware_version = buffer[0] + (buffer[1] << 8);
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_keyboard_keys(tanmatsu_coprocessor_handle_t handle,
                                                 tanmatsu_coprocessor_keys_t* out_keys) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(
                            handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_0}, 1,
                            (uint8_t*)out_keys, sizeof(tanmatsu_coprocessor_keys_t), TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_display_backlight(tanmatsu_coprocessor_handle_t handle, uint16_t* out_brightness) {
    uint8_t buffer[2];
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT_0},
                                    1, buffer, sizeof(buffer), TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    *out_brightness = buffer[0] + (buffer[1] << 8);
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_display_backlight(tanmatsu_coprocessor_handle_t handle, uint16_t brightness) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle,
                                            (uint8_t[]){
                                                TANMATSU_COPROCESSOR_I2C_REG_DISPLAY_BACKLIGHT_0,
                                                brightness & 0xFF,
                                                (brightness >> 8) & 0xFF,
                                            },
                                            3, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint16_t* out_brightness) {
    uint8_t buffer[2];
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT_0},
                                    1, buffer, sizeof(buffer), TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    *out_brightness = buffer[0] + (buffer[1] << 8);
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint16_t brightness) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle,
                                            (uint8_t[]){
                                                TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT_0,
                                                brightness & 0xFF,
                                                (brightness >> 8) & 0xFF,
                                            },
                                            3, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_inputs(tanmatsu_coprocessor_handle_t handle,
                                          tanmatsu_coprocessor_inputs_t* out_inputs) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_INPUT},
                                                    1, (uint8_t*)out_inputs, sizeof(tanmatsu_coprocessor_inputs_t),
                                                    TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_get_outputs(tanmatsu_coprocessor_handle_t handle,
                                           tanmatsu_coprocessor_outputs_t* out_outputs) {
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_OUTPUT}, 1,
                                    (uint8_t*)out_outputs, sizeof(tanmatsu_coprocessor_outputs_t),
                                    TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_outputs(tanmatsu_coprocessor_handle_t handle,
                                           tanmatsu_coprocessor_outputs_t* outputs) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle,
                                            (uint8_t[]){
                                                TANMATSU_COPROCESSOR_I2C_REG_KEYBOARD_BACKLIGHT_0,
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
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL}, 1,
                                    (uint8_t*)out_state, sizeof(tanmatsu_coprocessor_radio_state_t),
                                    TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_radio_state(tanmatsu_coprocessor_handle_t handle,
                                               tanmatsu_coprocessor_radio_state_t state) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle,
                                            (uint8_t[]){
                                                TANMATSU_COPROCESSOR_I2C_REG_RADIO_CONTROL,
                                                (uint8_t)(state),
                                            },
                                            2, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
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
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_RTC_VALUE_0}, 1,
                                    (uint8_t*)out_value, 4, TANMATSU_COPROCESSOR_TIMEOUT_MS),
        TAG, "Communication fault");
    return ESP_OK;
}

esp_err_t tanmatsu_coprocessor_set_real_time(tanmatsu_coprocessor_handle_t handle, uint32_t value) {
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle,
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
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(handle->dev_handle, (uint8_t[]){TANMATSU_COPROCESSOR_I2C_REG_BACKUP_0 + reg}, 1,
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
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle->dev_handle, buffer, length + 1, TANMATSU_COPROCESSOR_TIMEOUT_MS),
                        TAG, "Communication fault");
    return ESP_OK;
}
