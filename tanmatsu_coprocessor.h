// Tanmatsu coprocessor interface component
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Defines

#define TANMATSU_COPROCESSOR_KEYBOARD_NUM_REGS 9
#define TANMATSU_COPROCESSOR_BACKUP_NUM_REGS   84
#define TANMATSU_COPROCESSOR_TIMEOUT_MS        10000

// Types

typedef struct {
    union {
        uint8_t raw[TANMATSU_COPROCESSOR_KEYBOARD_NUM_REGS];
        struct {
            uint8_t key_esc   : 1;
            uint8_t key_f1    : 1;
            uint8_t key_f2    : 1;
            uint8_t key_f3    : 1;
            uint8_t key_tilde : 1;
            uint8_t key_1     : 1;
            uint8_t key_2     : 1;
            uint8_t key_3     : 1;

            uint8_t key_tab : 1;
            uint8_t key_q   : 1;
            uint8_t key_w   : 1;
            uint8_t key_e   : 1;
            uint8_t key_fn  : 1;
            uint8_t key_a   : 1;
            uint8_t key_s   : 1;
            uint8_t key_d   : 1;

            uint8_t key_shift_l   : 1;
            uint8_t key_z         : 1;
            uint8_t key_x         : 1;
            uint8_t key_c         : 1;
            uint8_t key_ctrl      : 1;
            uint8_t key_meta      : 1;
            uint8_t key_alt_l     : 1;
            uint8_t key_backslash : 1;

            uint8_t key_4 : 1;
            uint8_t key_5 : 1;
            uint8_t key_6 : 1;
            uint8_t key_7 : 1;
            uint8_t key_r : 1;
            uint8_t key_t : 1;
            uint8_t key_y : 1;
            uint8_t key_u : 1;

            uint8_t key_f : 1;
            uint8_t key_g : 1;
            uint8_t key_h : 1;
            uint8_t key_j : 1;
            uint8_t key_v : 1;
            uint8_t key_b : 1;
            uint8_t key_n : 1;
            uint8_t key_m : 1;

            uint8_t key_f4        : 1;
            uint8_t key_f5        : 1;
            uint8_t key_f6        : 1;
            uint8_t key_backspace : 1;
            uint8_t key_9         : 1;
            uint8_t key_0         : 1;
            uint8_t key_minus     : 1;
            uint8_t key_equals    : 1;

            uint8_t key_o               : 1;
            uint8_t key_p               : 1;
            uint8_t key_sqbracket_open  : 1;
            uint8_t key_sqbracket_close : 1;
            uint8_t key_l               : 1;
            uint8_t key_semicolon       : 1;
            uint8_t key_quote           : 1;
            uint8_t key_return          : 1;

            uint8_t key_dot     : 1;
            uint8_t key_slash   : 1;
            uint8_t key_up      : 1;
            uint8_t key_shift_r : 1;
            uint8_t key_alt_r   : 1;
            uint8_t key_left    : 1;
            uint8_t key_down    : 1;
            uint8_t key_right   : 1;

            uint8_t key_8         : 1;
            uint8_t key_i         : 1;
            uint8_t key_k         : 1;
            uint8_t key_comma     : 1;
            uint8_t key_space_l   : 1;
            uint8_t key_space_m   : 1;
            uint8_t key_space_r   : 1;
            uint8_t key_volume_up : 1;
        };
    };
} tanmatsu_coprocessor_keys_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t sd_card_detect   : 1;
            uint8_t headphone_detect : 1;
            uint8_t power_button     : 1;
        };
    };
} tanmatsu_coprocessor_inputs_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t amplifier_enable : 1;
            uint8_t camera_gpio0     : 1;
        };
    };
} tanmatsu_coprocessor_outputs_t;

typedef struct {
    bool watchdog;
    bool boost;
    bool chrg_input;
    bool chrg_thermal;
    bool chrg_safety;
    bool batt_ovp;
    bool ntc_cold;
    bool ntc_hot;
    bool ntc_boost;
} tanmatsu_coprocessor_pmic_faults_t;

typedef struct tanmatsu_coprocessor tanmatsu_coprocessor_t;
typedef tanmatsu_coprocessor_t*     tanmatsu_coprocessor_handle_t;

typedef void (*tanmatsu_coprocessor_keyboard_cb)(tanmatsu_coprocessor_handle_t, tanmatsu_coprocessor_keys_t*,
                                                 tanmatsu_coprocessor_keys_t*);
typedef void (*tanmatsu_coprocessor_input_cb)(tanmatsu_coprocessor_handle_t, tanmatsu_coprocessor_inputs_t*,
                                              tanmatsu_coprocessor_inputs_t*);
typedef void (*tanmatsu_coprocessor_faults_cb)(tanmatsu_coprocessor_handle_t, tanmatsu_coprocessor_pmic_faults_t*,
                                               tanmatsu_coprocessor_pmic_faults_t*);

typedef struct {
    gpio_num_t                       int_io_num;             /// GPIO to which the interrupt line is connected
    i2c_master_bus_handle_t          i2c_bus;                /// Handle of the I2C bus of the coprocessor
    uint16_t                         i2c_address;            /// I2C address of the coprocessor (7-bit)
    SemaphoreHandle_t                concurrency_semaphore;  /// Semaphore for making I2C bus operation thread safe
    tanmatsu_coprocessor_keyboard_cb on_keyboard_change;  /// Function called when the state of a keyboard key changes
    tanmatsu_coprocessor_input_cb    on_input_change;     /// Function called when the state of an input changes
    tanmatsu_coprocessor_faults_cb   on_faults_change;    /// Function called when PMIC fault status changes
} tanmatsu_coprocessor_config_t;

typedef enum {
    tanmatsu_coprocessor_radio_state_disabled            = 0,
    tanmatsu_coprocessor_radio_state_enabled_bootloader  = 1,
    tanmatsu_coprocessor_radio_state_enabled_application = 2,
} tanmatsu_coprocessor_radio_state_t;

typedef enum {
    TANMATSU_CHARGE_STATUS_NOT_CHARGING            = 0,
    TANMATSU_CHARGE_STATUS_PRE_CHARGING            = 1,
    TANMATSU_CHARGE_STATUS_FAST_CHARGING           = 2,
    TANMATSU_CHARGE_STATUS_CHARGE_TERMINATION_DONE = 3,
} tanmatsu_coprocessor_charger_status_t;

// Functions

esp_err_t tanmatsu_coprocessor_initialize(const tanmatsu_coprocessor_config_t* configuration,
                                          tanmatsu_coprocessor_handle_t*       out_handle);

esp_err_t tanmatsu_coprocessor_get_firmware_version(tanmatsu_coprocessor_handle_t handle,
                                                    uint16_t*                     out_firmware_version);

esp_err_t tanmatsu_coprocessor_get_keyboard_keys(tanmatsu_coprocessor_handle_t handle,
                                                 tanmatsu_coprocessor_keys_t*  out_keys);

esp_err_t tanmatsu_coprocessor_get_display_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t* out_brightness);
esp_err_t tanmatsu_coprocessor_set_display_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t brightness);

esp_err_t tanmatsu_coprocessor_get_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t* out_brightness);
esp_err_t tanmatsu_coprocessor_set_keyboard_backlight(tanmatsu_coprocessor_handle_t handle, uint8_t brightness);

esp_err_t tanmatsu_coprocessor_get_interrupt(tanmatsu_coprocessor_handle_t handle, bool* out_keyboard, bool* out_input,
                                             bool* out_pmic);

esp_err_t tanmatsu_coprocessor_get_inputs(tanmatsu_coprocessor_handle_t  handle,
                                          tanmatsu_coprocessor_inputs_t* out_inputs);

esp_err_t tanmatsu_coprocessor_get_outputs(tanmatsu_coprocessor_handle_t   handle,
                                           tanmatsu_coprocessor_outputs_t* out_outputs);
esp_err_t tanmatsu_coprocessor_set_outputs(tanmatsu_coprocessor_handle_t   handle,
                                           tanmatsu_coprocessor_outputs_t* outputs);

esp_err_t tanmatsu_coprocessor_get_amplifier_enable(tanmatsu_coprocessor_handle_t handle, bool* out_enable);
esp_err_t tanmatsu_coprocessor_set_amplifier_enable(tanmatsu_coprocessor_handle_t handle, bool enable);

esp_err_t tanmatsu_coprocessor_get_camera_gpio0(tanmatsu_coprocessor_handle_t handle, bool* out_enable);
esp_err_t tanmatsu_coprocessor_set_camera_gpio0(tanmatsu_coprocessor_handle_t handle, bool enable);

esp_err_t tanmatsu_coprocessor_get_radio_state(tanmatsu_coprocessor_handle_t       handle,
                                               tanmatsu_coprocessor_radio_state_t* out_state);
esp_err_t tanmatsu_coprocessor_set_radio_state(tanmatsu_coprocessor_handle_t      handle,
                                               tanmatsu_coprocessor_radio_state_t state);

esp_err_t tanmatsu_coprocessor_radio_disable(tanmatsu_coprocessor_handle_t handle);
esp_err_t tanmatsu_coprocessor_radio_enable_application(tanmatsu_coprocessor_handle_t handle);
esp_err_t tanmatsu_coprocessor_radio_enable_bootloader(tanmatsu_coprocessor_handle_t handle);

esp_err_t tanmatsu_coprocessor_get_real_time(tanmatsu_coprocessor_handle_t handle, uint32_t* out_value);
esp_err_t tanmatsu_coprocessor_set_real_time(tanmatsu_coprocessor_handle_t handle, uint32_t value);

esp_err_t tanmatsu_coprocessor_get_alarm_time(tanmatsu_coprocessor_handle_t handle, uint32_t* out_value);
esp_err_t tanmatsu_coprocessor_set_alarm_time(tanmatsu_coprocessor_handle_t handle, uint32_t value);

esp_err_t tanmatsu_coprocessor_power_off(tanmatsu_coprocessor_handle_t handle, bool enable_alarm_wakeup);

esp_err_t tanmatsu_coprocessor_get_backup_registers(tanmatsu_coprocessor_handle_t handle, uint8_t reg,
                                                    uint8_t* out_value, uint8_t length);
esp_err_t tanmatsu_coprocessor_set_backup_registers(tanmatsu_coprocessor_handle_t handle, uint8_t reg,
                                                    const uint8_t* value, uint8_t length);

esp_err_t tanmatsu_coprocessor_get_pmic_communication_fault(tanmatsu_coprocessor_handle_t handle, bool* out_last,
                                                            bool* out_latch);
esp_err_t tanmatsu_coprocessor_get_pmic_faults(tanmatsu_coprocessor_handle_t       handle,
                                               tanmatsu_coprocessor_pmic_faults_t* out_faults);
esp_err_t tanmatsu_coprocessor_set_pmic_adc_control(tanmatsu_coprocessor_handle_t handle, bool trigger,
                                                    bool continuous);
esp_err_t tanmatsu_coprocessor_get_pmic_vbat(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vbat);
esp_err_t tanmatsu_coprocessor_get_pmic_vsys(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vsys);
esp_err_t tanmatsu_coprocessor_get_pmic_ts(tanmatsu_coprocessor_handle_t handle, uint16_t* out_ts);
esp_err_t tanmatsu_coprocessor_get_pmic_vbus(tanmatsu_coprocessor_handle_t handle, uint16_t* out_vbus);
esp_err_t tanmatsu_coprocessor_get_pmic_ichgr(tanmatsu_coprocessor_handle_t handle, uint16_t* out_ichgr);
esp_err_t tanmatsu_coprocessor_set_pmic_charging_control(tanmatsu_coprocessor_handle_t handle, bool disable,
                                                         uint8_t speed);
esp_err_t tanmatsu_coprocessor_get_pmic_charging_control(tanmatsu_coprocessor_handle_t handle, bool* out_disable,
                                                         uint8_t* out_speed);
esp_err_t tanmatsu_coprocessor_get_pmic_charging_status(tanmatsu_coprocessor_handle_t handle,
                                                        bool* out_battery_attached, bool* out_usb_attached,
                                                        bool* out_charging_disabled, uint8_t* out_charging_status);
esp_err_t tanmatsu_coprocessor_get_pmic_otg_control(tanmatsu_coprocessor_handle_t handle, bool* out_enable);
esp_err_t tanmatsu_coprocessor_set_pmic_otg_control(tanmatsu_coprocessor_handle_t handle, bool enable);
esp_err_t tanmatsu_coprocessor_set_led_data(tanmatsu_coprocessor_handle_t handle, uint8_t* data, uint8_t length);
