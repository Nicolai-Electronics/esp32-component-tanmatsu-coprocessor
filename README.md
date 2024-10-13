# Tanmatsu coprocessor component

[![Component Registry](https://components.espressif.com/components/nicolaielectronics/tanmatsu-coprocessor/badge.svg)](https://components.espressif.com/components/nicolaielectronics/esp32-component-tanmatsu-coprocessor)

This component implements a driver for the coprocessor on the Tanmatsu and WHY2025 badge.

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.

```
    idf.py add-dependency "nicolaielectronics/tanmatsu_coprocessor"
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example

```
void coprocessor_keyboard_callback(tanmatsu_coprocessor_handle_t handle, tanmatsu_coprocessor_keys_t* prev_keys,
                                   tanmatsu_coprocessor_keys_t* keys) {
    printf("Keyboard state changed: %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", keys->raw[0], keys->raw[1],
           keys->raw[2], keys->raw[3], keys->raw[4], keys->raw[5], keys->raw[6], keys->raw[7], keys->raw[8]);
    if (keys->key_esc) {
        printf("ESC ");
    }
    if (keys->key_f1) {
        printf("F1 ");
    }
    if (keys->key_f2) {
        printf("F2 ");
    }
    if (keys->key_f3) {
        printf("F3 ");
    }
    if (keys->key_tilde) {
        printf("~ ");
    }
    if (keys->key_1) {
        printf("1 ");
    }
    if (keys->key_2) {
        printf("2 ");
    }
    if (keys->key_3) {
        printf("3 ");
    }
    if (keys->key_tab) {
        printf("TAB ");
    }
    if (keys->key_q) {
        printf("q ");
    }
    if (keys->key_w) {
        printf("w ");
    }
    if (keys->key_e) {
        printf("e ");
    }
    if (keys->key_fn) {
        printf("FN ");
    }
    if (keys->key_a) {
        printf("a ");
    }
    if (keys->key_s) {
        printf("s ");
    }
    if (keys->key_d) {
        printf("d ");
    }
    if (keys->key_shift_l) {
        printf("SHIFT_L ");
    }
    if (keys->key_z) {
        printf("Z ");
    }
    if (keys->key_x) {
        printf("X ");
    }
    if (keys->key_c) {
        printf("C ");
    }
    if (keys->key_ctrl) {
        printf("CTRL ");
    }
    if (keys->key_meta) {
        printf("META ");
    }
    if (keys->key_alt_l) {
        printf("ALT_L ");
    }
    if (keys->key_backslash) {
        printf("BACKSLASH ");
    }
    if (keys->key_4) {
        printf("4 ");
    }
    if (keys->key_5) {
        printf("5 ");
    }
    if (keys->key_6) {
        printf("6 ");
    }
    if (keys->key_7) {
        printf("7 ");
    }
    if (keys->key_r) {
        printf("r ");
    }
    if (keys->key_t) {
        printf("t ");
    }
    if (keys->key_y) {
        printf("y ");
    }
    if (keys->key_u) {
        printf("u ");
    }
    if (keys->key_f) {
        printf("f ");
    }
    if (keys->key_g) {
        printf("g ");
    }
    if (keys->key_h) {
        printf("h ");
    }
    if (keys->key_j) {
        printf("j ");
    }
    if (keys->key_v) {
        printf("v ");
    }
    if (keys->key_b) {
        printf("b ");
    }
    if (keys->key_n) {
        printf("n ");
    }
    if (keys->key_m) {
        printf("m ");
    }
    if (keys->key_f4) {
        printf("F4 ");
    }
    if (keys->key_f5) {
        printf("F5 ");
    }
    if (keys->key_f6) {
        printf("F6 ");
    }
    if (keys->key_backspace) {
        printf("BACKSPACE ");
    }
    if (keys->key_9) {
        printf("9 ");
    }
    if (keys->key_0) {
        printf("0 ");
    }
    if (keys->key_minus) {
        printf("- ");
    }
    if (keys->key_equals) {
        printf("= ");
    }
    if (keys->key_o) {
        printf("o ");
    }
    if (keys->key_p) {
        printf("p ");
    }
    if (keys->key_sqbracket_open) {
        printf("[ ");
    }
    if (keys->key_sqbracket_close) {
        printf("] ");
    }
    if (keys->key_l) {
        printf("l ");
    }
    if (keys->key_semicolon) {
        printf("; ");
    }
    if (keys->key_quote) {
        printf("' ");
    }
    if (keys->key_return) {
        printf("RETURN ");
    }
    if (keys->key_dot) {
        printf(". ");
    }
    if (keys->key_slash) {
        printf("/ ");
    }
    if (keys->key_up) {
        printf("UP ");
    }
    if (keys->key_shift_r) {
        printf("SHIFT_R ");
    }
    if (keys->key_alt_r) {
        printf("ALT_R ");
    }
    if (keys->key_left) {
        printf("LEFT ");
    }
    if (keys->key_down) {
        printf("DOWN ");
    }
    if (keys->key_right) {
        printf("RIGHT ");
    }
    if (keys->key_8) {
        printf("8 ");
    }
    if (keys->key_i) {
        printf("i ");
    }
    if (keys->key_k) {
        printf("k ");
    }
    if (keys->key_comma) {
        printf(", ");
    }
    if (keys->key_space_l) {
        printf("SPACE_L ");
    }
    if (keys->key_space_m) {
        printf("SPACE_M ");
    }
    if (keys->key_space_r) {
        printf("SPACE_R ");
    }
    if (keys->key_volume_up) {
        printf("VOLUME_UP ");
    }
    printf("\r\n");
}

void coprocessor_input_callback(tanmatsu_coprocessor_handle_t handle, tanmatsu_coprocessor_inputs_t* prev_inputs,
                                tanmatsu_coprocessor_inputs_t* inputs) {
    if (inputs->sd_card_detect != prev_inputs->sd_card_detect) {
        ESP_LOGW(TAG, "SD card %s", inputs->sd_card_detect ? "inserted" : "removed");
    }

    if (inputs->headphone_detect != prev_inputs->headphone_detect) {
        ESP_LOGW(TAG, "Audio jack %s", inputs->headphone_detect ? "inserted" : "removed");
    }

    if (inputs->power_button != prev_inputs->power_button) {
        ESP_LOGW(TAG, "Power button %s", inputs->power_button ? "pressed" : "released");
    }
}
```

```
gpio_install_isr_service(0);

tanmatsu_coprocessor_handle_t coprocessor_handle = NULL;

SemaphoreHandle_t i2c_concurrency_semaphore = NULL;
i2c_concurrency_semaphore = xSemaphoreCreateBinary();
xSemaphoreGive(i2c_concurrency_semaphore);

i2c_master_bus_config_t i2c_master_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = 10,
    .sda_io_num = 9,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t i2c_bus_handle = NULL;

ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &i2c_bus_handle));

tanmatsu_coprocessor_config_t coprocessor_config = {
    .int_io_num = 6,
    .i2c_bus = i2c_bus_handle,
    .i2c_address = 0x5F,
    .concurrency_semaphore = i2c_concurrency_semaphore,
    .on_keyboard_change = coprocessor_keyboard_callback,
    .on_input_change = coprocessor_input_callback,
};

ESP_ERROR_CHECK(tanmatsu_coprocessor_initialize(&coprocessor_config, &coprocessor_handle));
```
