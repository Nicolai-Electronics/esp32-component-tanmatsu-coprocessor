# Tanmatsu coprocessor component

[![Component Registry](https://components.espressif.com/components/nicolaielectronics/tanmatsu_coprocessor/badge.svg)](https://components.espressif.com/components/nicolaielectronics/tanmatsu_coprocessor)

This component implements a driver for the I2C interface of the CH32V203 coprocessor on Tanmatsu.

You can find the firmware running on the CH32V203 coprocessor in the [Tanmatsu coprocessor firmware](https://github.com/nicolai-electronics/tanmatsu-coprocessor-firmware) repository.

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.

```
idf.py add-dependency "nicolaielectronics/tanmatsu_coprocessor"
```

Alternatively, you can manually create an `idf_component.yml` file for your project (more information in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html)) or clone this repository into your projects components folder (be sure to name the folder tanmatsu_coprocessor).
