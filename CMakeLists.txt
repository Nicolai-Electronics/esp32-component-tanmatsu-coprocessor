idf_component_register(
    SRCS
        "tanmatsu_coprocessor"
    INCLUDE_DIRS
        "."
    REQUIRES
        "esp_driver_i2c"
        "esp_driver_gpio"
    )

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
