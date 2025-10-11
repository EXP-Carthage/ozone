idf_component_register(
    SRCS "anemometer.c" "i2c.c" "tasks.c" "wind_vane.c" "zigbee.c"
    INCLUDE_DIRS "include"
    REQUIRES ads1115 bh1750 bme280 driver esp-zigbee-lib nvs_flash
)
