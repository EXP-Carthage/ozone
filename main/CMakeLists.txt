idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ads1115 bh1750 bme280 core
)
