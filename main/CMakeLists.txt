set(srcs "main.c"
        "board.c"
        "ds18b20_custom.c"
        "adc_sensors.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
