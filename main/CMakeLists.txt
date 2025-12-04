set(srcs "main.c"
        "ds18b20_custom.c"
        "adc_sensors.c"
        "yl69.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
