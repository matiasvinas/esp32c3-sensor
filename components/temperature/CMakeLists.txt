idf_component_register(SRCS "temperature.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ds18b20 onewire_bus)