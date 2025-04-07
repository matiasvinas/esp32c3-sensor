set(srcs "main.c"
        "board.c"
        "yl69.c"
        "ds18b20_custom.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
