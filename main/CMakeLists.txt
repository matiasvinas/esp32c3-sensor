set(srcs "main.c"
        "board.c"
        "yl69.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
