set(srcs "main.c"
    "drivers/nrf24l01.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver)
