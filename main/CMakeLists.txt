idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_adc
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES uniboard_lib)