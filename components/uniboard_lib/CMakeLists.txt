# components/mylib/CMakeLists.txt
idf_component_register(SRCS "util.c" "onewire.c" "lcd_1602_i2c.c"
                       INCLUDE_DIRS "./"
                       REQUIRES driver)