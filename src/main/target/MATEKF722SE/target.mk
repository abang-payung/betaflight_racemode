F7X2RE_TARGETS += $(TARGET)
<<<<<<< HEAD
FEATURES       += VCP ONBOARDFLASH SDCARD

TARGET_SRC = \
=======
FEATURES       += VCP ONBOARDFLASH SDCARD_SPI

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu.c \
>>>>>>> betaflight/4.0.x-maintenance
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
<<<<<<< HEAD
=======
            drivers/compass/compass_lis3mdl.c \
>>>>>>> betaflight/4.0.x-maintenance
            drivers/max7456.c
