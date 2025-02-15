/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "FRF3"
#define USE_TARGET_CONFIG

<<<<<<< HEAD
// Removed to make the firmware fit into flash (in descending order of priority):
//#undef USE_GYRO_OVERFLOW_CHECK
//#undef USE_GYRO_LPF2

//#undef USE_ITERM_RELAX
//#undef USE_RC_SMOOTHING_FILTER

#undef USE_MSP_DISPLAYPORT
#undef USE_MSP_OVER_TELEMETRY

#undef USE_HUFFMAN
#undef USE_PINIO
#undef USE_PINIOBOX

#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_LTM
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_SUMH
#undef USE_PWM

#undef USE_BOARD_INFO
#undef USE_EXTENDED_CMS_MENUS
#undef USE_RTC_TIME
#undef USE_RX_MSP
#undef USE_ESC_SENSOR_INFO

=======
>>>>>>> betaflight/4.0.x-maintenance
#define LED0_PIN                PB3
#define USE_BEEPER
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU_INT, SDCardDetect
#define MPU_ADDRESS             0x69

#ifdef MYMPU6000
#define GYRO_1_SPI_INSTANCE     SPI2
#define GYRO_1_CS_PIN           PB12
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#else
#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6050
#endif

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3

#define SERIAL_PORT_COUNT       4
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7


#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  PB9  // (HARDARE=0)
#define USE_SPI

// include the max7456 driver
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB4

#define USE_SPI
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_1

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI1_NSS_PIN            PC14
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PB5
#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_PIN                   SPI1_NSS_PIN

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE

#define USE_ADC
#define VBAT_ADC_PIN                PA4
#define CURRENT_METER_ADC_PIN       PB2
#define ADC_INSTANCE                ADC2
#define ADC24_DMA_REMAP

#define USE_TRANSPONDER
#define REDUCE_TRANSPONDER_CURRENT_DRAW_WHEN_USB_CABLE_PRESENT

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_LED_STRIP | FEATURE_OSD)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8))
