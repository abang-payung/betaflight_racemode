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
#define TARGET_BOARD_IDENTIFIER "RACE"
#define USBD_PRODUCT_STRING     "RacePit"
#define USE_TARGET_CONFIG

<<<<<<< HEAD
<<<<<<< HEAD
=======
#define USE_HARDWARE_REVISION_DETECTION

>>>>>>> betaflight/4.0.x-maintenance
=======
#define USE_HARDWARE_REVISION_DETECTION

>>>>>>> betaflight/master
/*--------------LED----------------*/
#define LED0_PIN                PB9
#define LED1_PIN                PB8
/*---------------------------------*/

/*------------BEEPER---------------*/
#define USE_BEEPER
<<<<<<< HEAD
<<<<<<< HEAD
#define BEEPER_PIN              PB3
=======
#define BEEPER_PIN              PC3
>>>>>>> betaflight/4.0.x-maintenance
=======
#define BEEPER_PIN              PC3
>>>>>>> betaflight/master
#define BEEPER_INVERTED
/*---------------------------------*/

/*---------- VTX POWER SWITCH---------*/
#define USE_PINIO
#define PINIO1_PIN              PC0 // VTX power switcher
<<<<<<< HEAD
<<<<<<< HEAD
=======
#define PINIO2_PIN              PC8 // 2xCamera switcher
>>>>>>> betaflight/master
#define USE_PINIOBOX

/*----------CAMERA CONTROL---------*/
#define CAMERA_CONTROL_PIN      PA10
/*---------------------------------*/

/*------------SENSORS--------------*/
// MPU interrupt
#define USE_EXTI
<<<<<<< HEAD
#define MPU_INT_EXTI            PC4
=======
#define PINIO2_PIN              PC8 // 2xCamera switcher
#define USE_PINIOBOX

/*------------SENSORS--------------*/
// MPU interrupt
#define USE_EXTI
#define GYRO_1_INT_EXTI_PIN     PC4
>>>>>>> betaflight/4.0.x-maintenance
=======
#define GYRO_1_INT_EXTI_PIN     PC4
>>>>>>> betaflight/master

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_ACC

<<<<<<< HEAD
#define MPU6000_CS_PIN          SPI1_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW90_DEG_FLIP

<<<<<<< HEAD
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW90_DEG_FLIP
=======
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000

#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1
>>>>>>> betaflight/4.0.x-maintenance
=======
#define GYRO_1_CS_PIN           SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI1
>>>>>>> betaflight/master
/*---------------------------------*/


/*-------------OSD-----------------*/
#define USE_MAX7456
<<<<<<< HEAD
<<<<<<< HEAD
#define MAX7456_SPI_INSTANCE    SPI2  
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN 
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
/*---------------------------------*/

/*------------FLASH----------------*/
#define FLASH_CS_PIN            PA15
#define FLASH_SPI_INSTANCE      SPI3
=======
#define MAX7456_SPI_INSTANCE     SPI2
#define MAX7456_SPI_CS_PIN       SPI2_NSS_PIN
#define MAX7456_SPI_CLK          (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK      (SPI_CLOCK_FAST)
/*---------------------------------*/

/*------------FLASH----------------*/
#define FLASH_CS_PIN             PA15
#define FLASH_SPI_INSTANCE       SPI3
>>>>>>> betaflight/4.0.x-maintenance
=======
#define MAX7456_SPI_INSTANCE     SPI2
#define MAX7456_SPI_CS_PIN       SPI2_NSS_PIN
/*---------------------------------*/

/*------------FLASH----------------*/
#define FLASH_CS_PIN             PA15
#define FLASH_SPI_INSTANCE       SPI3
>>>>>>> betaflight/master

#define USE_FLASHFS
#define USE_FLASH_M25P16
/*---------------------------------*/

/*-----------USB-UARTs-------------*/
#define USE_VCP

#define USE_UART1
<<<<<<< HEAD
<<<<<<< HEAD
#define UART1_RX_PIN            PB7	
#define UART1_TX_PIN            PA9
=======
#define UART1_RX_PIN             PB7
#define UART1_TX_PIN             PA9
>>>>>>> betaflight/master

#define USE_UART2
#define UART2_RX_PIN             PA3
#define UART2_TX_PIN             PA2

#define USE_UART3
#define UART3_RX_PIN             PC11
#define UART3_TX_PIN             PC10
#define INVERTER_PIN_UART3       PC15

#define USE_UART4
#define UART4_TX_PIN             PA0
#define UART4_RX_PIN             PA1

#define USE_UART5
#define UART5_RX_PIN             PD2
#define UART5_TX_PIN             PC12

#define USE_UART6
#define UART6_RX_PIN             PC7
#define UART6_TX_PIN             PC6

<<<<<<< HEAD
#define SERIAL_PORT_COUNT       7 
=======
#define UART1_RX_PIN             PB7
#define UART1_TX_PIN             PA9

#define USE_UART2
#define UART2_RX_PIN             PA3
#define UART2_TX_PIN             PA2

#define USE_UART3
#define UART3_RX_PIN             PC11
#define UART3_TX_PIN             PC10
#define INVERTER_PIN_UART3       PC15

#define USE_UART4
#define UART4_TX_PIN             PA0
#define UART4_RX_PIN             PA1

#define USE_UART5
#define UART5_RX_PIN             PD2
#define UART5_TX_PIN             PC12

#define USE_UART6
#define UART6_RX_PIN             PC7
#define UART6_TX_PIN             PC6

#define SERIAL_PORT_COUNT        7
>>>>>>> betaflight/4.0.x-maintenance
=======
#define SERIAL_PORT_COUNT        7
>>>>>>> betaflight/master
/*---------------------------------*/

/*-------------SPIs----------------*/
#define USE_SPI

#define USE_SPI_DEVICE_1
<<<<<<< HEAD
<<<<<<< HEAD
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
=======
#define SPI1_NSS_PIN             PA4
#define SPI1_SCK_PIN             PA5
#define SPI1_MISO_PIN            PA6
#define SPI1_MOSI_PIN            PA7
>>>>>>> betaflight/master

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN             PB12
#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15

#define USE_SPI_DEVICE_3
<<<<<<< HEAD
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
=======
#define SPI1_NSS_PIN             PA4
#define SPI1_SCK_PIN             PA5
#define SPI1_MISO_PIN            PA6
#define SPI1_MOSI_PIN            PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN             PB12
#define SPI2_SCK_PIN             PB13
#define SPI2_MISO_PIN            PB14
#define SPI2_MOSI_PIN            PB15

#define USE_SPI_DEVICE_3
=======
>>>>>>> betaflight/master
#define SPI3_NSS_PIN             PA15
#define SPI3_SCK_PIN             PB3
#define SPI3_MISO_PIN            PB4
#define SPI3_MOSI_PIN            PB5
<<<<<<< HEAD
>>>>>>> betaflight/4.0.x-maintenance
=======
>>>>>>> betaflight/master
/*---------------------------------*/

/*-------------I2C-----------------*/
#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_3
<<<<<<< HEAD
<<<<<<< HEAD
#define I2C_DEVICE              (I2CDEV_3)
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9
=======
#define I2C_DEVICE               (I2CDEV_3)
#define I2C3_SCL                 PA8
#define I2C3_SDA                 PC9
>>>>>>> betaflight/4.0.x-maintenance
=======
#define I2C_DEVICE               (I2CDEV_3)
#define I2C3_SCL                 PA8
#define I2C3_SDA                 PC9
>>>>>>> betaflight/master
/*---------------------------------*/

/*-------------ADCs----------------*/
#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define VBAT_ADC_PIN                    PC2
#define CURRENT_METER_ADC_PIN           PC1
/*---------------------------------*/

/*-------------ESCs----------------*/
#define USE_ESCSERIAL
<<<<<<< HEAD
<<<<<<< HEAD
#define ESCSERIAL_TIMER_TX_PIN  PB0  // (HARDWARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3
#define DEFAULT_FEATURES        ( FEATURE_LED_STRIP | FEATURE_OSD | FEATURE_MOTOR_STOP )

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
=======
#define ESCSERIAL_TIMER_TX_PIN   PB0  // (HARDWARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
=======
#define ESCSERIAL_TIMER_TX_PIN   PB0  // (HARDWARE=0)
/*---------------------------------*/

/*--------DEFAULT VALUES-----------*/
>>>>>>> betaflight/master
#define DEFAULT_RX_FEATURE       FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER        SERIALRX_SBUS
#define SERIALRX_UART            SERIAL_PORT_USART3
#define DEFAULT_FEATURES         FEATURE_OSD

#define TARGET_IO_PORTA          0xffff
#define TARGET_IO_PORTB          0xffff
#define TARGET_IO_PORTC          0xffff
#define TARGET_IO_PORTD          (BIT(2))
<<<<<<< HEAD
>>>>>>> betaflight/4.0.x-maintenance
=======
>>>>>>> betaflight/master
/*---------------------------------*/

/*--------------TIMERS-------------*/
#define USABLE_TIMER_CHANNEL_COUNT  6
<<<<<<< HEAD
<<<<<<< HEAD
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
=======
#define USED_TIMERS              (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
>>>>>>> betaflight/4.0.x-maintenance
=======
#define USED_TIMERS              (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )
>>>>>>> betaflight/master
/*---------------------------------*/
