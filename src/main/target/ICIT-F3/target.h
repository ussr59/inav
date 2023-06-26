/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "AFF3" // ALIENFLIGHT F3

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PB2
#define BRUSHED_ESC_AUTODETECT

// LED's V3
#define LED0                    PB3

#define BEEPER                  PA5

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT

// Using MPU6050 for the moment.
#define USE_IMU_MPU6050
#define IMU_MPU6050_ALIGN       CW270_DEG
#define MPU6050_I2C_BUS         BUS_I2C2

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW270_DEG
#define MPU6500_CS_PIN          PA15
#define MPU6500_SPI_BUS         BUS_SPI3

#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW270_DEG
#define MPU9250_CS_PIN          PA15
#define MPU9250_SPI_BUS         BUS_SPI3

// No baro support.
//#define USE_BARO
//#define USE_BARO_MS5611

// option to use MPU9150 or MPU9250 integrated AK89xx Mag
#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_MPU9250
#define MAG_MPU9250_ALIGN       CW0_DEG
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_LIS3MDL

#define USE_VCP
#define USE_UART1 // Not connected - TX (PB6) RX PB7 (AF7)
#define USE_UART3 // Not connected - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_PULLUP
#define USE_I2C_DEVICE_2        // SDA (PA10/AF4), SCL (PA9/AF4)
#define I2C2_SCL                PA9
#define I2C2_SDA                PA10


#define USE_SPI
#define USE_SPI_DEVICE_2

#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC_CHANNEL_1_PIN       PA4
#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define VBAT_SCALE_DEFAULT      20

#define USE_SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB12 (Pin 25)
#define BINDPLUG_PIN            PB12


#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_MOTOR_STOP)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SPEKTRUM2048
#define SERIALRX_UART           SERIAL_PORT_USART3
#define RX_CHANNELS_TAER

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define USE_SERIAL_4WAY_BLHELI_INTERFACE


// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))


#define PCA9685_I2C_BUS         BUS_I2C2

#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(16) | TIM_N(17) )
