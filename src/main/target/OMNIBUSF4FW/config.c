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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "pg/pg.h"
#include "drivers/max7456.h"
#include "io/serial.h"

#include "config_helper.h"

static targetSerialPortFunction_t targetSerialPortFunction[] = {
#ifdef OMNIBUSF4V6
<<<<<<< HEAD
    { SERIAL_PORT_USART6, FUNCTION_RX_SERIAL },
#else
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_UART4,  FUNCTION_ESC_SENSOR },
=======
    { SERIAL_PORT_USART6, FUNCTION_RX_SERIAL },   
#else
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_UART4,  FUNCTION_ESC_SENSOR },    
>>>>>>> betaflight/4.0.x-maintenance
#endif
};

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
}
#endif
