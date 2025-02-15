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
#include "drivers/serial.h"
#include "pg/rx.h"
#include "pg/piniobox.h"
#include "rx/rx.h"
#include "telemetry/telemetry.h"
#include "fc/config.h"
#include "drivers/pwm_output.h"
#include "sensors/gyro.h"
#include "io/vtx.h"
#include "io/ledstrip.h"
#include "fc/config.h"
#include "pg/piniobox.h"
<<<<<<< HEAD
=======
#include "pg/motor.h"
>>>>>>> betaflight/master
#include "common/axis.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "drivers/pwm_output.h"

#ifdef USE_TARGET_CONFIG

void targetConfiguration(void)
{
    pinioBoxConfigMutable()->permanentId[0] = 39;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    gyroConfigMutable()->gyro_sync_denom = 1;  // 8kHz gyro
    pidConfigMutable()->pid_process_denom = 1; // 8kHz PID
}
#endif
