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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_reset.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/rpm_filter.h"

#include "pid.h"

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

static FAST_RAM_ZERO_INIT uint8_t antiGravityMode;
static FAST_RAM_ZERO_INIT float antiGravityThrottleHpf;
static FAST_RAM_ZERO_INIT uint16_t itermAcceleratorGain;
static FAST_RAM float antiGravityOsdCutoff = 1.0f;
static FAST_RAM_ZERO_INIT bool antiGravityEnabled;
static FAST_RAM_ZERO_INIT bool zeroThrottleItermReset;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#ifdef STM32F10X
#define PID_PROCESS_DENOM_DEFAULT       1
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT       4
#else
#define PID_PROCESS_DENOM_DEFAULT       2
#endif
#if defined(USE_D_MIN)
#define D_MIN_GAIN_FACTOR 0.00005f
#define D_MIN_SETPOINT_GAIN_FACTOR 0.00005f
#define D_MIN_RANGE_HZ 80    // Biquad lowpass input cutoff to peak D around propwash frequencies
#define D_MIN_LOWPASS_HZ 10  // PT1 lowpass cutoff to smooth the boost effect
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#ifdef USE_AIRMODE_LPF
static FAST_RAM_ZERO_INIT float airmodeThrottleOffsetLimit;
#endif

#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 9);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 60, 35, 70 },
            [PID_PITCH] = { 46, 70, 38, 75 },
            [PID_YAW] =   { 35, 100, 0, 0 },
            [PID_LEVEL] = { 50, 50, 0, 0},
          },

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID

// Seperate horizon hacked to racemode and angle so ignoring pitch axis on racemode doesnt break angle mode
if ((FLIGHT_MODE(HORIZON_MODE)) && ((axis != FD_YAW)&&(axis != FD_PITCH))) {
          currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
}
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) && axis != FD_YAW) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && acroTrainerActive && !inCrashRecoveryMode && !launchControlActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y
#if defined(USE_ACC)
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);
#endif

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

#if defined(USE_ITERM_RELAX)
        if (!launchControlActive && !inCrashRecoveryMode) {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate * tpaFactorKp;
        if (axis == FD_YAW) {
            pidData[axis].P = ptermYawLowpassApplyFn((filter_t *) &ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
#ifdef USE_LAUNCH_CONTROL
        // if launch control is active override the iterm gains
        const float Ki = launchControlActive ? launchControlKi : pidCoefficient[axis].Ki;
#else
        const float Ki = pidCoefficient[axis].Ki;
#endif
        pidData[axis].I = constrainf(previousIterm + Ki * itermErrorRate * dynCi, -itermLimit, itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
        pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
        previousPidSetpoint[axis] = currentPidSetpoint;

#ifdef USE_RC_SMOOTHING_FILTER
        pidSetpointDelta = applyRcSmoothingDerivativeFilter(axis, pidSetpointDelta);
#endif // USE_RC_SMOOTHING_FILTER

        // -----calculate D component
        // disable D if launch control is active
        if ((pidCoefficient[axis].Kd > 0) && !launchControlActive){

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta =
                - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;

#if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif

            float dMinFactor = 1.0f;
#if defined(USE_D_MIN)
            if (dMinPercent[axis] > 0) {
                float dMinGyroFactor = biquadFilterApply(&dMinRange[axis], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * dMinGyroGain;
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = dMinPercent[axis] + (1.0f - dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt1FilterApply(&dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_MIN, 0, lrintf(dMinGyroFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 1, lrintf(dMinSetpointFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 2, lrintf(pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_MIN, 3, lrintf(pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                }
            }
#endif
            pidData[axis].D = pidCoefficient[axis].Kd * delta * tpaFactor * dMinFactor;
        } else {
            pidData[axis].D = 0;
        }
        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
        // Only enable feedforward for rate mode and if launch control is inactive
        const float feedforwardGain = (flightModeFlags || launchControlActive) ? 0.0f : pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // no transition if feedForwardTransition == 0
            float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
            pidData[axis].F = feedforwardGain * transition * pidSetpointDelta * pidFrequency;

#if defined(USE_SMART_FEEDFORWARD)
            applySmartFeedforward(axis);
#endif
        } else {
            pidData[axis].F = 0;
        }

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
#endif
        // calculating the PID sum
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && useIntegratedYaw) {
            pidData[axis].Sum += pidSum * dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * integratedYawRelax / 100000.0f * dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (zeroThrottleItermReset) {
        pidResetIterm();
    }
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != antiGravityEnabled) {
        // reset the accelerator on state changes
        itermAccelerator = 1.0f;
    }
    antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (dynLpfFilter != DYN_LPF_NONE) {
        const unsigned int cutoffFreq = fmax(dynThrottle(throttle) * dynLpfMax, dynLpfMin);

         if (dynLpfFilter == DYN_LPF_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, dT));
            }
        } else if (dynLpfFilter == DYN_LPF_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
        }
    }
}
#endif

void pidSetItermReset(bool enabled)
{
    zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}
