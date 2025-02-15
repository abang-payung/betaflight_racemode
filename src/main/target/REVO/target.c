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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
#if defined(ELINF405)
<<<<<<< HEAD
    DEF_TIM(TIM11, CH1,  PB9, TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM8,  CH2N, PB0, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM8,  CH3N, PB1, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM2,  CH4,  PA3, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM2,  CH3,  PA2, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM1,  CH1,  PA8, TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM3,  CH4,  PC9, TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_PPM | TIM_USE_LED,   0, 0),
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_ANY, 0, 0), // Camera control
=======
    DEF_TIM(TIM3,  CH3,  PB0, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM3,  CH4,  PB1, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM2,  CH4,  PA3, TIM_USE_MOTOR, 0, 1),
    DEF_TIM(TIM2,  CH3,  PA2, TIM_USE_MOTOR, 0, 0),
    DEF_TIM(TIM1,  CH1,  PA8, TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM8,  CH4,  PC9, TIM_USE_ANY,   0, 0),
    DEF_TIM(TIM4,  CH1,  PB6, TIM_USE_PPM | TIM_USE_LED,   0, 0),
    DEF_TIM(TIM4,  CH2,  PB7, TIM_USE_CAMERA_CONTROL, 0, 0),
>>>>>>> betaflight/4.0.x-maintenance
#else
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_PWM | TIM_USE_PPM,   0, 0), // PPM (5th pin on FlexiIO port)
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_PWM,                 0, 0), // S2_IN
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_PWM,                 0, 0), // S3_IN
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_PWM,                 0, 0), // S4_IN
    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_PWM,                 0, 0), // S5_IN
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_PWM,                 0, 0), // S6_IN
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0, 0), // S1_OUT D1_ST7
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0, 0), // S2_OUT D1_ST2
    DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_MOTOR,               0, 1), // S3_OUT D1_ST6
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,               0, 0), // S4_OUT D1_ST1
#ifdef REVOLT
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_LED,                 0, 0), // LED for REVOLT D1_ST0
#elif defined(AIRBOTF4) || defined(AIRBOTF4SD)
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0, 0), // S5_OUT
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_MOTOR,               0, 0), // S6_OUT
    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_LED,                 0, 0), // LED D1_ST0, n/a on older AIRBOTF4
#else
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR | TIM_USE_LED, 0, 0), // S5_OUT / LED
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MOTOR,               0, 0), // S6_OUT D1_ST2
#endif
#endif
};
