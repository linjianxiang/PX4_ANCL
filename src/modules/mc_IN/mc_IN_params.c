/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_IN_params.c
 * Multicopter IN controller parameters.
 *
 * @author yunzhi <yunzhi2@ualberta.ca>
 */


/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_X_P, 0.09f);

/**
 * Gain for image around image y axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Y_P, 0.15f);

/**
 * Gain for image around image z axis  in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Z_P, 0.16f);


/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_X_I, 0.02f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Y_I, 0.02f);

/**
 * Integral gain for vertical velocity error
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Z_I, 0.02f);

/**
 * Gain for yaw
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_YAW_P,0.6f);


/**
 * x integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_X_I_MAX, 0.035f);

/**
 * y integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Y_I_MAX, 0.035f);

/**
 * z integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(IN_PID_Z_I_MAX, 0.035f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_X_D, 0.007f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.005
 * @max 0.1
 * @decimal 3  
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_Y_D, 0.01f);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_Z_D, 0f);

/**
 * Lowpass for x error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_X_D_LP, 0.1f);

/**
 * Lowpass for y error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_Y_D_LP, 0.1f);

/**
 * Lowpass for y error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_PID_Z_D_LP, 0.1f);





/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_POSX_P, 0.95f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_POSY_P, 0.95f);

/**
 * Proportional gain for vertical position error
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(IN_POSZ_P, 1.0f);

/**
 * thrust required for gravity
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter  IN Control
 */
PARAM_DEFINE_FLOAT(IN_G, 0.05f);

/**
 * yaw setpoint
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @group Multicopter  IN Control
 */
PARAM_DEFINE_FLOAT(IN_YAW_CONST, 0.0f);



