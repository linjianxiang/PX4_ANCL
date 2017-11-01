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
 * @file mc_NPID_params.c
 * Multicopter NPID controller parameters.
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
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_X_P, 0.1f);

/**
 * Gain for image around image y axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Y_P, 0.1f);

/**
 * Gain for image around image z axis  in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Z_P, 0.1f);


/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_X_I, 0.0f);

/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Y_I, 0.0f);

/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Z_I, 0.0f);

/**
 * Gain for yaw
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_YAW_P,0.6f);


/**
 * s1 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_X_I_MAX, 0.035f);

/**
 * s2 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Y_I_MAX, 0.035f);

/**
 * s3 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter NPID Control
 */
PARAM_DEFINE_FLOAT(NPID_PID_Z_I_MAX, 0.035f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_X_D, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_Y_D, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_Z_D, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_X_D_LP, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_Y_D_LP, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_PID_Z_D_LP, 0.1f);





/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_POSX_P, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_POSY_P, 0.1f);

/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter NPID Control
 */

PARAM_DEFINE_FLOAT(NPID_POSZ_P, 0.1f);

