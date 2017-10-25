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
 * @file mc_ibvs_params.c
 * Multicopter IBVS controller parameters.
 *
 * @author Geoff <gfink@ualberta.ca>
 */


/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFX_P, 0.1f);

/**
 * Gain for image around image y axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFY_P, 0.1f);

/**
 * Gain for image around image z axis  in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFZ_P, 0.1f);

/**
 * Gain for velocity around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_VX_P, 0.1f);

/**
 * Gain for velocity around image yx axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_VY_P,0.1f);

/**
 * Gain for velocity around image z axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_VZ_P, 0.1f);

/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFX_I, 0.0f);

/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFY_I, 0.0f);

/**
 * Gain for integral
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFZ_I, 0.0f);

/**
 * Gain for yaw
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_YAW_P,0.6f);

/**
 * Thrust Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_T_MIN, 0.1f);

/**
 * Thrust Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_T_MAX, 0.9f);


/**
 * s1 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFX_I_MAX, 0.035f);

/**
 * s2 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFY_I_MAX, 0.035f);

/**
 * s3 integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_IFZ_I_MAX, 0.035f);

/**
 * thrust required for gravity
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_G, 0.3f);

/**
 * Gain for controller input roll on v1
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KL1_R, 0.8f);

/**
 * Gain for controller input roll
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KL2_R, 0.3f);

/**
 * Gain for controller input pitch on v2
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KL1_P, 0.8f);

/**
 * Gain for controller input pitch
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KL2_P, 0.15f);

/**
 * Gain for controller input thrust
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KH1, 1.1f);

/**
 * Gain for controller input thrust
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KH2, 0.3f);

/**
 * Gain for controller input thrust
 *
 * @unit norm
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IBVS Control
 */
PARAM_DEFINE_FLOAT(IBVS_KPSI, 0.6f);
