/****************************************************************************
 *
 *   Copyright (c) 2014 ANCL Development Team. All rights reserved.
 *   Author: @Hui Xie <xie1@ualberta.ca>
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
 * @file mc_ibvs_main.cpp
 * Multicopter Image Based Visual Servoing controller.
 *
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>

#include "BlockIBVSController.hpp"

namespace ibvs
{
	static bool thread_should_exit = false;
	static bool thread_running = false;
	static int deamon_task;
	static BlockIBVSController *control;
}

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_ibvs_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mc_ibvs_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: mc_ibvs {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mc_ibvs_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (ibvs::thread_running) {
                        warnx("mc_ibvs already running");
			/* this is not an error */
			exit(0);
		}

		ibvs::thread_should_exit = false;

		ibvs::deamon_task = px4_task_spawn_cmd("mc_ibvs",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_POSITION_CONTROL,
						 4048,
						 mc_ibvs_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		ibvs::thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (ibvs::thread_running) {
                        struct img_moments_s data;
                        struct vehicle_image_attitude_setpoint_s img_sp;
                        int sub=-1;
                        int sub1=-1;
                        warnx("mc_ibvs is running");
                        sub = orb_subscribe(ORB_ID(img_moments));
                        sub1 = orb_subscribe(ORB_ID(vehicle_image_attitude_setpoint));
                        if(sub>0){
                            PX4_INFO("IMAGE MOMENTS RECEIVED");
                            orb_copy(ORB_ID(img_moments), sub, &data);
                            PX4_INFO("Timestamp: (%u)",data.usec);
                            PX4_INFO("s1 of Objects: (%.5f)",(double)data.s[0]);
                            PX4_INFO("s2 of Objects: (%.5f)",(double)data.s[1]);
                            PX4_INFO("s3 of Objects: (%.5f)",(double)data.s[2]);
                            PX4_INFO("s4 of Objects: (%.5f)",(double)data.s[3]);
                            PX4_INFO("Valid: %u",data.valid);
                        } else {
                                PX4_INFO("Could not subscribe to img_moments topic");
                        }
                        sub = orb_unsubscribe(sub);
                        if (sub1>0) {
                            PX4_INFO("MC_IBVS OUTPUT");
                            orb_copy(ORB_ID(vehicle_image_attitude_setpoint), sub1, &img_sp);
                            PX4_INFO("Timestamp: (%" PRIu64 ")",img_sp.timestamp);
                            PX4_INFO("Roll: (%.5f)",(double)img_sp.roll);
                            PX4_INFO("Pitch: (%.5f)",(double)img_sp.pitch);
                            PX4_INFO("Yaw: (%.5f)",(double)img_sp.yaw);
                            PX4_INFO("Thrust: (%.5f)",(double)img_sp.thrust);
                            if(img_sp.valid){
                                PX4_INFO("Valid: TRUE");
                            } else {
                                PX4_INFO("Valid: FALSE");
                            }
                        } else {
                                PX4_INFO("Could not subscribe to vehicle_image_attitude_setpoint topic");
                        }
                        sub1 = orb_unsubscribe(sub1);

		} else {
                        warnx("mc_ibvs not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int mc_ibvs_thread_main(int argc, char *argv[])
{

	warnx("starting");

	ibvs::control = new BlockIBVSController;

	if (ibvs::control == nullptr) {
		warnx("alloc failed");
		return 1;
	}

	ibvs::thread_running = true;

	while (!ibvs::thread_should_exit) {
		ibvs::control->update();
	}

	warnx("exiting.");
	delete ibvs::control;
	ibvs::control = nullptr;
	ibvs::thread_running = false;

	return 0;
}



