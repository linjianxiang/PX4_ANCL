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
 * @file mc_NewPID_main.cpp
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

#include "BlockNewPIDController.hpp"

namespace NewPID
{
	static bool thread_should_exit = false;
	static bool thread_running = false;
	static int deamon_task;
	static BlockNewPIDController *control;
}

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_NewPID_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mc_NewPID_thread_main(int argc, char *argv[]);

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

	fprintf(stderr, "usage: mc_NewPID {start|stop|status} [-p <additional params>]\n\n");
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
int mc_NewPID_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (NewPID::thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		NewPID::thread_should_exit = false;

		NewPID::deamon_task = px4_task_spawn_cmd("mc_NewPID",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_POSITION_CONTROL,
						 4048,
						 mc_NewPID_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		NewPID::thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (NewPID::thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int mc_NewPID_thread_main(int argc, char *argv[])
{

	warnx("starting");

	NewPID::control = new BlockNewPIDController;

	if (NewPID::control == nullptr) {
		warnx("alloc failed");
		return 1;
	}

	NewPID::thread_running = true;

	while (!NewPID::thread_should_exit) {
		NewPID::control->update();
	}

	warnx("exiting.");
	delete NewPID::control;
	NewPID::control = nullptr;
	NewPID::thread_running = false;

	return 0;
}



