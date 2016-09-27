/****************************************************************************
 *
 *   Copyright (c) 2013 - 2015 PX4 Development Team. All rights reserved.
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
 * @file vicon.cpp
 * Vicon Status information.
 *
 * @author Geoff <gfink@ualbert.ca>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/vicon.h>


/**
 * 
 *
 * @ingroup apps
 */
extern "C" __EXPORT int vicon_main(int argc, char *argv[]);


int vicon_main(int argc, char *argv[])
{
	int sub=-1;
	struct vicon_s data;
	memset(&data,0,sizeof(data));

	if (argc < 2) {
		warnx("usage: vicon {status}");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		sub = orb_subscribe(ORB_ID(vicon));
		if (sub>0) {
			orb_copy(ORB_ID(vicon), sub, &data);
			PX4_INFO("Vicon:");
			PX4_INFO("pos: (%2.3f,%2.3f,%2.3f)",(double)data.x,(double)data.y,(double)data.z);
			PX4_INFO("vel: (%2.3f,%2.3f,%2.3f)",(double)data.vx,(double)data.vy,(double)data.vz);
			PX4_INFO("q: (%2.3f,%2.3f,%2.3f,%2.3f)",(double)data.q[0],(double)data.q[1],(double)data.q[2],(double)data.q[3]);
			PX4_INFO("received @ %" PRIu64 " / %" PRIu64 "\n      sent @ %" PRIu64, data.t_local, hrt_absolute_time(), data.t_remote);
			sub = orb_unsubscribe(sub);
		} else {
			PX4_INFO("Could not subscribe to Vicon topic");
		}
		return 0;
	}
	warnx("unrecognized command");
	return 1;
}

