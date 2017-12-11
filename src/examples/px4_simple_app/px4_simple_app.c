/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.cpp
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/vehicle_attitude.h>

//#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	//PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/*
		 advertise attitude topic 
		struct vehicle_attitude_s att;
		memset(&att, 0, sizeof(att));
		orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
	*/
	
	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_sub_fd;
	fds[0].events= POLLIN ;
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */


	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_attitude_s data;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), sensor_sub_fd, &data);



				
			//matrix::Eulerf Eta=matrix::Quatf((double)raw.q[0],(double)raw.q[1],(double)raw.q[2],(double)raw.q[3]);
		
			float roll =	atan2f(2.0f * (data.q[0] * data.q[1] + data.q[2] * data.q[3]), 1.0f - 2.0f * (data.q[1] * data.q[1] + data.q[2] * data.q[2])); 
			float  pitch = asinf(2.0f * (data.q[0] * data.q[2] - data.q[3] * data.q[1])); 
    		float 	yaw = atan2f(2.0f * (data.q[0] * data.q[3] + data.q[1] * data.q[2]), 1.0f - 2.0f * (data.q[2] * data.q[2] + data.q[3] * data.q[3])); 



   			//float roll = atan2f(2.f * (raw.q[2]*raw.q[3] + raw.q[0]*raw.q[1]), raw.q[0]*raw.q[0] - raw.q[1]*raw.q[1] - raw.q[2]*raw.q[2] + raw.q[3]*raw.q[3]); 
			//float  pitch = asinf(2.f * (raw.q[0]*raw.q[2] - raw.q[1]*raw.q[3])); 
    		//float 	yaw = atan2f(2.f * (raw.q[1]*raw.q[2] + raw.q[0]*raw.q[3]), raw.q[0]*raw.q[0] + raw.q[1]*raw.q[1] - raw.q[2]*raw.q[2] - raw.q[3]*raw.q[3]); 



			//PX4_INFO("Eta: (%2.3f,%2.3f,%2.3f)",(double)Eta(0),(double)Eta(1),(double)Eta(2));

				PX4_INFO("roll:%8.4f pitch:%8.4f yaw:%8.4f",
					 (double)roll,
					 (double) pitch,
					(double)	yaw);
				

				/* set att and publish this information for other appspatam pp
				 the following does not have any meaning, it's just an example
				*/
				//att.q[0] = raw.accelerometer_m_s2[0];
				//att.q[1] = raw.accelerometer_m_s2[1];
				//att.q[2] = raw.accelerometer_m_s2[2];

				//orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
