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
 * @file img_moments.cpp
 * Image Moments information.
 *
 * @author Kenny <kenny3@ualberta.ca>
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

#include <uORB/topics/img_moments.h>


/**
 *
 *
 * @ingroup apps
 */
extern "C" __EXPORT int img_moments_main(int argc, char *argv[]);


int img_moments_main(int argc, char *argv[])
{
        int sub=-1;
        struct img_moments_s data;
        memset(&data,0,sizeof(data));

        if (argc < 2) {
                warnx("usage: img_moments {status}");
                return 1;
        }

        if (!strcmp(argv[1], "status")) {
                sub = orb_subscribe(ORB_ID(img_moments));
                if (sub>0) {
                        PX4_INFO("Image moments:");
                        orb_copy(ORB_ID(img_moments), sub, &data);
                        PX4_INFO("Timestamp: (%d)",data.usec);
                        PX4_INFO("s1 of Objects: (%.2f)",(double)data.s[0]);
                        PX4_INFO("s2 of Objects: (%.2f)",(double)data.s[1]);
                        PX4_INFO("s3 of Objects: (%.2f)",(double)data.s[2]);
                        PX4_INFO("s4 of Objects: (%.2f)",(double)data.s[3]);
                } else {
                        PX4_INFO("Could not subscribe to img_moments topic");
                }
                sub = orb_unsubscribe(sub);
                return 0;
        }
        warnx("unrecognized command");
        return 1;
}
