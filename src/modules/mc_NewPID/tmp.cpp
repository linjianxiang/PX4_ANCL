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
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/img_moments.h>
#include <uORB/topics/img_point.h>
#include <uORB/topics/img_line.h>
#include <uORB/topics/vicon.h>
#include <uORB/topics/vehicle_image_attitude_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>

#include <systemlib/perf_counter.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

class MulticopterIBVS
{
public:
	/**
	* Constructor
	*/
	MulticopterIBVS();

	/**
	* Desctructor, also kills task
	*/
	~MulticopterIBVS();
	/**
	* Start task.
	* @return      OK on success
	*/
	int start();
	void status();
private:
	bool _task_should_exit;		/**< if true, task should exit */
	int _ibvs_task;             /**< task handle for task */

	/* Various subscription topic file descriptor */
	int _img_moments_sub;
	int _img_point_sub;
	int _img_line_sub;
	int _vicon_sub;
	int _params_sub;
	int _att_sub;

	orb_advert_t _ibvs_att_sp_pub;

	/* Various topics subscription variables definition */
	struct vicon_s _vicon;
	struct img_point_s _point;
	struct img_line_s _line;
	struct img_moments_s _moments;
	struct vehicle_image_attitude_setpoint_s _ibvs_att_sp;
	struct vehicle_attitude_s _att;

	control::BlockPID


	struct {
		float ibvs_kyaw;
		float ibvs_kx;
		float ibvs_ky;
		float ibvs_kz;
		float ibvs_kvx;
		float ibvs_kvy;
		float ibvs_kvz;
		float ibvs_kix;
		float ibvs_kiy;
		float ibvs_kiz;
		float thrust_int_sat;
		float p1_int_sat;
		float p2_int_sat;
		float thrust_g;
	}   _params;

	struct {
		float ibvs_kyaw;
		float ibvs_kx;
		float ibvs_ky;
		float ibvs_kvx;
		float ibvs_kvy;
		float ibvs_kz;
		float ibvs_kvz;
		float ibvs_kiz;
		float ibvs_kix;
		float ibvs_kiy;
		float thrust_int_sat;
		float p1_int_sat;
		float p2_int_sat;
		float thrust_g;
	}   _params_handles;

	/**
	* Shim for calling task_main from task_create.
	*/
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	* Main sensor collection task.
	*/
	void		task_main() __attribute__((noreturn));

	/**
	* Check for changes in subscribed topics.
	*/
	void		poll_subscriptions();

	/**
	* Update our local parameter cache.
	*/
	int			parameters_update(bool force);

	perf_counter_t	_loop_perf;
	hrt_abstime t_prev;

};

MulticopterIBVS::MulticopterIBVS():
	_task_should_exit(false),
	_ibvs_task(-1),
	_img_moments_sub(-1),
	_img_point_sub(-1),
	_img_line_sub(-1),
	_vicon_sub(-1),
	_params_sub(-1),
	_att_sub(-1),
	_ibvs_att_sp_pub(nullptr)
{

	memset(&_vicon, 0, sizeof(_vicon));
	memset(&_point, 0, sizeof(_point));
	memset(&_line, 0, sizeof(_line));
	memset(&_moments, 0, sizeof(_moments));
	memset(&_att,0,sizeof(_att));
	t_prev = 0;

	_loop_perf = perf_alloc(PC_INTERVAL, "mc_ibvs");
	t_prev=0;

	_params_handles.ibvs_kx = param_find("IBVS_KX");
	_params_handles.ibvs_ky = param_find("IBVS_KY");
	_params_handles.ibvs_kz = param_find("IBVS_KZ");
	_params_handles.ibvs_kvx = param_find("IBVS_KVX");
	_params_handles.ibvs_kvy = param_find("IBVS_KVY");
	_params_handles.ibvs_kvz = param_find("IBVS_KVZ");
	_params_handles.ibvs_kyaw = param_find("IBVS_KYAW");
	_params_handles.ibvs_kix = param_find("IBVS_KIX");
	_params_handles.ibvs_kiy = param_find("IBVS_KIY");
	_params_handles.ibvs_kiz = param_find("IBVS_KIZ");
	_params_handles.thrust_int_sat = param_find("IBVS_T_ISAT");
	_params_handles.p1_int_sat = param_find("IBVS_P1_ISAT");
	_params_handles.p2_int_sat = param_find("IBVS_P2_ISAT");
	_params_handles.thrust_g = param_find("IBVS_G");

	/* fetch initial parameter values */
	parameters_update(true);

}

MulticopterIBVS::~MulticopterIBVS(){

    if(_ibvs_task !=-1){
        _task_should_exit = true;
        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do{
            usleep(20000);
            if(++i > 50){
                task_delete(_ibvs_task);
                break;
            }
        }while(_ibvs_task !=-1);
    }
    ibvs::control = nullptr;
}

int
MulticopterIBVS::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;
    orb_check(_params_sub, &updated);

    if(updated)
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);

    if (updated || force){
	/* update C++ param system */
	updateParams();
	param_get(_params_handles.ibvs_kx, &(_params.ibvs_kx));
	param_get(_params_handles.ibvs_ky, &(_params.ibvs_ky));
	param_get(_params_handles.ibvs_kz, &(_params.ibvs_kz));
	param_get(_params_handles.ibvs_kvx, &(_params.ibvs_kvx));
	param_get(_params_handles.ibvs_kvy, &(_params.ibvs_kvy));
	param_get(_params_handles.ibvs_kvz, &(_params.ibvs_kvz));
	param_get(_params_handles.ibvs_kyaw, &(_params.ibvs_kyaw));
	param_get(_params_handles.ibvs_kix, &(_params.ibvs_kix));
	param_get(_params_handles.ibvs_kiy, &(_params.ibvs_kiy));
	param_get(_params_handles.ibvs_kiz, &(_params.ibvs_kiz));
	param_get(_params_handles.thrust_int_sat, &(_params.thrust_int_sat));
	param_get(_params_handles.p1_int_sat, &(_params.p1_int_sat));
	param_get(_params_handles.p2_int_sat, &(_params.p2_int_sat));
	param_get(_params_handles.thrust_g, &(_params.thrust_g));
	
	//constrain parameters?

    }
    return OK;
}


int
MulticopterIBVS::start()
{
    ASSERT(_ibvs_task == -1);

    //TODO check the priority
    /* start the task */
    _ibvs_task = task_spawn_cmd("mc_ibvs",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_POSITION_CONTROL,
                       1024,
                       (main_t)&MulticopterIBVS::task_main_trampoline,
                       nullptr);

    if (_ibvs_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
MulticopterIBVS::task_main_trampoline(int argc, char *argv[])
{
    ibvs::control->task_main();
}

void
MulticopterIBVS::poll_subscriptions()
{
	bool updated;

	orb_check(_img_moments_sub, &updated);
	if (updated){
		orb_copy(ORB_ID(img_moments), _img_moments_sub, &_moments);
	}

	orb_check(_img_point_sub, &updated);
	if (updated){
		orb_copy(ORB_ID(img_point), _img_point_sub, &_point);
	}

	orb_check(_img_line_sub, &updated);
	if (updated){
		orb_copy(ORB_ID(img_line), _img_line_sub, &_line);
	}

	orb_check(_vicon_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vicon), _vicon_sub, &_vicon);
	}

	orb_check(_att_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}
}

void
MulticopterIBVS::task_main()
{
	/* subscription */

	_img_moments_sub = orb_subscribe(ORB_ID(img_moments));
	_img_point_sub = orb_subscribe(ORB_ID(img_point));
	_img_line_sub = orb_subscribe(ORB_ID(img_line));
	_vicon_sub = orb_subscribe(ORB_ID(vicon));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));


	parameters_update(true);
	poll_subscriptions();

	struct pollfd fds[3];
	fds[0].fd = _img_moments_sub;
	fds[1].fd = _img_point_sub;
	fds[2].fd = _img_line_sub;
	for (int i=0;i<3;i++)
		fds[i].events = POLLIN;


	float ibvs_thrust_int = 0.0f;
	float ibvs_p12_int = 0.0f;
	float ibvs_p11_int = 0.0f;
	float dt = 0.0f;

	while(!_task_should_exit){
		/* wait for up to 500ms for data */
        	int poll_ret = poll(fds,(sizeof(fds) / sizeof(fds[0])),500);
		if (poll_ret <= 0) {
			/* time out - image att sp no longer valid */
			if (poll_ret ==0) warn("time out");
			else warn("poll error %d, %d", poll_ret, errno);
			_ibvs_att_sp.valid=false;
		} else {
			poll_subscriptions();
			parameters_update(false);
			bool mission_switch = (_vehicle_status.mission_switch==MISSION_SWITCH_MISSION);
			if(_global_pos.valid)
			{
			    _att_sp_ibvs.valid = 0;
			    float vxb, vyb, vzb;
			    vxb = _global_pos.x_dot*cos(_global_pos.yaw) - _global_pos.y_dot*sin(_global_pos.yaw);
			    vyb = _global_pos.x_dot*sin(_global_pos.yaw) + _global_pos.y_dot*cos(_global_pos.yaw);
			    vzb = _global_pos.z_dot;
			    if(isValid(&_img_feature,0)) //pitch theta
			    {
				float q11 = _img_feature.s[0];
				float Kp1, Kp2;
				float delta1, delta2;

				Kp2 = _params.ibvs_kx;
				Kp1 = Kp2/_params.ibvs_kvx;
				delta1 = q11;
				delta2 = vxb/Kp1 - delta1;

		//                if(mission_switch)
		//                    ibvs_p11_int += _params.ibvs_kix*q11*dt -_params.ibvs_kvx*_params.ibvs_kix/_params.ibvs_kx*vxb;
		//                else
		//                    ibvs_p11_int = 0;

				if(mission_switch)
				    ibvs_p11_int += _params.ibvs_kix*delta2*dt;
				else
				    ibvs_p11_int = 0;

				if(ibvs_p11_int>_params.p1_int_sat)
				    ibvs_p11_int = _params.p1_int_sat;
				else if(ibvs_p11_int<-_params.p1_int_sat)
				    ibvs_p11_int = -_params.p1_int_sat;
				else
				{}

		//                _att_sp_ibvs.pitch = -_params.ibvs_kx*q11 + _params.ibvs_kvx*vxb - ibvs_p11_int;
				_att_sp_ibvs.pitch = Kp2*delta2 + ibvs_p11_int;
		//                _att_sp_ibvs.pitch = -_params.ibvs_kx*q11 + _params.ibvs_kvx*vxb;
				_att_sp_ibvs.valid += 0|((uint8_t)2);
			    }
			    else
			    {
				_att_sp_ibvs.pitch = 0.0;
			    }


			    if(isValid(&_img_feature,1)) //roll phi
			    {
				float q12 = _img_feature.s[1];
				float Kr1, Kr2;
				float delta1, delta2;

				Kr2 = _params.ibvs_ky;
				Kr1 = Kr2/_params.ibvs_kvy;
				delta1 = q12;
				delta2 = vyb/Kr1 - delta1;


		//                if(mission_switch)
		//                    ibvs_p12_int += _params.ibvs_kiy*q12*dt -_params.ibvs_kvy*_params.ibvs_kiy/_params.ibvs_ky*vyb;
		//                else
		//                    ibvs_p12_int = 0;

				if(mission_switch)
				    ibvs_p12_int += _params.ibvs_kiy*delta2*dt;
				else
				    ibvs_p12_int = 0;

				if(ibvs_p12_int>_params.p2_int_sat)
				    ibvs_p12_int = _params.p2_int_sat;
				else if(ibvs_p12_int<-_params.p2_int_sat)
				    ibvs_p12_int = -_params.p2_int_sat;
				else
				{}

		//                _att_sp_ibvs.roll   = _params.ibvs_ky*q12 - _params.ibvs_kvy*vyb + ibvs_p12_int;
		//                _att_sp_ibvs.roll   = _params.ibvs_ky*q12 - _params.ibvs_kvy*vyb;
				_att_sp_ibvs.roll   = -Kr2*delta2 - ibvs_p12_int;
				_att_sp_ibvs.valid += 0|((uint8_t)1);

			    }
			    else
			    {
				_att_sp_ibvs.roll = 0.0;
			    }

			    if(isValid(&_img_feature,2))
			    {
				float q13 = _img_feature.s[2] - 1.0;

				float Kh1, Kh2;
				float delta1, delta2;

				Kh2 = _params.ibvs_kz;
				Kh1 = Kh2/_params.ibvs_kvz;
				delta1 = q13;
				delta2 = vzb/Kh1 - delta1;

		//                if(mission_switch)
		//                    ibvs_thrust_int += _params.ibvs_kiz*q13*dt -_params.ibvs_kvz*_params.ibvs_kiz/_params.ibvs_kz*vzb;
		//                else
		//                    ibvs_thrust_int = 0.0f;

				if(mission_switch)
				    ibvs_thrust_int += _params.ibvs_kiz*delta2*dt;
				else
				    ibvs_thrust_int = 0.0f;

				if(ibvs_thrust_int>_params.thrust_int_sat)//0.04
				    ibvs_thrust_int = _params.thrust_int_sat;
				else if(ibvs_thrust_int<(0.0f-_params.thrust_int_sat))
				    ibvs_thrust_int = 0.0f-_params.thrust_int_sat;
				else
				{}
				_att_sp_ibvs.thrust = -_params.ibvs_kz*q13 + _params.ibvs_kvz*vzb - ibvs_thrust_int;
				_att_sp_ibvs.thrust = Kh2*delta2 + ibvs_thrust_int + _params.thrust_g;
				_att_sp_ibvs.valid += 0|((uint8_t)8);
			    }
			    else
			    {
				_att_sp_ibvs.thrust = 0.0;
			    }

			    if(isValid(&_img_feature,3))
			    {
				float alpha = _img_feature.s[3];
				if (alpha>0.4) alpha  = 0.4;
				if (alpha<-0.4) alpha = -0.4;
				_att_sp_ibvs.yaw = _att.yaw + _params.ibvs_kyaw*alpha;
				_att_sp_ibvs.valid += 0|((uint8_t)4);
			    }
			    else
			    {
				_att_sp_ibvs.yaw = 0;
			    }

		//            warnx("r p t:%2.3f, %2.3f, %2.3f",_att_sp_ibvs.roll, _att_sp_ibvs.pitch, _att_sp_ibvs.thrust);

			}
			else {
			    _att_sp_ibvs.roll = 0;
			    _att_sp_ibvs.pitch = 0;
			    _att_sp_ibvs.yaw = 0;
			    _att_sp_ibvs.thrust = 0;
			    _att_sp_ibvs.valid = 0;
			}

			if(_att_sp_ibvs_pub_fd>0){
			    orb_publish(ORB_ID(vehicle_image_attitude_setpoint),_att_sp_ibvs_pub_fd, &_att_sp_ibvs);
			}
			else{
			    orb_advertise(ORB_ID(vehicle_image_attitude_setpoint),&_att_sp_ibvs);
			}
			perf_count(_loop_perf);
			//usleep(10000);
		}

		if (_ibvs_att_sp_pub == nullptr)
			_ibvs_att_sp_pub = orb_advertise(ORB_ID(vehicle_image_attitude_setpoint), &_att_sp_ibvs);
		else
			orb_publish(ORB_ID(vehicle_image_attitude_setpoint),ibvs_att_sp_pub,&_att_sp_ibvs);
    }


    _ibvs_task = -1;
    _exit(0);

}

void MulticopterIBVSPTMOMENTS::status()
{
    warnx("IBVS Gain: IBVS_IZ_P %.4f",_params.ibvs_kz);
    warnx("centeroid features valid : %d", _img_feature.valid);
    warnx("\t s1 s2 s3 s4 s5: %2.3f %2.3f %2.3f %2.3f %2.3f",_img_feature.s[0], _img_feature.s[1], _img_feature.s[2], _img_feature.s[3], _img_feature.s[4]);
    warnx("Reference roll pitch yaw thrust: %8.4f %8.4f %8.4f %8.4f",_att_sp_ibvs.roll,_att_sp_ibvs.pitch,_att_sp_ibvs.yaw, _att_sp_ibvs.thrust);
}

/**
 * Multicopter IBVS control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_ibvs_ptmoments_main(int argc, char *argv[]);


int mc_ibvs_ptmoments_main(int argc, char *argv[])
{
    if (argc < 2)
            errx(1, "missing command usage: mc_ibvs_ptmoments {start|stop|status}");

    if (!strcmp(argv[1], "start")){

        if(ibvs::g_control != nullptr)
            errx(1, "already running");

        ibvs::g_control = new MulticopterIBVSPTMOMENTS;

        if (ibvs::g_control == nullptr)
            errx(1, "alloc failed");

        if (OK != ibvs::g_control->start()) {
                    delete ibvs::g_control;
                    ibvs::g_control = nullptr;
                    err(1, "start failed");
                }
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (ibvs::g_control == nullptr)
            warnx("not running");

        delete ibvs::g_control;
        ibvs::g_control = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (ibvs::g_control) {
		ibvs::g_control->status();
//            errx(0, "running");
        }
        else{
            warnx("not running");
        }
	exit(0);
    }

    warnx("unrecognized command");
    return 1;
}
