#pragma once

#include <px4_posix.h>
#include <controllib/uorb/blocks.hpp>
#include <math.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
using namespace std;


using namespace control;

class BlockTASKController : public control::BlockTASKLoop
{
public:
		BlockTASKController() :
		BlockTASKLoop(NULL,"TASK"),
		_pposx(this,"POSX_P"),
		_pposy(this,"POSY_P"),
		_pposz(this,"POSZ_P"),

		_pidx(this,"PID_X"),
		_pidy(this,"PID_Y"),
		_pidz(this,"PID_Z"),

		_p_rate_raw(this,"RATE_RAW_P"),
		_p_rate_pitch(this,"RATE_PIT_P"),
		_p_rate_yaw(this,"RATE_YAW_P"),

		_pid_raw(this,"A_R"),
		_pid_pitch(this,"A_P"),
		_pid_yaw(this,"A_Y"),




		//_pyaw(this,"YAW_P"),

		_fds(),
		_t(0),
		_pos_sp(),
		_vel_sp()

	{
		_fds[0].fd =_pos.getHandle();
		
		_fds[0].events = POLLIN;

		//OUT
		_pos_sp.push_back(0);
		_pos_sp.push_back(0);
		_pos_sp.push_back(-1);

		_vel_sp.push_back(0);
		_vel_sp.push_back(0);
		_vel_sp.push_back(0);
		
		
		_gravity_val=0.0;
		_yaw_const_val=0.0;
		
		_gravity	= param_find("TASK_G");
		_yaw_const	= param_find("TASK_YAW_CONST");

		//IN
		_rates_sp.push_back(0);
	 	_rates_sp.push_back(0);
	  	_rates_sp.push_back(0);
		
		_final_control.push_back(0);
		_final_control.push_back(0);
		_final_control.push_back(0);

		//for publishing
		_actuators_0_pub=nullptr;

		_roll_rate_max=param_find("TASK_RRATE_MAX");
		_pitch_rate_max=param_find("TASK_PRATE_MAX");
		_yaw_rate_max=param_find("TASK_YRATE_MAX");

		//initial
		rate_max.push_back(0.0);
		rate_max.push_back(0.0);
		rate_max.push_back(0.0);
	}
	void update();
private:

	//OUT
	BlockP _pposx; 
	BlockP _pposy; 
	BlockP _pposz; 


	BlockPID _pidx;
	BlockPID _pidy;
	BlockPID _pidz;
	//BlockP _pyaw;

	//IN
	BlockP _p_rate_raw; 
	BlockP _p_rate_pitch; 
	BlockP _p_rate_yaw; 


	BlockPID _pid_raw;
	BlockPID _pid_pitch;
	BlockPID _pid_yaw;





	px4_pollfd_struct_t _fds[1];
	uint64_t _t;

	//OUT
	vector<float> _pos_sp;
	vector<float> _vel_sp;

    param_t _gravity;
	float _gravity_val;
	param_t _yaw_const;
	float _yaw_const_val;

	//IN
	vector<float> _rates_sp;
	vector<float> _final_control;

	orb_advert_t _actuators_0_pub;

	param_t _roll_rate_max;
	param_t _pitch_rate_max;
	param_t _yaw_rate_max;
	
	float _roll_rate_max_val;
	float _pitch_rate_max_val;
	float _yaw_rate_max_val;
	vector<float> rate_max;


};
