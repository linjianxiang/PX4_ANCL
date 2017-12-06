#pragma once

#include <px4_posix.h>
#include <controllib/uorb/blocks.hpp>
#include <math.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
using namespace std;


using namespace control;

class BlockINController : public control::BlockINLoop
{
public:
	BlockINController() :
		BlockINLoop(NULL,"IN"),
		//_pix(this,"IFX"),
		//_piy(this,"IFY"),
		//_piz(this,"IFZ"),

		_pposx(this,"POSX_P"),
		_pposy(this,"POSY_P"),
		_pposz(this,"POSZ_P"),

		_pidx(this,"PID_X"),
		_pidy(this,"PID_Y"),
		_pidz(this,"PID_Z"),

		_pyaw(this,"YAW_P"),

		_fds(),
		_t(0)
	
	
		//_gravity(this,"G")
		//_yawconst(this,"YAWCONST")
	{
		_fds[0].fd =_v_att_sp.getHandle();
		
		_fds[0].events = POLLIN;


		_rates_sp.push_back(0);
	 	_rates_sp.push_back(0);
	  	_rates_sp.push_back(0);
		
		_final_control.push_back(0);
		_final_control.push_back(0);
		_final_control.push_back(0);

		  _gravity_val=0.0;
		_yaw_const_val=0.0;
		
		_gravity	= param_find("IN_G");
		_yaw_const	= param_find("IN_YAW_CONST");

		//_vel_att_sp_sub=-1;
		/*
		_fds[0].fd = _img_moments.getHandle();
		_fds[1].fd = _img_point.getHandle();
		_fds[2].fd = _img_line.getHandle();
		for (int i=0;i<3;i++)
			_fds[i].events = POLLIN;*/
	}
	void update();
private:

	BlockP _pposx; 
	BlockP _pposy; 
	BlockP _pposz; 


	BlockPID _pidx;
	BlockPID _pidy;
	BlockPID _pidz;

	//BlockPID _pidt;
	//BlockPI _pix;
	//BlockPI _piy;
	//BlockPI _piz;
	//BlockP _dx;
	//BlockP _dy;
	//BlockP _dz;

	BlockP _pyaw;
	//BlockLimit _t_sat;

	px4_pollfd_struct_t _fds[1];
	//px4_pollfd_struct_t _fds[3];
	uint64_t _t;


	//vector<float> _att_sp;
	vector<float> _rates_sp;
	vector<float> _final_control;

    param_t _gravity;
	float _gravity_val;
	param_t _yaw_const;
	float _yaw_const_val;

	//int _vel_att_sp_sub;			/**< parameter updates subscription */
};
