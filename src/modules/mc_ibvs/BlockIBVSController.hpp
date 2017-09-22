#pragma once

#include <px4_posix.h>
#include <controllib/uorb/blocks.hpp>

using namespace control;

class BlockIBVSController : public control::BlockIBVSOuterLoop
{
public:
	BlockIBVSController() :
		BlockIBVSOuterLoop(NULL,"IBVS"),
		_pix(this,"IFX"),
		_piy(this,"IFY"),
		_piz(this,"IFZ"),
		_dx(this,"VX_P"),
		_dy(this,"VY_P"),
		_dz(this,"VZ_P"),
		_pyaw(this,"YAW_P"),
		_t_sat(this,"T"),
		_fds(),
		_t(0) 
	{
		_kl1       = param_find("IBVS_KL1");
		_kl2       = param_find("IBVS_KL2");
		_kh1       = param_find("IBVS_KH1");
		_kh2       = param_find("IBVS_KH2");
		_kpsi      = param_find("IBVS_KPSI");
		_j3        = param_find("IBVS_J3");
		_fds[0].fd = _img_moments.getHandle();
		_fds[1].fd = _img_point.getHandle();
		_fds[2].fd = _img_line.getHandle();
		for (int i=0;i<3;i++)
			_fds[i].events = POLLIN;
	}
	void update();
private:
	BlockPI _pix;
	BlockPI _piy;
	BlockPI _piz;
	BlockP _dx;
	BlockP _dy;
	BlockP _dz;
	BlockP _pyaw;
	BlockLimit _t_sat;
	px4_pollfd_struct_t _fds[3];
	float _kl1;
	float _kl2;
	float _kh1;
	float _kh2;
	float _kpsi;
	float _j3;
	uint64_t _t;
};
