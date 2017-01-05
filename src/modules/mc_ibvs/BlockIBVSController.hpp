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
	uint64_t _t;
};
