#include "BlockIBVSController.hpp"

void BlockIBVSController::update()
{
	// wait for an image feature, timeout every 500 ms
	int poll_ret = poll(_fds,(sizeof(_fds) / sizeof(_fds[0])),500);
	if (poll_ret <= 0) {
		if (poll_ret == 0) warn("time out");
		else warn("poll error %d, %d", poll_ret, errno);
		_att_sp.get().valid=false;
	} else {

		uint64_t t1 = hrt_absolute_time();
		float dt = (t1 - _t) / 1.0e6f;
		_t=t1;
		// check for sane values of dt
		if (dt>1.0f || dt<0) {
			warn("dt=%3.3f",(double)dt);
			_att_sp.get().valid=false;
		} else {
			// set dt for all child blocks
			setDt(dt);
			
			// check for new updates
			if (_param_update.updated()) { updateParams(); }

			// get new information from subscriptions
			updateSubscriptions();

			//Calculate Control
			//convert velocity to body frame
			float vx = _pos.get().vx*cosf(_pos.get().yaw)-_pos.get().vy*sinf(_pos.get().yaw);
			float vy = _pos.get().vx*sinf(_pos.get().yaw)+_pos.get().vy*cosf(_pos.get().yaw);
                        float vz = _pos.get().vz;
                        float fz = _kh2*(_piz.update(_img_moments.get().s[2]-1)-_dz.update(vz)/_kh1)+_mass*_grav;

                        _att_sp.get().roll   = -(_mass/fz)*_kl2*(_pix.update(_img_moments.get().s[1])-_dy.update(vy)/_kl1);
                        _att_sp.get().pitch  = (_mass/fz)*_kl2*(_piy.update(_img_moments.get().s[0])-_dx.update(vx)/_kl1);
                        //_att_sp.get().yaw = _pos.get().yaw + _pyaw.update(_pos.get().yaw);
                        _att_sp.get().yaw    = _pyaw.update(_img_moments.get().s[3]+(_kpsi)*_img_moments.get().s[3]);
                        _att_sp.get().thrust = _t_sat.update((float)0.5-fz);
                        _att_sp.get().valid  = true;
                        _att_sp.get().timestamp = t1;

			
		}
	}
	
	//update all publications
	updatePublications();

}
