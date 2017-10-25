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

                //Set message timestamp
                _att_sp.get().timestamp = t1;

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

                        if (_img_moments.get().valid == (int)0 ) {
                            //Only non-zero values are valid
                            _att_sp.get().valid=false;

                        } else {

			//Calculate Control
			//convert velocity to body frame
			float vx = _pos.get().vx*cosf(_pos.get().yaw)-_pos.get().vy*sinf(_pos.get().yaw);
			float vy = _pos.get().vx*sinf(_pos.get().yaw)+_pos.get().vy*cosf(_pos.get().yaw);
                        float vz = _pos.get().vz;
                        float fz = _kh2*(_img_moments.get().s[2]-1-(vz/_kh1));

                        //Compute roll and check if saturation is necessary
                        float phi_sp = _kl2_roll*(_img_moments.get().s[1]-vy/_kl1_roll);
//                        if(phi_sp > _phi_max){
//                            phi_sp = _phi_max;
//                        } else if (phi_sp < -_phi_max){
//                            phi_sp = -_phi_max;
//                        }
                        _att_sp.get().roll   = phi_sp;

                        //Compute the pitch and saturate if necessary
                        float theta_sp = -_kl2_pitch*(_img_moments.get().s[0]-vx/_kl1_pitch);
//                        if(theta_sp > _theta_max){
//                            theta_sp = _theta_max;
//                        } else if (phi_sp < -_theta_max){
//                            theta_sp = -_theta_max;
//                        }
                        _att_sp.get().pitch  = theta_sp;

                        float moment_s3 = _img_moments.get().s[3];
                        if( moment_s3 > _yaw_max){
                            moment_s3 = _yaw_max;
                        } else if (moment_s3 < -_yaw_max){
                            moment_s3 = -_yaw_max;
                        }
                        float yaw_sp = _pos.get().yaw+(_kpsi)*moment_s3;
                        _att_sp.get().yaw    = yaw_sp;

                        //Thrust is 1 if at desired altitude + _thrust_g needed for hover
                        _att_sp.get().thrust = _t_sat.update(-fz+_thrust_g);
                        _att_sp.get().valid  = true;

                        }
		}
	}
	
	//update all publications
	updatePublications();

}
