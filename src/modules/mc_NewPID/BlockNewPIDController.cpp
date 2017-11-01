#include "BlockNewPIDController.hpp"

void BlockNewPIDController::update()
{
	// wait for an image feature, timeout every 500 ms
	int poll_ret = poll(_fds,(sizeof(_fds) / sizeof(_fds[0])),500);
	
	if (poll_ret <= 0) 
	{
		if (poll_ret == 0) warn("time out");
		else warn("poll error %d, %d", poll_ret, errno);
		_att_sp.get().valid=false;
	} 
	else {

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
			
			// check for new updates   //_param_update is a topic structure
			if (_param_update.updated()) 
			{ 
				updateParams(); 
			}

			// get new information from subscriptions
			updateSubscriptions();
					
			//Calculate Control
			//convert velocity to body frame


			/* wait for declaration
			math::Vector<3> _vel;
			_vel.zero();
			*/

			//math::Vector<3> vel_err = _vel_sp - _vel;
			//thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + thrust_int

			//_pos.get().vx

			_vel_sp[0]=_pposx.update(_pos_sp[0]-_pos.get().x);
			_vel_sp[1]=_pposy.update(_pos_sp[1]-_pos.get().y);
			_vel_sp[2]=_pposz.update(_pos_sp[2]-_pos.get().z);

			math::Vector<3> thrust_sp;

			thrust_sp(0) = _pidx.update(_vel_sp[0]-_pos.get().vx);
			thrust_sp(1) = _pidy.update(_vel_sp[1]-_pos.get().vy);
			thrust_sp(2) = _pidz.update(_vel_sp[2]-_pos.get().vz);
			float thrust_abs = thrust_sp.length();

			
			
			math::Vector<3> body_x;
			math::Vector<3> body_y;
			math::Vector<3> body_z;
			body_z = -thrust_sp / thrust_abs;
			math::Vector<3> y_C(-sinf(_pos.get().yaw), cosf(_pos.get().yaw), 0.0f);
			body_x = y_C % body_z;
			body_y = body_z % body_x;

			matrix::Dcmf R;
			R.identity();

			for (int i = 0; i < 3; i++) {
				R(i, 0) = body_x(i);
				R(i, 1) = body_y(i);
				R(i, 2) = body_z(i);
			}

			matrix::Eulerf euler = R;
			_att_sp.get().roll = euler(0);
			_att_sp.get().pitch = euler(1);
			_att_sp.get().yaw = euler(2);
			_att_sp.get().thrust =thrust_abs ;
			/*
			temp++;
			if(temp==30)
			{
				PX4_INFO("Attitude Setpoint:  roll: %8.4f  pitch: %8.4f  yaw: %8.4f  thrust: %8.4f",
				(double)_att_sp.get().roll,
				(double)_att_sp.get().pitch,
				(double)_att_sp.get().yaw,
				(double)_att_sp.get().thrust);
				PX4_INFO("Local Position:  x: %8.4f  y: %8.4f  z: %8.4f",
				(double)_pos.get().x,
				(double)_pos.get().y,
				(double)_pos.get().z);
				PX4_INFO("Local Position:  vx: %8.4f  vy: %8.4f  vz: %8.4f",
				(double)_pos.get().vx,
				(double)_pos.get().vy,
				(double)_pos.get().vz);
				temp=0;
			}
			_att_sp.get().valid = true;

			

			_att_sp.get().roll = _pidx.update(_vel_sp[0]-_pos.get().vx);
			_att_sp.get().pitch = _pidy.update(_vel_sp[1]-_pos.get().vy);

			_att_sp.get().yaw = _pos.get().yaw + _pyaw.update(_pos.get().yaw);

			float thrust_abs = thrust_sp.length();




			_att_sp.get().thrust = _pidz.update(_vel_sp[2]-_pos.get().vz);
			
			temp++;
			if(temp==30)
			{
				PX4_INFO("Attitude Setpoint:  roll: %8.4f  pitch: %8.4f  yaw: %8.4f  thrust: %8.4f",
				(double)_att_sp.get().roll,
				(double)_att_sp.get().pitch,
				(double)_att_sp.get().yaw,
				(double)_att_sp.get().thrust);
				PX4_INFO("Local Position:  x: %8.4f  y: %8.4f  z: %8.4f",
				(double)_pos.get().x,
				(double)_pos.get().y,
				(double)_pos.get().z);
				PX4_INFO("Local Position:  vx: %8.4f  vy: %8.4f  vz: %8.4f",
				(double)_pos.get().vx,
				(double)_pos.get().vy,
				(double)_pos.get().vz);
				temp=0;
			}*/

			/*
			float vx = _pos.get().vx*cosf(_pos.get().yaw)-_pos.get().vy*sinf(_pos.get().yaw);
			float vy = _pos.get().vx*sinf(_pos.get().yaw)+_pos.get().vy*cosf(_pos.get().yaw);
			float vz = _pos.get().vz;			 

			_att_sp.get().roll = _pix.update(_img_moments.get().s[0])-_dx.update(vx);
			_att_sp.get().pitch = _piy.update(_img_moments.get().s[1])-_dy.update(vy);

			_att_sp.get().yaw = _pos.get().yaw + _pyaw.update(_pos.get().yaw);

			_att_sp.get().thrust = _t_sat.update(_piz.update(_img_moments.get().s[2]-1)-_dz.update(vz));
			_att_sp.get().valid = true;*/
			
			
		}
	}
	
	//update all publications
	updatePublications();

}
