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
		//Set message timestamp
		_att_sp.get().timestamp = t1;
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
			param_get(_gravity, &_gravity_val);
			param_get(_yaw_const, &_yaw_const_val);
			//single pid on pos
			/*
				float x = _pos.get().x*cosf(Eta(2))+_pos.get().y*sinf(Eta(2));
				float y = -_pos.get().x*sinf(Eta(2))+_pos.get().y*cosf(Eta(2));
				float z = _pos.get().z;	

				_att_sp.get().roll = _pidy.update(_pos_sp[1]-y);
				_att_sp.get().pitch=-_pidx.update(_pos_sp[0]-x);
				
				_att_sp.get().yaw = Eta(2);
				float temp_thrust = float(0.05)-float(0.8)*_pidz.update(_pos_sp[2]-z);
				if(temp_thrust<0)
					temp_thrust=0;

				_att_sp.get().thrust=temp_thrust;
			*/
			
			
			//matrix::Eulerf Eta=matrix::Quatf((double)_vicon.get().q[0],(double)_vicon.get().q[1],(double)_vicon.get().q[2],(double)_vicon.get().q[3]);
		
			//PX4_INFO("Eta: (%2.3f,%2.3f,%2.3f)",(double)Eta(0),(double)Eta(1),(double)Eta(2));
		

			//New_PID
			
			float x = _pos.get().x*cosf(_pos.get().yaw)+_pos.get().y*sinf(_pos.get().yaw);
			float y = -_pos.get().x*sinf(_pos.get().yaw)+_pos.get().y*cosf(_pos.get().yaw);
			float z = _pos.get().z;	

			float vx = _pos.get().vx*cosf(_pos.get().yaw)+_pos.get().vy*sinf(_pos.get().yaw);
			float vy = -_pos.get().vx*sinf(_pos.get().yaw)+_pos.get().vy*cosf(_pos.get().yaw);
			float vz = _pos.get().vz;


			_vel_sp[0]=_pposx.update(_pos_sp[0]-x);
			_vel_sp[1]=_pposy.update(_pos_sp[1]-y);
			_vel_sp[2]=_pposz.update(_pos_sp[2]-z);

			_att_sp.get().roll = _pidy.update(_vel_sp[1]-vy);
			_att_sp.get().pitch=-_pidx.update(_vel_sp[0]-vx);
			
			_att_sp.get().yaw = _yaw_const_val;
			float temp_thrust = _gravity_val-_pidz.update(_vel_sp[2]-vz);
			
			 if(temp_thrust<0)
			 	temp_thrust=0;


			 _att_sp.get().thrust=temp_thrust;
			

            /*
			//math::Vector<3> e3(0,0,1);
			float g=9.8;
			float m=1.6;


			//float psi=_pos.get().yaw;

			//math::Matrix<3, 3> R3psi(cosf(psi),-sinf(psi),0;cosf(psi),-sinf(psi),0)
			
	
			_pos_sp[0]=_pos_sp[0]-_pos.get().x;
			_pos_sp[1]=_pos_sp[1]-_pos.get().y;
			_pos_sp[2]=_pos_sp[2]-_pos.get().z;

		
			_vel_sp[0]=_pidx.update(_pos_sp[0]);
			_vel_sp[1]=_pidy.update(_pos_sp[1]);
			_vel_sp[2]=-m*g+_pidz.update(_pos_sp[2]);

			_att_sp.get().roll=-(_vel_sp[0]*cosf(_pos.get().yaw)+_vel_sp[1]*sinf(_pos.get().yaw))/m/g;
			_att_sp.get().pitch=(-_vel_sp[0]*sinf(_pos.get().yaw)+_vel_sp[1]*cosf(_pos.get().yaw))/m/g;
			
			_att_sp.get().yaw = _yaw_const_val;
			
			_att_sp.get().thrust=-_vel_sp[2];*/
			



/*
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
			math::Vector<3> y_C(-sinf(Eta(2)), cosf(Eta(2)), 0.0f);
			body_x = y_C % body_z;
			
			if (body_z(2) < 0.0f) 
			{
						body_x = -body_x;
			}
			body_x.normalize();
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
			_att_sp.get().yaw =  Eta(2);
			_att_sp.get().thrust =thrust_abs ;
*/


			/*
			float vx = _pos.get().vx*cosf(Eta(2))-_pos.get().vy*sinf(_pos.get().yaw);
			float vy = _pos.get().vx*sinf(Eta(2))+_pos.get().vy*cosf(Eta(2));
			float vz = _pos.get().vz;			 

			_att_sp.get().roll = _pix.update(_img_moments.get().s[0])-_dx.update(vx);
			_att_sp.get().pitch = _piy.update(_img_moments.get().s[1])-_dy.update(vy);

			_att_sp.get().yaw = Eta(2) + _pyaw.update(Eta(2));

			_att_sp.get().thrust = _t_sat.update(_piz.update(_img_moments.get().s[2]-1)-_dz.update(vz));
			_att_sp.get().valid = true;*/
			
			
		}
	}
	
	//update all publications
	updatePublications();

}
