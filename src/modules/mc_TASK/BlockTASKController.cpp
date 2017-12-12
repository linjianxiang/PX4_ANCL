#include "BlockTASKController.hpp"
//temporary
#include <uORB/topics/battery_status.h>
void BlockTASKController::update()
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
		_actuators.get().timestamp = t1;
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
			
			// //New_PID
			
			// float x = _pos.get().x*cosf(_pos.get().yaw)+_pos.get().y*sinf(_pos.get().yaw);
			// float y = -_pos.get().x*sinf(_pos.get().yaw)+_pos.get().y*cosf(_pos.get().yaw);
			// float z = _pos.get().z;	

			// float vx = _pos.get().vx*cosf(_pos.get().yaw)+_pos.get().vy*sinf(_pos.get().yaw);
			// float vy = -_pos.get().vx*sinf(_pos.get().yaw)+_pos.get().vy*cosf(_pos.get().yaw);
			// float vz = _pos.get().vz;


			// _vel_sp[0]=_pposx.update(_pos_sp[0]-x);
			// _vel_sp[1]=_pposy.update(_pos_sp[1]-y);
			// _vel_sp[2]=_pposz.update(_pos_sp[2]-z);

			// _att_sp.get().roll = _pidy.update(_vel_sp[1]-vy);
			// _att_sp.get().pitch=-_pidx.update(_vel_sp[0]-vx);
			
			// _att_sp.get().yaw = _yaw_const_val;
			// float temp_thrust = _gravity_val-_pidz.update(_vel_sp[2]-vz);
			
			// if(temp_thrust<0)
			//  	temp_thrust=0;


			// _att_sp.get().thrust=temp_thrust;
			

			// //IN
         	// param_get(_roll_rate_max, &_roll_rate_max_val);
			// param_get(_pitch_rate_max, &_pitch_rate_max_val);
			// param_get(_yaw_rate_max, &_yaw_rate_max_val);


			// //rate_max[0]=_roll_rate_max_val;
			// //rate_max[1]=_pitch_rate_max_val;
			// //rate_max[0]=_roll_rate_max_val;
			// //rate_max[1]=_pitch_rate_max_val;	


			// //TODO: _v_att_sp change to _att_sp
			// float _thrust_sp = _att_sp.get().thrust;
			
			// matrix::Eulerf Eta=matrix::Quatf((double)_vicon.get().q[0],(double)_vicon.get().q[1],(double)_vicon.get().q[2],(double)_vicon.get().q[3]);
			


			// // In this way we do not use NewPID module.
			// _rates_sp[0] = _pposx.update(_att_sp.get().roll-Eta(0));
			// _rates_sp[1] = _pposy.update(_att_sp.get().pitch-Eta(1));
			// _rates_sp[2] = _pposz.update(_att_sp.get().yaw-Eta(2));

			// // //small angle judge
			// // int flag=0;
			// // if(_v_att_sp.get().yaw_body-Eta(2)<0.7f && _v_att_sp.get().yaw_body-Eta(2)>-0.7f)
			// // {
			// // 	flag=1;
			// // }

			// // /* limit rates */
			// // for (int i = 0; i < 3; i++) {
			// // 	_rates_sp[i] = math::constrain(_rates_sp[i], -rate_max[i], rate_max[i]);
			// // }

			// _final_control[0]=_pidx.update(_rates_sp[0]-_ctrl_state.get().roll_rate);
			// _final_control[1]=_pidy.update(_rates_sp[1]-_ctrl_state.get().pitch_rate);
			// _final_control[2]=_pidz.update(_rates_sp[2]-_ctrl_state.get().yaw_rate);

			// //for logging
			// _actuators.get().control[0] = (PX4_ISFINITE(_final_control[0])) ?_final_control[0] : 0.0f;
			// _actuators.get().control[1] = (PX4_ISFINITE(_final_control[1])) ?_final_control[1] : 0.0f;
			// _actuators.get().control[2] = (PX4_ISFINITE(_final_control[2])) ? _final_control[2] : 0.0f;
			// _actuators.get().control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
			
			// // NEED UPDATE
			// //_actuators.get().control[7] = _v_att_sp.get().landing_gear;		
			// //_actuators.get().timestamp = hrt_absolute_time();
			// _actuators.get().timestamp_sample = _ctrl_state.get().timestamp;

			
			
			// struct actuator_controls_s	_temp;			/**< actuator controls */
			// memset(&_temp, 0, sizeof(_temp));
			// _temp.control[0]= _actuators.get().control[0];
			// _temp.control[1]=_actuators.get().control[1];
			// _temp.control[2]=_actuators.get().control[2];
			// _temp.control[3]=_actuators.get().control[3];
			// _temp.timestamp=_actuators.get().timestamp;
			
			// //final publishment
			// if (_actuators_0_pub != nullptr) {
			// 	orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_temp);
			// }
			// else{
			// 	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_temp);
			// }

			//TODO: test



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

			float roll = _pidy.update(_vel_sp[1]-vy);
			float pitch=-_pidx.update(_vel_sp[0]-vx);
			
			float yaw = _yaw_const_val;
			float temp_thrust = _gravity_val-_pidz.update(_vel_sp[2]-vz);
			
			if(temp_thrust<0)
			 	temp_thrust=0;


			float thrust=temp_thrust;
			

			// temporary show
			_att_sp.get().q[0]= roll;
			_att_sp.get().q[1]= pitch;
			_att_sp.get().q[2]= yaw;
			_att_sp.get().q[3]= thrust;



			//IN
         	param_get(_roll_rate_max, &_roll_rate_max_val);
			param_get(_pitch_rate_max, &_pitch_rate_max_val);
			param_get(_yaw_rate_max, &_yaw_rate_max_val);


			//rate_max[0]=_roll_rate_max_val;
			//rate_max[1]=_pitch_rate_max_val;
			//rate_max[0]=_roll_rate_max_val;
			//rate_max[1]=_pitch_rate_max_val;	


			float _thrust_sp = thrust;
			
			matrix::Eulerf Eta=matrix::Quatf((double)_vicon.get().q[0],(double)_vicon.get().q[1],(double)_vicon.get().q[2],(double)_vicon.get().q[3]);
			


			// In this way we do not use NewPID module.
			_rates_sp[0] = _pposx.update(roll-Eta(0));
			_rates_sp[1] = _pposy.update(pitch-Eta(1));
			_rates_sp[2] = _pposz.update(yaw-Eta(2));

			// //small angle judge
			// int flag=0;
			// if(_v_att_sp.get().yaw_body-Eta(2)<0.7f && _v_att_sp.get().yaw_body-Eta(2)>-0.7f)
			// {
			// 	flag=1;
			// }

			// /* limit rates */
			// for (int i = 0; i < 3; i++) {
			// 	_rates_sp[i] = math::constrain(_rates_sp[i], -rate_max[i], rate_max[i]);
			// }

			_final_control[0]=_pidx.update(_rates_sp[0]-_ctrl_state.get().roll_rate);
			_final_control[1]=_pidy.update(_rates_sp[1]-_ctrl_state.get().pitch_rate);
			_final_control[2]=_pidz.update(_rates_sp[2]-_ctrl_state.get().yaw_rate);







			//for logging
			_actuators.get().control[0] = (PX4_ISFINITE(_final_control[0])) ?_final_control[0] : 0.0f;
			_actuators.get().control[1] = (PX4_ISFINITE(_final_control[1])) ?_final_control[1] : 0.0f;
			_actuators.get().control[2] = (PX4_ISFINITE(_final_control[2])) ? _final_control[2] : 0.0f;
			_actuators.get().control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
			
			// NEED UPDATE
			//_actuators.get().control[7] = _v_att_sp.get().landing_gear;		
			//_actuators.get().timestamp = hrt_absolute_time();
			_actuators.get().timestamp_sample = _ctrl_state.get().timestamp;

			
			if(_actuators.get().control[2]>0.2f)
				_actuators.get().control[2]=0.2f;
			else if(_actuators.get().control[2]<-0.2f)
				_actuators.get().control[2]=-0.2f;
			
			for (int i = 0; i < 2; i++) {
			if(_actuators.get().control[i]>0.6f)
				_actuators.get().control[i]=0.6f;
			else if(_actuators.get().control[i]<-0.6f)
				_actuators.get().control[i]=-0.6f;
			}



			//FIXME: normal
			// struct actuator_controls_s	_temp;			/**< actuator controls */
			// memset(&_temp, 0, sizeof(_temp));
			// _temp.control[0]= _actuators.get().control[0];
			// _temp.control[1]=_actuators.get().control[1];
			// _temp.control[2]=_actuators.get().control[2];
			// _temp.control[3]=_actuators.get().control[3];
			// _temp.timestamp=_actuators.get().timestamp;
			
			// //final publishment
			// if (_actuators_0_pub != nullptr) {
			// 	orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_temp);
			// }
			// else{
			// 	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_temp);
			// }

			//FIXME: show
			_att_sp.get().roll=_actuators.get().control[0];
			_att_sp.get().pitch=_actuators.get().control[1];
			_att_sp.get().yaw=_actuators.get().control[2];
			_att_sp.get().thrust=_actuators.get().control[3];
		}
	}
	
	//update all publications
	updatePublications();

}
