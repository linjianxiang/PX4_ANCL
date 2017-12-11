#include "BlockINController.hpp"
//temporary
#include <uORB/topics/battery_status.h>

void BlockINController::update()
{
	// wait for an image feature, timeout every 500 ms
	int poll_ret = poll(_fds,(sizeof(_fds) / sizeof(_fds[0])),500);
	
	if (poll_ret <= 0) 
	{
		if (poll_ret == 0) warn("time out");
		else warn("poll error %d, %d", poll_ret, errno);

		

		//_att_sp.get().valid=false;
	} 
	else {

		uint64_t t1 = hrt_absolute_time();
		float dt = (t1 - _t) / 1.0e6f;
		_t=t1;
		//Set message timestamp

		_actuators.get().timestamp = t1;

		_att_sp.get().timestamp = t1;

		// check for sane values of dt
		if (dt>1.0f || dt<0) {
			warn("dt=%3.3f",(double)dt);
			//_att_sp.get().valid=false;
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


			param_get(_roll_rate_max, &_roll_rate_max_val);
			param_get(_pitch_rate_max, &_pitch_rate_max_val);
			param_get(_yaw_rate_max, &_yaw_rate_max_val);

			//rate_max[0]=_roll_rate_max_val;
			//rate_max[1]=_pitch_rate_max_val;
			//rate_max[0]=_roll_rate_max_val;
			//rate_max[1]=_pitch_rate_max_val;	



			float _thrust_sp = _v_att_sp.get().thrust;
			
			matrix::Eulerf Eta=matrix::Quatf((double)_vicon.get().q[0],(double)_vicon.get().q[1],(double)_vicon.get().q[2],(double)_vicon.get().q[3]);
			


			// In this way we do not use NewPID module.
			_rates_sp[0] = _pposx.update(_v_att_sp.get().roll_body-Eta(0));
			_rates_sp[1] = _pposy.update(_v_att_sp.get().pitch_body-Eta(1));
			_rates_sp[2] = _pposz.update(_v_att_sp.get().yaw_body-Eta(2));

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
			_actuators.get().control[7] = _v_att_sp.get().landing_gear;		
			//_actuators.get().timestamp = hrt_absolute_time();
			_actuators.get().timestamp_sample = _ctrl_state.get().timestamp;


			//  _battery_status 
			// int _battery_status_sub = orb_subscribe(ORB_ID(battery_status));
			// bool updated;
			// orb_check(_battery_status_sub, &updated);

			// struct battery_status_s	_battery_status;
			// if (updated) {
			// 	orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
			// }
			// if ( _battery_status.scale > 0.0f) 
			// {
			// 	for (int i = 0; i < 4; i++) {
			// 		_actuators.get().control[i] *= _battery_status.scale;
			// 	}
			// }

			//large angle limitation
			
			//FIXME:
			// if(_actuators.get().control[2]>0.2f)
			// 	_actuators.get().control[2]=0.2f;
			// else if(_actuators.get().control[2]<-0.2f)
			// 	_actuators.get().control[2]=-0.2f;
			
			// for (int i = 0; i < 2; i++) {
			// if(_actuators.get().control[i]>0.6f)
			// 	_actuators.get().control[i]=0.6f;
			// else if(_actuators.get().control[i]<-0.6f)
			// 	_actuators.get().control[i]=-0.6f;
			// }
			
			//test
			//_att_sp.get().roll= _actuators.get().control[0];
			//_att_sp.get().pitch= _actuators.get().control[1];
			//_att_sp.get().yaw=_actuators.get().control[2];
			//_att_sp.get().thrust=_actuators.get().control[3];

			
			//PX4_INFO("sec:%8.4f",(double)_actuators.get().control[3]);

			//for publishment
			struct actuator_controls_s	_temp;			/**< actuator controls */
			memset(&_temp, 0, sizeof(_temp));
			_temp.control[0]= _actuators.get().control[0];
			_temp.control[1]=_actuators.get().control[1];
			_temp.control[2]=_actuators.get().control[2];
			_temp.control[3]=_actuators.get().control[3];
			_temp.timestamp=_actuators.get().timestamp;
			
			//final publishment
			if (_actuators_0_pub != nullptr) {
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_temp);
			}
			else{
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_temp);
			}
		
		}
	}
	
	//update all publications
	updatePublications();

}
