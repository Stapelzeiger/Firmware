/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include "vtol_att_control_main.h"
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>

#include <float.h>
#include <cmath>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;
	_pusher_active = false;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;

	_params_handles_standard.pusher_ramp_dt = param_find("VT_PSHER_RMP_DT");
	_params_handles_standard.back_trans_ramp = param_find("VT_B_TRANS_RAMP");
	_params_handles_standard.down_pitch_max = param_find("VT_DWN_PITCH_MAX");
	_params_handles_standard.forward_thrust_scale = param_find("VT_FWD_THRUST_SC");
	_params_handles_standard.pitch_setpoint_offset = param_find("FW_PSP_OFF");
	_params_handles_standard.reverse_output = param_find("VT_B_REV_OUT");
	_params_handles_standard.reverse_delay = param_find("VT_B_REV_DEL");

	/* advertise debug value */
	struct debug_key_value_s dbg;
	strncpy(dbg.key, "test", 10);
	dbg.value = 10;
	_pub_dbg_val = orb_advertise(ORB_ID(debug_key_value), &dbg);
	struct debug_vect_s dbgv;
	strncpy(dbgv.name, "testv", 10);
	dbgv.x = 1;
	dbgv.y = 2;
	dbgv.z = 3;
	_pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbgv);
}

void
Standard::parameters_update()
{
	float v;

	/* duration of a forwards transition to fw mode */
	param_get(_params_handles_standard.pusher_ramp_dt, &v);
	_params_standard.pusher_ramp_dt = math::constrain(v, 0.0f, 20.0f);

	/* MC ramp up during back transition to mc mode */
	param_get(_params_handles_standard.back_trans_ramp, &v);
	_params_standard.back_trans_ramp = math::constrain(v, 0.0f, _params->back_trans_duration);

	_airspeed_trans_blend_margin = _params->transition_airspeed - _params->airspeed_blend;

	/* maximum down pitch allowed */
	param_get(_params_handles_standard.down_pitch_max, &v);
	_params_standard.down_pitch_max = math::radians(v);

	/* scale for fixed wing thrust used for forward acceleration in multirotor mode */
	param_get(_params_handles_standard.forward_thrust_scale, &_params_standard.forward_thrust_scale);

	/* pitch setpoint offset */
	param_get(_params_handles_standard.pitch_setpoint_offset, &v);
	_params_standard.pitch_setpoint_offset = math::radians(v);

	/* reverse output */
	param_get(_params_handles_standard.reverse_output, &v);
	_params_standard.reverse_output = math::constrain(v, 0.0f, 1.0f);

	/* reverse output */
	param_get(_params_handles_standard.reverse_delay, &v);
	_params_standard.reverse_delay = math::constrain(v, 0.0f, 10.0f);

}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// in mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// transition to mc mode
			if (_vtol_vehicle_status->vtol_transition_failsafe == true) {
				// Failsafe event, engage mc motors immediately
				_vtol_schedule.flight_mode = MC_MODE;
				_pusher_throttle = 0.0f;
				_reverse_output = 0.0f;


			} else {
				// Regular backtransition
				_vtol_schedule.flight_mode = TRANSITION_TO_MC;
				_vtol_schedule.transition_start = hrt_absolute_time();
				_reverse_output = _params_standard.reverse_output;

			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;


		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed or forward velocity drops below MPC cruise speed

			const Dcmf R_to_body(Quatf(_v_att->q).inversed());
			const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);

			float x_vel = vel(0);

			if (time_since_trans_start > _params->back_trans_duration ||
			    (_local_pos->v_xy_valid && x_vel <= _params->mpc_xy_cruise)) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE || _vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			_vtol_schedule.flight_mode = TRANSITION_TO_FW;
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// in fw mode
			_vtol_schedule.flight_mode = FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// continue the transition to fw mode while monitoring airspeed for a final switch to fw mode
			if (((_params->airspeed_disabled ||
			      _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed) &&
			     time_since_trans_start > _params->front_trans_time_min) ||
			    can_transition_on_ground()) {

				_vtol_schedule.flight_mode = FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}

		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case TRANSITION_TO_FW:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case TRANSITION_TO_MC:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	float mc_weight = 1.0f;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
		if (_params_standard.pusher_ramp_dt <= 0.0f) {
			// just set the final target throttle value
			_pusher_throttle = _params->front_trans_throttle;

		} else if (_pusher_throttle <= _params->front_trans_throttle) {
			// ramp up throttle to the target throttle value
			_pusher_throttle = _params->front_trans_throttle * time_since_trans_start / _params_standard.pusher_ramp_dt;
		}

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (_airspeed_trans_blend_margin > 0.0f &&
		    _airspeed->indicated_airspeed_m_s > 0.0f &&
		    _airspeed->indicated_airspeed_m_s >= _params->airspeed_blend &&
		    time_since_trans_start > _params->front_trans_time_min) {

			mc_weight = 1.0f - fabsf(_airspeed->indicated_airspeed_m_s - _params->airspeed_blend) /
				    _airspeed_trans_blend_margin;
			// time based blending when no airspeed sensor is set

		} else if (_params->airspeed_disabled) {
			mc_weight = 1.0f - time_since_trans_start / _params->front_trans_time_min;
			mc_weight = math::constrain(2.0f * mc_weight, 0.0f, 1.0f);

		}

		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset * (1.0f - mc_weight);
		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// check front transition timeout
		if (_params->front_trans_timeout > FLT_EPSILON) {
			if (time_since_trans_start > _params->front_trans_timeout) {
				// transition timeout occured, abort transition
				_attc->abort_front_transition("Transition timeout");
			}
		}

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {

		// maintain FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset;
		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		_pusher_throttle = 0.0f;

		if (time_since_trans_start >= _params_standard.reverse_delay) {
			// Handle throttle reversal for active breaking
			float thrscale = (time_since_trans_start - _params_standard.reverse_delay) / (_params_standard.pusher_ramp_dt);
			thrscale = math::constrain(thrscale, 0.0f, 1.0f);
			_pusher_throttle = thrscale * _params->back_trans_throttle;
		}

		// continually increase mc attitude control as we transition back to mc mode
		if (_params_standard.back_trans_ramp > FLT_EPSILON) {
			mc_weight = time_since_trans_start / _params_standard.back_trans_ramp;

		}

		// in back transition we need to start the MC motors again
		if (_motor_state != ENABLED) {
			_motor_state = set_motor_state(_motor_state, ENABLED);
		}
	}

	mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

static const float Sref = 0.2255f;
static const float rho = 1.225f;

static float CL(float alpha)
{
	const float CL0 = 0.5322f;
	const float CLalpha = 3.9859f;
	return (CL0 + CLalpha*alpha);
}
static float lift(float alpha, float V)
{
	return 0.5f*rho*V*V*Sref*CL(alpha);
}

static float drag(float alpha, float V)
{
	const float CD0 = 0.030f;//0.0150f;
	const float k_CL = 0.0757f;
	float CL_ = CL(alpha);
	float CD = CD0 + k_CL*CL_*CL_;
	return 0.5f*rho*V*V*Sref*CD;
}

static Quatf slerp(const Quatf &q0, const Quatf &q1, float t)
{
	// translated from Eigen
	// https://github.com/eigenteam/eigen-git-mirror/blob/6d062f0584523e3e282cf9f62ae260e0d961f3dc/Eigen/src/Geometry/Quaternion.h#L746
	const float eps = 1e-5f;
	const float one = 1.0f - eps;
	float d = q0.dot(q1);
	float absD = fabsf(d);

	float scale0;
	float scale1;

  	if(absD>=one)
  	{
    	scale0 = 1.0f - t;
    	scale1 = t;
  	}
  	else
  	{
    	// theta is the angle between the 2 quaternions
    	float theta = acosf(absD);
    	float sinTheta = sinf(theta);

    	scale0 = sin( ( 1.0f - t ) * theta) / sinTheta;
    	scale1 = sin( ( t * theta) ) / sinTheta;
  	}
  	if(d<0) scale1 = -scale1;

  	return scale0 * q0 + scale1 * q1;
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	// if the thrust scale param is zero,
	// then the pusher-for-pitch strategy is disabled and we can return
	if (_params_standard.forward_thrust_scale < FLT_EPSILON) {
		return;
	}

	// Do not engage pusher assist during a failsafe event
	// There could be a problem with the fixed wing drive
	if (_attc->get_vtol_vehicle_status()->vtol_transition_failsafe) {
		return;
	}

	// disable pusher assist during landing
	if (_attc->get_pos_sp_triplet()->current.valid
	    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		return;
	}

	const Dcmf R(Quatf(_v_att->q));
	const Dcmf R_sp(Quatf(_v_att_sp->q_d));
	const Eulerf euler(R);
	const Eulerf euler_sp(R_sp);
	_pusher_throttle = 0.0f;

	// direction of reference body z axis represented in earth frame
	const Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));
	// direction of the current body x axis in earth frame
	const Vector3f x_body_in_earth(R(0, 0), R(1, 0), R(2, 0));
	const Vector3f z_body_in_earth(R(0, 2), R(1, 2), R(2, 2));

	const float Vinf = fabsf(_airspeed->indicated_airspeed_m_s);

	const Vector3f airspeed_dir(x_body_in_earth(0), x_body_in_earth(1), 0); // xy projection of body x-axis
	const Vector3f airspeed_earth_frame = airspeed_dir.normalized()*Vinf;

	const float aoa_max = M_PI*8/180; // (0.8-0.5322)/3.9859/pi*180 = 3.8
	const float hover_throttle = _params->mpc_thr_hover;
	const float m = 1.7f;
	const float g = 9.81f;

	// reference force includes force to cancel gravity (in paper f_r does not include gravity)
	// (_v_att_sp->thrust_body[2] is negative)
	const Vector3f f_r = body_z_sp * _v_att_sp->thrust_body[2]; // reference force in earth frame

	// Low speed force allocation
	const Quatf att_sp_low_speed = _v_att_sp->q_d; // just use desired MC attitude

	// High speed force allocation
	const Vector3f x_wind_in_earth = airspeed_earth_frame.normalized();
	float f_r_parallel = x_wind_in_earth*f_r;
	const Vector3f f_r_parallel_v = f_r_parallel * x_wind_in_earth;
	const Vector3f f_r_perpendicular_v = f_r - f_r_parallel_v;
	const Vector3f y_wind_in_earth = (x_wind_in_earth.cross(f_r)).normalized();
	const Vector3f z_wind_in_earth = x_wind_in_earth.cross(y_wind_in_earth);

	// definition of scalar f_r_perpendicular is along z_w which is pointing away
	// from f_r, so f_r_perpendicular is always negative
	float f_r_perpendicular = -f_r_perpendicular_v.norm();
	float f_L_max_force = lift(aoa_max, Vinf);
	float f_L_max = hover_throttle * f_L_max_force/(m*g);
	float alpha_d;
	if (- f_r_perpendicular > f_L_max) {
		alpha_d = aoa_max; // limit wing to max lift
	} else {
		float f_L_zero_force = lift(0, Vinf);
		float f_L_zero = hover_throttle * f_L_zero_force/(m*g);
		alpha_d = (-f_r_perpendicular - f_L_zero)/(f_L_max-f_L_zero) * aoa_max; // linear interpolation
	}
	const Eulerf alpha_pitch_up(0, alpha_d, 0);
	const Dcmf body_to_wind(alpha_pitch_up);
	Dcmf wind_to_earth;
	wind_to_earth.setCol(0, x_wind_in_earth);
	wind_to_earth.setCol(1, y_wind_in_earth);
	wind_to_earth.setCol(2, z_wind_in_earth);
	Quatf att_sp_high_speed(wind_to_earth * body_to_wind); // body to earth quaternion
	if (airspeed_earth_frame.norm() < 0.1f || -f_r_perpendicular < 0.0001f) {
		att_sp_high_speed = _v_att->q; // set target to current attitude
	}

	// _params->airspeed_blend is the transition to prioritizing fixed wing lift
	// gamma [0...1] interpolates from low to high speed
	float gamma = 0.5f + 0.5f*tanhf(Vinf - _params->airspeed_blend);
	const Quatf att_sp = slerp(att_sp_low_speed, att_sp_high_speed, gamma);


	float aoa = atan2f(x_wind_in_earth * z_body_in_earth, x_wind_in_earth * x_body_in_earth);
	if (Vinf < 1 || isnan(aoa)) {
		aoa = 0;
	}
	// normalized lift/drag force (corresponds to a MC throttle point)
	float f_lift = hover_throttle * lift(aoa, Vinf)/(m*g);
	float f_drag = hover_throttle * drag(aoa, Vinf)/(m*g);
	// const Vector3f f_aero = - f_lift*z_wind_in_earth - f_drag*x_wind_in_earth;
	const Vector3f f_aero = (-f_lift*cosf(aoa) - f_drag*sinf(aoa))*z_body_in_earth
						  + (f_lift*sinf(aoa) - f_drag*cosf(aoa))*x_body_in_earth;
	const Vector3f f_th = f_r - f_aero;
	// thruster force in body z (negative)
	float fz = z_body_in_earth * f_th;
	// thruster force in body x (positive)
	float fx = x_body_in_earth * f_th;


	// _params_standard.forward_thrust_scale converts MC throttle to forward throttle
	// should be the ratio of max_lifter_force / max_thruster_force
	_pusher_throttle = fx * _params_standard.forward_thrust_scale;
	att_sp.copyTo(_v_att_sp->q_d);
	_v_att_sp->q_d_valid = true;
	_v_att_sp->thrust_body[0] = 0;
	_v_att_sp->thrust_body[1] = 0;
	_v_att_sp->thrust_body[2] = fz; // this is for multirotor thrust

	_pusher_throttle = _pusher_throttle < 0.0f ? 0.0f : _pusher_throttle;

	static int i = 0;
	i++;
	int i_mod = i%5;
	static struct debug_key_value_s dbg;
	if (i_mod == 0) {
		strncpy(dbg.key, "xx_gamma", 10);
		dbg.value = gamma;
	} else if (i_mod == 1) {
		strncpy(dbg.key, "xx_aoa", 10);
		dbg.value = aoa;
	} else if (i_mod == 2) {
		strncpy(dbg.key, "xx_fz", 10);
		dbg.value = fz;
	} else if (i_mod == 3) {
		strncpy(dbg.key, "xx_fx", 10);
		dbg.value = fx;
	} else if (i_mod == 4) {
		strncpy(dbg.key, "xx_alpha_d", 10);
		dbg.value = alpha_d;
	}
	orb_publish(ORB_ID(debug_key_value), _pub_dbg_val, &dbg);

	static struct debug_vect_s dbg_vect;
	static int j = 0;
	j++;
	int j_mod = j%3;
	if (j_mod == 0) {
		// dbg_vect.x = (float)math::signNoZero(att_sp_high_speed(0))*att_sp_high_speed.imag()(0);
		// dbg_vect.y = (float)math::signNoZero(att_sp_high_speed(0))*att_sp_high_speed.imag()(1);
		// dbg_vect.z = (float)math::signNoZero(att_sp_high_speed(0))*att_sp_high_speed.imag()(2);
		// strncpy(dbg_vect.name, "xx_att_hs", 10);

		dbg_vect.x = _airspeed->airspeed_body_x;
		dbg_vect.y = _airspeed->airspeed_body_y;
		dbg_vect.z = _airspeed->airspeed_body_z;
		strncpy(dbg_vect.name, "xx_3d_wind", 10);
	} else if (j_mod == 1) {
		// dbg_vect.x = f_aero(0);
		// dbg_vect.y = f_aero(1);
		// dbg_vect.z = f_aero(2);
		// strncpy(dbg_vect.name, "xx_f_aero", 10);
		dbg_vect.x = x_wind_in_earth(0);
		dbg_vect.y = x_wind_in_earth(1);
		dbg_vect.z = x_wind_in_earth(2);
		strncpy(dbg_vect.name, "xx_wind_x", 10);
	} else if (j_mod == 2) {
		// dbg_vect.x = z_body_in_earth(0);
		// dbg_vect.y = z_body_in_earth(1);
		// dbg_vect.z = z_body_in_earth(2);
		// strncpy(dbg_vect.name, "xx_body_z", 10);
		// Quatf body_to_wind_q(body_to_wind);
		// dbg_vect.x = (float)math::signNoZero(body_to_wind_q(0))*body_to_wind_q.imag()(0);
		// dbg_vect.y = (float)math::signNoZero(body_to_wind_q(0))*body_to_wind_q.imag()(1);
		// dbg_vect.z = (float)math::signNoZero(body_to_wind_q(0))*body_to_wind_q.imag()(2);
		Quatf wind_to_earth_q(wind_to_earth);
		dbg_vect.x = (float)math::signNoZero(wind_to_earth_q(0))*wind_to_earth_q.imag()(0);
		dbg_vect.y = (float)math::signNoZero(wind_to_earth_q(0))*wind_to_earth_q.imag()(1);
		dbg_vect.z = (float)math::signNoZero(wind_to_earth_q(0))*wind_to_earth_q.imag()(2);
		strncpy(dbg_vect.name, "xx_w2e", 10);
	}
	orb_publish(ORB_ID(debug_vect), _pub_dbg_vect, &dbg_vect);
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
 * Prepare message to acutators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	// multirotor controls
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	// roll
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	// pitch
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	// yaw
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;
	// throttle
	_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;


	// fixed wing controls
	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	if (_vtol_schedule.flight_mode != MC_MODE) {
		// roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];

		// pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
		// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];

		_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = _reverse_output;

	} else {

		if (_params->elevons_mc_lock) {
			// zero outputs when inactive
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = 0.0f;

		} else {
			// roll
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];

			// pitch
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];

			_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];

			_actuators_out_1->control[actuator_controls_s::INDEX_AIRBRAKES] = 0.0f;
		}
	}

	// set the fixed wing throttle control
	if (_vtol_schedule.flight_mode == FW_MODE) {

		// take the throttle value commanded by the fw controller
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

	} else {
		// otherwise we may be ramping up the throttle during the transition to fw mode
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
	}


}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};
