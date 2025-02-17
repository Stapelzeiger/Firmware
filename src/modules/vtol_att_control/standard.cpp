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
#include <uORB/topics/sensor_combined.h>

#include <float.h>
#include <cmath>

using namespace matrix;


static const float Sref = 0.2231244f;
static const float rho = 1.18f;
// TODO calibration offset, currently zero body angle is 2deg measured aoa
static const float aoa_max = M_PI*8/180; // (0.8-0.5322)/3.9859/pi*180 = 3.8
static const float m = 1.7f;
static const float g = 9.81f;


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

	_params_handles_standard.gamma_cd0 = param_find("VT_ADAPT_G_CD0");
	_params_handles_standard.gamma_cd1 = param_find("VT_ADAPT_G_CD1");
	_params_handles_standard.gamma_cd2 = param_find("VT_ADAPT_G_CD2");
	_params_handles_standard.gamma_cl0 = param_find("VT_ADAPT_G_CL0");
	_params_handles_standard.gamma_cl1 = param_find("VT_ADAPT_G_CL1");
	_params_handles_standard.gamma_ctx = param_find("VT_ADAPT_G_CTX");
	_params_handles_standard.gamma_ctz = param_find("VT_ADAPT_G_CTZ");
	_params_handles_standard.gamma_ctdz = param_find("VT_ADAPT_G_CTDZ");
	_params_handles_standard.lambda_a = param_find("VT_ADAPT_L_A");
	_params_handles_standard.lambda_t = param_find("VT_ADAPT_L_T");
	_params_handles_standard.reset_cd0 = param_find("VT_RESET_CD0");
	_params_handles_standard.reset_cd1 = param_find("VT_RESET_CD1");
	_params_handles_standard.reset_cd2 = param_find("VT_RESET_CD2");
	_params_handles_standard.reset_cl0 = param_find("VT_RESET_CL0");
	_params_handles_standard.reset_cl1 = param_find("VT_RESET_CL1");
	_params_handles_standard.reset_ctx = param_find("VT_RESET_CTX");
	_params_handles_standard.reset_ctz = param_find("VT_RESET_CTZ");
	_params_handles_standard.reset_ctdz = param_find("VT_RESET_CTDZ");
	_params_handles_standard.theta_max_dev = param_find("VT_ADAPT_MAX_DEV");
	_params_handles_standard.acc_lp_fc = param_find("VT_ADAPT_A_LP_FC");
	_params_handles_standard.lambda_P = param_find("VT_ADAPT_L_P");
	_params_handles_standard.pusher_ramp_dt = param_find("VT_PSHER_RMP_DT");
	_params_handles_standard.back_trans_ramp = param_find("VT_B_TRANS_RAMP");
	_params_handles_standard.down_pitch_max = param_find("VT_DWN_PITCH_MAX");
	_params_handles_standard.forward_thrust_scale = param_find("VT_FWD_THRUST_SC");
	_params_handles_standard.pitch_setpoint_offset = param_find("FW_PSP_OFF");
	_params_handles_standard.reverse_output = param_find("VT_B_REV_OUT");
	_params_handles_standard.reverse_delay = param_find("VT_B_REV_DEL");

	parameters_update();

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

	// adaptive model
	prev_Phi_T.setZero();
	prev_Phi_A.setZero();
	Gamma_A_diag.setZero();
	Gamma_T_diag.setZero();

	float hover_th = 0.45f;
	float fw_th_scale = 2.0f;
	theta_T0(0) = (m*g)/(hover_th*hover_th * fw_th_scale*fw_th_scale); // CTx
	theta_T0(1) = (m*g)/(hover_th*hover_th); // CTz
	theta_T0(2) = 0.068f*2; // CTDz
	theta_T = theta_T0;
	theta_A0(0) = 0.1543f; // CD0
	theta_A0(1) = 0.178f; // CD1
	theta_A0(2) = 1.619f; // CD2
	theta_A0(3) = 0.3707f; // CL0
	theta_A0(4) = 3.2566f; // CL1
	theta_A = theta_A0;

	W.setZero();
	a_f.setZero();
	P.setZero();
	P(0, 0) = _params_standard.gamma_ctx;
	P(1, 1) = _params_standard.gamma_ctz;
	P(2, 2) = _params_standard.gamma_ctdz;
	P(3, 3) = _params_standard.gamma_cd0;
	P(4, 4) = _params_standard.gamma_cd1;
	P(5, 5) = _params_standard.gamma_cd2;
	P(6, 6) = _params_standard.gamma_cl0;
	P(7, 7) = _params_standard.gamma_cl1;
}

void
Standard::parameters_update()
{
	float v;

	/* adaptation gains */
	param_get(_params_handles_standard.gamma_cd0, &v);
	_params_standard.gamma_cd0 = v;
	Gamma_A_diag(0) = _params_standard.gamma_cd0;

	param_get(_params_handles_standard.gamma_cd1, &v);
	_params_standard.gamma_cd1 = v;
	Gamma_A_diag(1) = _params_standard.gamma_cd1;

	param_get(_params_handles_standard.gamma_cd2, &v);
	_params_standard.gamma_cd2 = v;
	Gamma_A_diag(2) = _params_standard.gamma_cd2;

	param_get(_params_handles_standard.gamma_cl0, &v);
	_params_standard.gamma_cl0 = v;
	Gamma_A_diag(3) = _params_standard.gamma_cl0;

	param_get(_params_handles_standard.gamma_cl1, &v);
	_params_standard.gamma_cl1 = v;
	Gamma_A_diag(4) = _params_standard.gamma_cl1;

	param_get(_params_handles_standard.gamma_ctx, &v);
	_params_standard.gamma_ctx = v;
	Gamma_T_diag(0) = _params_standard.gamma_ctx;

	param_get(_params_handles_standard.gamma_ctz, &v);
	_params_standard.gamma_ctz = v;
	Gamma_T_diag(1) = _params_standard.gamma_ctz;

	param_get(_params_handles_standard.gamma_ctdz, &v);
	_params_standard.gamma_ctdz = v;
	Gamma_T_diag(2) = _params_standard.gamma_ctdz;

	param_get(_params_handles_standard.lambda_a, &v);
	_params_standard.lambda_a = v;

	param_get(_params_handles_standard.lambda_t, &v);
	_params_standard.lambda_t = v;

	param_get(_params_handles_standard.theta_max_dev, &v);
	_params_standard.theta_max_dev = v;

	param_get(_params_handles_standard.acc_lp_fc, &v);
	_params_standard.acc_lp_fc = v;

	param_get(_params_handles_standard.lambda_P, &v);
	_params_standard.lambda_P = v;


	param_get(_params_handles_standard.reset_cd0, &v);
	_params_standard.reset_cd0 = v;

	param_get(_params_handles_standard.reset_cd1, &v);
	_params_standard.reset_cd1 = v;

	param_get(_params_handles_standard.reset_cd2, &v);
	_params_standard.reset_cd2 = v;

	param_get(_params_handles_standard.reset_cl0, &v);
	_params_standard.reset_cl0 = v;

	param_get(_params_handles_standard.reset_cl1, &v);
	_params_standard.reset_cl1 = v;

	param_get(_params_handles_standard.reset_ctx, &v);
	_params_standard.reset_ctx = v;

	param_get(_params_handles_standard.reset_ctz, &v);
	_params_standard.reset_ctz = v;

	param_get(_params_handles_standard.reset_ctdz, &v);
	_params_standard.reset_ctdz = v;



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


static float force_allocation_compute_desired_alpha(float v, float alpha_max, float lift, const Vector<float, 5> &theta_A)
{
	const float CL0 = theta_A(3);
	const float CLalpha = theta_A(4);
	const float q_Sref = 0.5f*rho*v*v*Sref;
	const float zero_aoa_lift = q_Sref * CL0;
	const float lift_slope = q_Sref * CLalpha;
	if (v < 1.0f) { // very low speed, avoid dividing by zero lift_slope
		return 0;
	}
	float alpha = (lift - zero_aoa_lift)/lift_slope;


	float delta_lift = 0.2f*m*g; // [N] minimum lift to be compensated by thrusters
	static bool prev_above_alpha_max = false; // hysteresis around alpha max condition
	float hysteresis_alpha = 0;
	if (prev_above_alpha_max) {
		hysteresis_alpha = 0.5f*delta_lift/lift_slope;
	}
	if (alpha > alpha_max - hysteresis_alpha) {
		prev_above_alpha_max = true;
		// alpha = alpha_max - delta_lift/lift_slope;
		alpha = alpha - delta_lift/lift_slope; // reduce by delta_lift to ensure thrusters have a minimum thrust
		if (alpha > alpha_max) {
			alpha = alpha_max;
		}
	} else {
		prev_above_alpha_max = false;
	}
	return alpha;
}

static void build_aero_model_phi(float alpha, float v, Matrix<float, 3, 5> &phi)
{
	// parameters theta_a: [CD0, CD1, CD2, CL0, CL1]
	float ca = cosf(alpha);
	float sa = sinf(alpha);
	float alpha2 = alpha*alpha;
	// fx
	phi(0,0) = -ca;
	phi(0,1) = -ca*alpha;
	phi(0,2) = -ca*alpha2;
	phi(0,3) = sa;
	phi(0,4) = sa*alpha;
	// fy
	phi(1,0) = 0.0f;
	phi(1,1) = 0.0f;
	phi(1,2) = 0.0f;
	phi(1,3) = 0.0f;
	phi(1,4) = 0.0f;
	// fz
	phi(2,0) = -sa;
	phi(2,1) = -sa*alpha;
	phi(2,2) = -sa*alpha2;
	phi(2,3) = -ca;
	phi(2,4) = -ca*alpha;
	phi = 0.5f*rho*v*v*Sref/m * phi;
}

static void build_thrust_model_phi(float th_signal_x_sq, float th_signal_z_sq, Vector3f v, float alpha, Matrix<float, 3, 3> &phi)
{
	// parameters theta_t: [CTx, CTz, CTDz]
	phi.setZero();
	phi(0,0) = th_signal_x_sq / m;
	phi(2,1) = -th_signal_z_sq / m;
	float v_xy_norm = sqrtf(v(0)*v(0) + v(1)*v(1)) + 0.001f;
	const float pi_half_sq = (M_PI/2)*(M_PI/2);
	float alpha_flipped = -alpha;
	float fs = powf(fabsf(th_signal_z_sq), 0.7075f) * powf(v.norm(), 0.585f) * (alpha_flipped*alpha_flipped - pi_half_sq) * (alpha_flipped - 3.126f);
	phi(0,2) = - v(0)/v_xy_norm * fs;
	phi(1,2) = - v(1)/v_xy_norm * fs;
}

static float sqrtf_signed(float in)
{
	if (in < 0.0f) {
		return -sqrtf(-in);
	} else {
		return sqrtf(in);
	}
}

static float sq(float x)
{
	return x*x;
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

static float limit(float val, float c, float k)
{
	float min = c/k;
	float max = c*k;
	if (val > max) {
		return max;
	}
	if (val < min) {
		return min;
	}
	return val;
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

	hrt_abstime now_us = hrt_absolute_time();
	float dt = (float)(now_us - prev_iteration_time)/1000000;
	prev_iteration_time = now_us;

	const Dcmf R(Quatf(_v_att->q)); // R_body->earth
	const Dcmf R_sp(Quatf(_v_att_sp->q_d));


	// Parameter Adaptation

	// Adapt thrust & aero model based on velocity error
	const Vector3f v_err(_v_att_sp->vel_err_x, _v_att_sp->vel_err_y, _v_att_sp->vel_err_z);
	const Vector<float, 5> theta_A_err = prev_Phi_A.transpose()*R.transpose()*v_err;
	const Vector<float, 3> theta_T_err = prev_Phi_T.transpose()*R.transpose()*v_err;
	static bool adaptation_running = false;
	if (_v_att_sp->vel_err_valid) {
		if (adaptation_running == false) {
			adaptation_running = true;
			// reset P, theta
			prev_Phi_T.setZero();
			prev_Phi_A.setZero();
			Gamma_A_diag.setZero();
			Gamma_T_diag.setZero();
			W.setZero();
			a_f.setZero();
			P.setZero();
			P(0, 0) = _params_standard.gamma_ctx;
			P(1, 1) = _params_standard.gamma_ctz;
			P(2, 2) = _params_standard.gamma_ctdz;
			P(3, 3) = _params_standard.gamma_cd0;
			P(4, 4) = _params_standard.gamma_cd1;
			P(5, 5) = _params_standard.gamma_cd2;
			P(6, 6) = _params_standard.gamma_cl0;
			P(7, 7) = _params_standard.gamma_cl1;
			theta_A(0) = _params_standard.reset_cd0;
			theta_A(1) = _params_standard.reset_cd1;
			theta_A(2) = _params_standard.reset_cd2;
			theta_A(3) = _params_standard.reset_cl0;
			theta_A(4) = _params_standard.reset_cl1;
			theta_T(0) = _params_standard.reset_ctx;
			theta_T(1) = _params_standard.reset_ctz;
			theta_T(2) = _params_standard.reset_ctdz;
		}
		const float lambda_A = _params_standard.lambda_a;
		const float lambda_T = _params_standard.lambda_t;

		// theta_A += dt * (Gamma_A_diag.emult(theta_A_err) - lambda_A*(theta_A - theta_A0));
		// theta_T += dt * (Gamma_T_diag.emult(theta_T_err) - lambda_T*(theta_T - theta_T0));
		theta_A += dt * (P.slice<5, 5>(3, 3)*theta_A_err - lambda_A*(theta_A - theta_A0));
		theta_T += dt * (P.slice<3, 3>(0, 0)*theta_T_err - lambda_T*(theta_T - theta_T0));
	} else {
		adaptation_running = false;
	}

	// Adapt thrust & aero model based on acceleration prediction error
	if (_sensor_combined_sub == -1) {
		_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	}
	bool acc_updated;
	orb_check(_sensor_combined_sub, &acc_updated);
	struct sensor_combined_s sensor_combined_msg;
	if (acc_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_combined_msg);
	}
	Vector3f e_acc;
	if (_v_att_sp->vel_err_valid && // this is to stop adaptation from running on the ground (= not in position mode)
		acc_updated && sensor_combined_msg.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
		Vector3f a(sensor_combined_msg.accelerometer_m_s2[0],
					sensor_combined_msg.accelerometer_m_s2[1],
					sensor_combined_msg.accelerometer_m_s2[2]);
		// https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
		// out = alpha * in + (1-alpha) * prev_out;
		float fc_W_fiter = _params_standard.acc_lp_fc;
		float alpha = (2*(float)M_PI*dt*fc_W_fiter)/(2*(float)M_PI*dt*fc_W_fiter+1);
		// filter acceleration
		a_f = alpha * a + (1-alpha) * a_f;
		// filter W
		W = (1-alpha)*W;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				W(i, j) += alpha * prev_Phi_T(i, j);
			}
			for (int j = 0; j < 5; j++) {
				W(i, j+3) += alpha * prev_Phi_A(i, j);
			}
		}
		// update P, theta
		Vector<float, 3+5> theta;
		theta.set<3, 1>(theta_T, 0, 0);
		theta.set<5, 1>(theta_A, 3, 0);
		e_acc = W*theta - a_f;
		const Vector<float, 3+5> theta_dot = -P*W.transpose()*e_acc;
		theta_T += dt*theta_dot.slice<3, 1>(0, 0);
		theta_A += dt*theta_dot.slice<5, 1>(3, 0);
		// float lambda = _params_standard.lambda_P; // forgetting factor
		// P += dt*(-P*W.transpose()*W*P  + lambda * P);
	}

	// limit maximum deviation from initial value
	// const float max_param_deviation = _params_standard.theta_max_dev;
	// for (int i=0; i < 5; i++) {
	// 	float min = theta_A0(i)/max_param_deviation;
	// 	float max = theta_A0(i)*max_param_deviation;
	// 	if (theta_A(i) < min) {
	// 		theta_A(i) = min;
	// 	}
	// 	if (theta_A(i) > max) {
	// 		theta_A(i) = max;
	// 	}
	// }
	// for (int i=0; i < 3; i++) {
	// 	float min = theta_T0(i)/max_param_deviation;
	// 	float max = theta_T0(i)*max_param_deviation;
	// 	if (theta_T(i) < min) {
	// 		theta_T(i) = min;
	// 	}
	// 	if (theta_T(i) > max) {
	// 		theta_T(i) = max;
	// 	}
	// }

	theta_A(0) = limit(theta_A(0), theta_A0(0), 4); // CD0
	theta_A(1) = limit(theta_A(1), theta_A0(1), 4); // CD1
	theta_A(2) = limit(theta_A(2), theta_A0(2), 2); // CD2
	theta_A(3) = limit(theta_A(3), theta_A0(3), 4); // CL0
	theta_A(4) = limit(theta_A(4), theta_A0(4), 2); // CL1
	theta_T(0) = limit(theta_T(0), theta_T0(0), 2); // CTx
	theta_T(1) = limit(theta_T(1), theta_T0(1), 2); // CTz
	theta_T(2) = limit(theta_T(2), theta_T0(2), 4); // CTDz


	// Force allocation controller

	// direction of reference body z axis represented in earth frame
	const Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));
	// direction of the current body x axis in earth frame
	const Vector3f x_body_in_earth(R(0, 0), R(1, 0), R(2, 0));
	const Vector3f z_body_in_earth(R(0, 2), R(1, 2), R(2, 2));

	const float Vinf = fabsf(_airspeed->indicated_airspeed_m_s);

	// // previous inertial fixed airspeed direction
	// const Vector3f airspeed_dir(x_body_in_earth(0), x_body_in_earth(1), 0); // xy projection of body x-axis
	// const Vector3f airspeed_earth_frame = airspeed_dir.normalized()*Vinf;

	const Vector3f airspeed_body_frame(_airspeed->airspeed_body_x, _airspeed->airspeed_body_y, _airspeed->airspeed_body_z);
	const Vector3f airspeed_earth_frame = R*airspeed_body_frame;

	// reference force includes force to cancel gravity (in paper f_r does not include gravity)
	// (_v_att_sp->thrust_body[2] is negative)
	const Vector3f f_r_signal_units = body_z_sp * _v_att_sp->thrust_body[2]; // reference force in earth frame [signal]
	const Vector3f f_r = f_r_signal_units * (m*g/_params->mpc_thr_hover); // reference force in earth frame [N]

	// construct wind frame and decompose f_r in parallel and perpendicular
	const Vector3f x_wind_in_earth = airspeed_earth_frame.normalized();
	float f_r_parallel = x_wind_in_earth*f_r;
	const Vector3f f_r_parallel_v = f_r_parallel * x_wind_in_earth;
	const Vector3f f_r_perpendicular_v = f_r - f_r_parallel_v;
	const Vector3f y_wind_in_earth = (x_wind_in_earth.cross(f_r)).normalized();
	const Vector3f z_wind_in_earth = x_wind_in_earth.cross(y_wind_in_earth);
	// definition of scalar f_r_perpendicular is along z_w which is pointing away
	// from f_r, so f_r_perpendicular is always negative
	float f_r_perpendicular = -f_r_perpendicular_v.norm();

	// Low speed force allocation
	const Quatf att_sp_low_speed = _v_att_sp->q_d; // just use desired MC attitude

	// High speed force allocation
	float alpha_d = force_allocation_compute_desired_alpha(Vinf, aoa_max, -f_r_perpendicular, theta_A);
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


	// float aoa = atan2f(x_wind_in_earth * z_body_in_earth, x_wind_in_earth * x_body_in_earth);
	float aoa = _airspeed->aoa;
	if (Vinf < 1 || !PX4_ISFINITE(aoa)) {
		aoa = 0;
	}
	// lift/drag force
	build_aero_model_phi(aoa, Vinf, prev_Phi_A); // update Phi_A to compute current aero forces
	const Vector3f f_aero_b = m * prev_Phi_A * theta_A;
	const Vector3f f_aero = R*f_aero_b;
	const Vector3f f_th = f_r - f_aero;
	// thruster force in body z (negative)
	float fz = z_body_in_earth * f_th;

	// u^2 * CT = f
	float CTx = theta_T(0);
	float CTz = theta_T(1);
	float fz_signal_sq = fz / CTz; // this is negative
	if (-fz_signal_sq < sq(0.10f)) {
		fz_signal_sq = 0;
	}

	build_thrust_model_phi(0, -fz_signal_sq, airspeed_body_frame, aoa, prev_Phi_T);
	const Vector3f f_th_drag_compensated = f_th - R*(m*prev_Phi_T*theta_T);

	// thruster force in body x (positive)
	float fx = x_body_in_earth * f_th_drag_compensated;
	float fx_signal_sq = fx / CTx;
	if (fx_signal_sq < sq(0.10f)) {
		fx_signal_sq = 0;
	}
	build_thrust_model_phi(fx_signal_sq, -fz_signal_sq, airspeed_body_frame, aoa, prev_Phi_T);

	float fz_signal = sqrtf_signed(fz_signal_sq);
	float fx_signal = sqrtf_signed(fx_signal_sq);


	_pusher_throttle = fx_signal;
	att_sp.copyTo(_v_att_sp->q_d);
	_v_att_sp->q_d_valid = true;
	_v_att_sp->thrust_body[0] = 0;
	_v_att_sp->thrust_body[1] = 0;
	_v_att_sp->thrust_body[2] = fz_signal; // this is for multirotor thrust

	_pusher_throttle = _pusher_throttle < 0.0f ? 0.0f : _pusher_throttle;




 	///////////////////////////// publish debug values /////////////////////////

	static int i = 0;
	static int nb_elements = 1;
	i++;
	int i_mod = i%nb_elements;
	static struct debug_key_value_s dbg;
	// if (i_mod-- == 0) {
	// 	strncpy(dbg.key, "xx_Fa_z", 10);
	// 	dbg.value = f_aero_b(2);
	// }
	// if (i_mod-- == 0) {
	// 	strncpy(dbg.key, "xx_Fa_x", 10);
	// 	dbg.value = f_aero_b(0);
	// }
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_aoa", 10);
		dbg.value = aoa*180/(float)M_PI;
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_alphad", 10);
		dbg.value = alpha_d*180/(float)M_PI;
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_u2thz", 10);
		dbg.value = fz_signal_sq;
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_u2thx", 10);
		dbg.value = fx_signal_sq;
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CD0", 10);
		dbg.value = theta_A(0);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CD1", 10);
		dbg.value = theta_A(1);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CD2", 10);
		dbg.value = theta_A(2);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CL0", 10);
		dbg.value = theta_A(3);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CL1", 10);
		dbg.value = theta_A(4);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CTx", 10);
		dbg.value = theta_T(0);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CTz", 10);
		dbg.value = theta_T(1);
	}
	if (i_mod-- == 0) {
		strncpy(dbg.key, "xx_CTDz", 10);
		dbg.value = theta_T(2);
	}
	// // P matrix
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCTx", 10);
	// 	dbg.value = P(0,0);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCTz", 10);
	// 	dbg.value = P(1,1);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCTDz", 10);
	// 	dbg.value = P(2,2);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD0CD0", 10);
	// 	dbg.value = P(3,3);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD1CD1", 10);
	// 	dbg.value = P(4,4);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD2CD2", 10);
	// 	dbg.value = P(5,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CL0CL0", 10);
	// 	dbg.value = P(6,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CL1CL1", 10);
	// 	dbg.value = P(7,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCTz", 10);
	// 	dbg.value = P(0,1);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCTDz", 10);
	// 	dbg.value = P(0,2);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCD0", 10);
	// 	dbg.value = P(0,3);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCD1", 10);
	// 	dbg.value = P(0,4);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCD2", 10);
	// 	dbg.value = P(0,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCL0", 10);
	// 	dbg.value = P(0,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTxCL1", 10);
	// 	dbg.value = P(0,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCTDz", 10);
	// 	dbg.value = P(1,2);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCD0", 10);
	// 	dbg.value = P(1,3);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCD1", 10);
	// 	dbg.value = P(1,4);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCD2", 10);
	// 	dbg.value = P(1,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCL0", 10);
	// 	dbg.value = P(1,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTzCL1", 10);
	// 	dbg.value = P(1,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCD0", 10);
	// 	dbg.value = P(2,3);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCD1", 10);
	// 	dbg.value = P(2,4);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCD2", 10);
	// 	dbg.value = P(2,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCL0", 10);
	// 	dbg.value = P(2,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CTDzCL1", 10);
	// 	dbg.value = P(2,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD0CD1", 10);
	// 	dbg.value = P(3,4);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD0CD2", 10);
	// 	dbg.value = P(3,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD0CL0", 10);
	// 	dbg.value = P(3,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD0CL1", 10);
	// 	dbg.value = P(3,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD1CD2", 10);
	// 	dbg.value = P(4,5);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD1CL0", 10);
	// 	dbg.value = P(4,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD1CL1", 10);
	// 	dbg.value = P(4,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD2CL0", 10);
	// 	dbg.value = P(5,6);
	// }
	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CD2CL1", 10);
	// 	dbg.value = P(5,7);
	// }

	// if (i_mod-- ==  0) {
	// 	strncpy(dbg.key, "P_CL0CL1", 10);
	// 	dbg.value = P(6,7);
	// }

	if (-i_mod > nb_elements) {
		nb_elements = -i_mod;
	}

	orb_publish(ORB_ID(debug_key_value), _pub_dbg_val, &dbg);

	static struct debug_vect_s dbg_vect;
	static int j = 0;
	j++;
	int j_mod = j%3;
	if (j_mod == 0) {
		// dbg_vect.x = _airspeed->airspeed_body_x;
		// dbg_vect.y = _airspeed->airspeed_body_y;
		// dbg_vect.z = _airspeed->airspeed_body_z;
		// strncpy(dbg_vect.name, "xx_wind_b", 10);
		dbg_vect.x = v_err(0);
		dbg_vect.y = v_err(1);
		dbg_vect.z = v_err(2);
		strncpy(dbg_vect.name, "xx_v_err", 10);
	} else if (j_mod == 1) {
		// dbg_vect.x = fx;
		// dbg_vect.y = 0;
		// dbg_vect.z = fz;
		// strncpy(dbg_vect.name, "xx_f_th", 10);
		dbg_vect.x = e_acc(0);
		dbg_vect.y = e_acc(1);
		dbg_vect.z = e_acc(2);
		strncpy(dbg_vect.name, "xx_e_acc", 10);
	} else if (j_mod == 2) {
		// dbg_vect.x = f_r(0);
		// dbg_vect.y = f_r(1);
		// dbg_vect.z = f_r(2);
		// strncpy(dbg_vect.name, "xx_f_r", 10);
		// dbg_vect.x = a_f(0);
		// dbg_vect.y = a_f(1);
		// dbg_vect.z = a_f(2);
		// strncpy(dbg_vect.name, "xx_a_f", 10);
		dbg_vect.x = _airspeed->airspeed_body_x;
		dbg_vect.y = _airspeed->airspeed_body_y;
		dbg_vect.z = _airspeed->airspeed_body_z;
		strncpy(dbg_vect.name, "xx_wind_b", 10);
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

	if (_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] < 0.01f) {
		_mc_roll_weight = 0;
		_mc_pitch_weight = 0;
		_mc_yaw_weight = 0;
	}

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
