#include "pid_controller.h"


// PID controller module
// defining complete


namespace SelfController {


	double PIDController::Control(const double error, const double dt) {
		double diff = 0;
		double output = 0;
		// differential hold
		diff = (error - previous_error_) / dt;
		// integral hold
		integral_ += error * dt * ki_;
		previous_error_ = error;
		output = error * kp_ + integral_ + diff * kd_;  // Ki already applied
		previous_output_ = output;
		return output;
	}

	void PIDController::Reset() {
		previous_error_ = 0.0;
		previous_output_ = 0.0;
		integral_ = 0.0;
	}

	void PIDController::Init(const PidConf& pid_conf) {
		previous_error_ = 0.0;
		previous_output_ = 0.0;
		integral_ = 0.0;
		SetPID(pid_conf);
	}

	void PIDController::SetPID(const PidConf& pid_conf) {
		kp_ = pid_conf.kp();
		ki_ = pid_conf.ki();
		kd_ = pid_conf.kd();
		kaw_ = pid_conf.kaw();
	}

}