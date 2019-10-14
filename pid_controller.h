#include "modules/control/proto/pid_conf.pb.h"


// �������������Ҫ��PID Conf �ļ���Ԥ�����õĸ�������ֵ
// �����Ķ�OK��

namespace SelfController {


	/**
	* @class PIDController
	* @brief A proportional�Cintegral�Cderivative controller for speed and steering
	  using defualt integral hold
	*/
	class PIDController {
	public:
		/**
		* @brief initialize pid controller
		* @param pid_conf configuration for pid controller
		*/
		void Init(const PidConf& pid_conf);

		/**
		* @brief set pid controller coefficients for the proportional,
		* integral, and derivative
		* @param pid_conf configuration for pid controller
		*/
		void SetPID(const PidConf& pid_conf);

		/**
		* @brief reset variables for pid controller
		*/
		void Reset();

		/**
		* @brief compute control value based on the error
		* @param error error value, the difference between
		* a desired value and a measured value
		* @param dt sampling time interval
		* @return control value based on PID terms
		*/
		virtual double Control(const double error, const double dt);

	protected:
		double kp_ = 0.0;
		double ki_ = 0.0;
		double kd_ = 0.0;
		double kaw_ = 0.0;
		double previous_error_ = 0.0;
		double previous_output_ = 0.0;
		double integral_ = 0.0;
	};
}
