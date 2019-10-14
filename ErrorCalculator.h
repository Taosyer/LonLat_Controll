#include <vector>


namespace SelfController {


	class ErrorCalculator {
	public:
		/**
		* @brief constructor
		*/
		ErrorCalculator() = default;

		/**
		* @brief destructor
		 */
		~ErrorCalculator() = default;

		/**
		* @brief convert a position with theta and speed to trajectory frame,
		* -longitudinal and lateral direction to the trajectory
		* @param x x-value of the position
		* @param y y-value of the position
		* @param theta heading angle on the position
		* @param v speed on the position
		* @param matched_point matched point on trajectory for the given position
		* @param ptr_s longitudinal distance
		* @param ptr_s_dot longitudinal speed
		* @param ptr_d lateral distance
		* @param ptr_d_dot lateral speed
		*/
		void ErrorCalculate(const double x, const double y, const double theta, const double v, 
			const double a,const common::PathPoint& matched_point, const double current_control_time,
			const double preview_control_time,
			double* ptr_s, double* ptr_s_dot, double* ptr_d, double* ptr_d_dot) const;

		/**
		* @brief query a point of trajectory that its absolute time is closest
		* to the give time.
		* @param t absolute time for query
		* @return a point of trajectory
		*/
		common::TrajectoryPoint QueryNearestPointByAbsoluteTime(const double t) const;

		/**
		* @brief query a point of trajectory that its relative time is closest
		* to the give time. The time is relative to the first pointof trajectory
		* @param t relative time for query
		* @return a point of trajectory
		*/
		common::TrajectoryPoint QueryNearestPointByRelativeTime(const double t) const;


	};

}