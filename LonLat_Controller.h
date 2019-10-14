#include <memory>
#include <string>
#include <vector>
#include"pid_controller.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/interpolation_2d.h"
#include "modules/control/common/leadlag_controller.h"
//#include "modules/control/common/pid_controller.h"
#include "modules/control/common/trajectory_analyzer.h"
#include "modules/control/controller/controller.h"



namespace SelfController {

	/**
	 * @class SelfController
	 *
	 * @brief Longitudinal and lateral controller, to compute brake / throttle values and steer.
	 */
	class LonLatController {
	public:
		/**
		 * @brief constructor
		 */
		LonLatController();

		/**
		 * @brief destructor
		 */
		virtual ~LonLatController();

		/**
		 * @brief initialize Longitudinal Lateral Controller
		 * @param control_conf control configurations
		 * @return Status initialization status
		 */
		common::Status Init(const ControlConf* control_conf);

		/**
		 * @brief compute brake / throttle values based on current vehicle status
		 *        and target trajectory
		 * @param localization vehicle location
		 * @param chassis vehicle status e.g., speed, acceleration
		 * @param trajectory trajectory generated by planning
		 * @param cmd control command
		 * @return Status computation status
		 */
		common::Status ComputeControlCommand(
			const localization::LocalizationEstimate* localization,
			const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
			control::ControlCommand* cmd) override;

		/**
		 * @brief reset longitudinal controller
		 * @return Status reset status
		 */
		common::Status Reset();

		/**
		 * @brief stop longitudinal controller
		 */
		void Stop() ;

		/**
		 * @brief longitudinal controller name
		 * @return string controller name in string
		 */
		std::string Name() const ;

	protected:
		void ComputeLonLatErrors(const ErrorCalculator* error_calculator,
			const double preview_time, const double ts,SimpleLongitudinalDebug* debug);

	private:
		// Loading throttle brake table calibration
		void LoadControlCalibrationTable(const LonControllerConf& lon_controller_conf);

		void CloseLogFile();

		const localization::LocalizationEstimate* localization_ = nullptr;
		const canbus::Chassis* chassis_ = nullptr;

		// define some changed value, usefull, dont delete
		std::unique_ptr<Interpolation2D> control_interpolation_;
		const planning::ADCTrajectory* trajectory_message_ = nullptr;
		std::unique_ptr<ErrorCalculator> ErrorCalculator_;

		std::string name_;

		double previous_acceleration_ = 0.0;
		double previous_acceleration_reference_ = 0.0;

		PIDController speed_pid_controller_;
		PIDController station_pid_controller_;

		FILE* speed_log_file_ = nullptr;

		const ControlConf* control_conf_ = nullptr;

		// vehicle parameter
		common::VehicleParam vehicle_param_;
	}
}