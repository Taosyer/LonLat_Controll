#include"ErrorCalculator.h"
#include"pid_controller.h"
#include"LonLat_Controller.h"
#include <algorithm>
#include <utility>

//#include "modules/control/controller/lon_controller.h"



#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/localization/common/localization_gflags.h"


namespace SelfController {
	LonLatController::LonLatController(): name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
		if (FLAGS_enable_csv_debug) {
			time_t rawtime;
			char name_buffer[80];
			std::time(&rawtime);
			std::tm time_tm;
			localtime_r(&rawtime, &time_tm);
			strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
			speed_log_file_ = fopen(name_buffer, "w");
			if (speed_log_file_ == nullptr) {
				AERROR << "Fail to open file:" << name_buffer;
				FLAGS_enable_csv_debug = false;
			}
			if (speed_log_file_ != nullptr) {
				fprintf(speed_log_file_,
					"station_error,"
					"speed_error,"
					"preview_acceleration_reference,"
					"acceleration_cmd,"
					"acceleration_lookup,"
					"calibration_value,"
					"throttle_cmd,"
					"brake_cmd,"
					"\r\n");
				fflush(speed_log_file_);
			}
			AINFO << name_ << " used.";
		}
	}





	void LonLatController::CloseLogFile() {
		if (FLAGS_enable_csv_debug) {
			if (speed_log_file_ != nullptr) {
				fclose(speed_log_file_);
				speed_log_file_ = nullptr;
			}
		}
	}




    void LonLatController::Stop() { CloseLogFile(); }
	




	LonLatController::~LonLatController() { CloseLogFile(); }





	Status LonLatController::Init(const ControlConf* control_conf) {
		control_conf_ = control_conf;
		const LonControllerConf & lon_controller_conf =control_conf_->lon_controller_conf();
		double ts = lon_controller_conf.ts();

		// intiate the conf of station_speed pid controller
		station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
		speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
		
		vehicle_param_.CopyFrom(common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

		// Loading throttle brake table calibration
		LoadControlCalibrationTable(lon_controller_conf);

		return Status::OK();
		}




	void  LonLatController::LoadControlCalibrationTable(
		const LonControllerConf& lon_controller_conf) {
		const auto& control_table = lon_controller_conf.calibration_table();
		AINFO << "Control calibration table loaded";
		AINFO << "Control calibration table size is "<< control_table.calibration_size();
		Interpolation2D::DataType xyz;
		for (const auto& calibration : control_table.calibration()) {
			xyz.push_back(std::make_tuple(calibration.speed(),
				calibration.acceleration(),
				calibration.command()));
		}
		control_interpolation_.reset(new Interpolation2D);
		CHECK(control_interpolation_->Init(xyz))<< "Fail to load control calibration table";
	}




	Status  LonLatController::ComputeControlCommand(const localization::LocalizationEstimate* localization,
		const canbus::Chassis* chassis, const planning::ADCTrajectory* planning_published_trajectory,
		control::ControlCommand* cmd) {

		localization_ = localization;
		chassis_ = chassis;

		trajectory_message_ = planning_published_trajectory;

		const LonControllerConf& lon_controller_conf = control_conf_->lon_controller_conf();

		auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
	    debug->Clear();

		double brake_cmd = 0.0;
		double throttle_cmd = 0.0;
		double ts = lon_controller_conf.ts();
	    double preview_time = lon_controller_conf.preview_window() * ts;

		ComputeLonLatErrors(ErrorCalculator_.get(), preview_time, ts, debug);
		
		double station_error_limit = lon_controller_conf.station_error_limit();
	
		if (VehicleStateProvider::Instance()->linear_velocity() <=lon_controller_conf.switch_speed()) {
			speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
		}
		else {speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
		}

		// get the speed offset accoding to the station pid controller
		double speed_offset =station_pid_controller_.Control(debug->station_error, ts);
		// speed pid controller input
		double speed_controller_input = speed_offset + debug->speed_error();
        // speed pid controller output, acceleration offset
		double acceleration_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input, ts);
		// acceleration cmd, a offset plus desired a
		double acceleration_cmd =acceleration_cmd_closeloop + debug->preview_acceleration_reference();
		// judge the direciton? foward accerleration or backward acceleration
		double acceleration_lookup = (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) ? -acceleration_cmd : acceleration_cmd;

		double throttle_lowerbound =std::max(vehicle_param_.throttle_deadzone(),lon_controller_conf.throttle_minimum_action());
		double brake_lowerbound =std::max(vehicle_param_.brake_deadzone(),lon_controller_conf.brake_minimum_action());

		double calibration_value = 0.0;
		calibration_value = control_interpolation_->Interpolate(std::make_pair(chassis_->speed_mps(), acceleration_lookup));

		if (acceleration_lookup >= 0) {
			if (calibration_value >= 0) {
				throttle_cmd = std::max(calibration_value, throttle_lowerbound);
			}
			else {
				throttle_cmd = throttle_lowerbound;
			}
			brake_cmd = 0.0;
		}
		else {
			throttle_cmd = 0.0;
			if (calibration_value >= 0) {
				brake_cmd = brake_lowerbound;
			}
			else {
				brake_cmd = std::max(-calibration_value, brake_lowerbound);
			}
		}

		debug->set_speed_offset(speed_offset);
		debug->set_acceleration_cmd(acceleration_cmd);
		debug->set_acceleration_lookup(acceleration_lookup);

		debug->set_throttle_cmd(throttle_cmd);
		debug->set_brake_cmd(brake_cmd);

		debug->set_speed_lookup(chassis_->speed_mps());
		debug->set_calibration_value(calibration_value);

		if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
			fprintf(speed_log_file_,
				"%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,\r\n",
				debug->station_error(), debug->speed_error(),
				debug->preview_acceleration_reference(),
				debug->acceleration_cmd, debug->acceleration_lookup(),
				debug->calibration_value, throttle_cmd, brake_cmd);
		}

		// if the car is driven by acceleration, disgard the cmd->throttle and brake
		cmd->set_throttle(throttle_cmd);
		cmd->set_brake(brake_cmd);
		cmd->set_acceleration(acceleration_cmd);

		return Status::OK();   
		}





	Status  LonLatController::Reset() {
		speed_pid_controller_.Reset();
		station_pid_controller_.Reset();
		return Status::OK();
		}





	std::string  LonLatController::Name() const { return name_; }




	void LonLatController::ComputeLonLatErrors(
		const ErrorCalculator* error_calculator, const double preview_time,
		const double ts, SimpleLongitudinalDebug* debug) {
	    // the decomposed vehicle motion onto Frenet frame
		// s: longitudinal accumulated distance along reference trajectory
		// s_dot: longitudinal velocity along reference trajectory
		// d: lateral distance w.r.t. reference trajectory
		// d_dot: lateral distance change rate, i.e. dd/dt
		double lon_error = 0.0;
		double lon_error_dot = 0.0;
		double lat_error = 0.0;
		double lon_error_doot = 0.0;

		auto matched_point = error_calculator->QueryMatchedPathPoint(
			VehicleStateProvider::Instance()->x(),VehicleStateProvider::Instance()->y());

		double current_control_time = Clock::NowInSeconds();
		double preview_control_time = current_control_time + preview_time;

		error_calculator->ErrorCalculate(VehicleStateProvider::Instance()->x(),
			VehicleStateProvider::Instance()->y(),VehicleStateProvider::Instance()->heading(),
			VehicleStateProvider::Instance()->linear_acceleration(),
			VehicleStateProvider::Instance()->linear_velocity(), matched_point, current_control_time, preview_control_time,
			&lon_error, &lon_error_dot, &lat_error, &lon_error_doot);


		debug->set_station_error(lon_error);
		debug->set_speed_error(lon_error_dot);
		debug->set_preview_station_error(lat_error);
		debug->set_acceleration_reference(lon_error_doot);
		debug->set_preview_acceleration_reference(lon_error_doot);
	}
}