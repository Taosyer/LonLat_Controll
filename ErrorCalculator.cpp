#include <iostream>
#include <cmath>
#include "ErrorCalculator.h"
#include "pid_controller.h"


//using apollo::common::PathPoint;
//using apollo::common::TrajectoryPoint;


namespace SelfController {

	TrajectoryPoint ErrorCalculator::QueryNearestPointByAbsoluteTime(const double t) const {
		return QueryNearestPointByRelativeTime(t - header_time_);
	}

	TrajectoryPoint ErrorCalculator::QueryNearestPointByRelativeTime(const double t) const {
		auto func_comp = [](const TrajectoryPoint& point, const double relative_time) {
			return point.relative_time() < relative_time;
		};
		auto it_low = std::lower_bound(trajectory_points_.begin(),
			trajectory_points_.end(), t, func_comp);

		if (it_low == trajectory_points_.begin()) {
			return trajectory_points_.front();
		}

		if (it_low == trajectory_points_.end()) {
			return trajectory_points_.back();
		}

		if (FLAGS_query_forward_time_point_only) {
			return *it_low;
		}
		else {
			auto it_lower = it_low - 1;
			if (it_low->relative_time() - t < t - it_lower->relative_time()) {
				return *it_low;
			}
			return *it_lower;
		}
	}

	void ErrorCalculator::ErrorCalculate(const double x, const double y, 
		const double theta, const double v,const double a,const PathPoint& m_point, 
		const double current_control_time, const double preview_control_time,
		double* ptr_lo, double* ptr_lo_dot, double* ptr_la, double* ptr_lo_a_desired) const {
		// calculate the error in the Geodetic coordinate system
		// the error between matched point and actual point
		// attention, desiered value - actual value
		double dx = m_point.x() - x;
		double dy = m_point.y() - y;
		// get the value of sin and cos in the Geodetic coordinate system
		double cos_m_theta = std::cos(m_point.theta());
		double sin_m_theta = std::sin(m_point.theta());
		// longitudinal error between matched point and actual point
		double longitudinal_error_m = dx * cos_m_theta + dy * sin_m_theta;
		*ptr_lo = m_point.s() + longitudinal_error_m;
		// heading error
		double delta_theta = theta - m_point.theta();
		// get the value of sin and cos in the vehicle coordinate system
		double cos_delta_theta = std::cos(delta_theta);
		double sin_delta_theta = std::sin(delta_theta);


		// dont' know
		double one_minus_kappa_r_d = 1 - m_point.kappa() * (*ptr_la);
		if (one_minus_kappa_r_d <= 0.0) {
			AERROR << " ErrorCalculator::ErrorCalculate found fatal reference and actual difference. "
				"Control output might be unstable:" << " m_point.kappa:" << m_point.kappa()
				<< " m_point.x:" << m_point.x() << " m_point.y:" << m_point.y() << " car x:" << x
				<< " car y:" << y << " *ptr_la:" << *ptr_la
				<< " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
			// currently set to a small value to avoid control crash.
			one_minus_kappa_r_d = 0.01;
		}
		// longitudinal spped in the matched point coordinate system, not error
		*ptr_lo_dot = v * cos_delta_theta / one_minus_kappa_r_d;


		TrajectoryPoint reference_point =
			trajectory_analyzer->QueryNearestPointByAbsoluteTime(
				current_control_time);

		TrajectoryPoint preview_point =
			trajectory_analyzer->QueryNearestPointByAbsoluteTime(
				preview_control_time);

		// longitudinal station error
		*ptr_lo = reference_point.path_point().s() - *ptr_lo;
		// longitudinal speed error
		*ptr_lo_dot = reference_point.v() - *ptr_lo_dot;
		// lateral station error
		// lateral error in the matched point's vehicle coordinate system
		// between preview point and actual point
		dx = preview_point.path_point().x() - x;
		dy = preview_point.path_point().y() - y;
		double lateral_error_p = -dx * sin_m_theta + dy * cos_m_theta;
		*ptr_la = lateral_error_p;
		// get the acceleration error
		double lon_acceleration = a* cos_delta_theta;
		double a_error = reference_point.a() - lon_acceleration ;
		// get the desired acceleration of reference point back to the fuction
		*ptr_lo_a_desired = reference_point.a();

	}

}



      



/*
// 调用
// 计算车辆实际位置与轨迹点的距离的平方值
// TrajectoryPoint为轨迹点
// x,y分别为车辆实际位置坐标
double PointDistanceSquare(const TrajectoryPoint& point, const double x, const double y) {
	const double dx = point.path_point().x() - x;
	const double dy = point.path_point().y() - y;
	return dx * dx * +dy * dy;
}

// 调用
// 不知道这个函数是干什么的，point.has_path_point()这个的定义是什么
PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint& point) {
	if (point.has_path_point()) {
		return point.path_point();
	}
	else {
		return PathPoint();
	}
}



// 调用
// 一个差值函数
template <typename T>
T lerp(const T& x0, const double t0, const T& x1, const double t1,
	const double t) {
	if (std::abs(t1 - t0) <= 1.0e-6) {
		AERROR << "input time difference is too small";
		return x0;
	}
	const double r = (t - t0) / (t1 - t0);
	const T x = x0 + r * (x1 - x0);
	return x;
}




// 调用
// 函数根据start和end两点进行差值计算，得到中间点
// p0为start点，p1为end点，x和y分别是实际车辆位置坐标
// 最后返回值是一个start和end两点的差值点
// lerp为math中的一个差值函数
PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint& p0, const TrajectoryPoint& p1, const double x, const double y) const {
	// given the fact that the discretized trajectory is dense enough,
	// we assume linear trajectory between consecutive trajectory points.

	// 差值计算点，px py横纵坐标
	auto dist_square = [&p0, &p1, &x, &y](const double s) {
		double px = common::math::lerp(p0.path_point().x(), p0.path_point().s(),
			p1.path_point().x(), p1.path_point().s(), s);
		double py = common::math::lerp(p0.path_point().y(), p0.path_point().s(),
			p1.path_point().y(), p1.path_point().s(), s);
		double dx = px - x;
		double dy = py - y;
		return dx * dx + dy * dy;
	};

	PathPoint p = p0.path_point();

	double s = common::math::GoldenSectionSearch(dist_square, p0.path_point().s(),
		p1.path_point().s());
	p.set_s(s);
	p.set_x(common::math::lerp(p0.path_point().x(), p0.path_point().s(),
		p1.path_point().x(), p1.path_point().s(), s));

	p.set_y(common::math::lerp(p0.path_point().y(), p0.path_point().s(),
		p1.path_point().y(), p1.path_point().s(), s));

	p.set_theta(common::math::slerp(p0.path_point().theta(), p0.path_point().s(),
		p1.path_point().theta(), p1.path_point().s(),
		s));

	// approximate the curvature at the intermediate point
	p.set_kappa(common::math::lerp(p0.path_point().kappa(), p0.path_point().s(),
		p1.path_point().kappa(), p1.path_point().s(),
		s));

	return p;
}





// 找相近的轨迹点
// 返回值为PathPoint，为一个类，using apollo::common::PathPoint;
// TrajectoryAnalyzer为一个类，对类中的成员函数QueryMatchedPathPoint进行定义
PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x, const double y) const {

	// CHECK_GT没有找到相应的定义
	// trajectory_points.size()应该为轨迹点的数量
	CHECK_GT(trajectory_points_.size(), 0);

	// std::vector<common::TrajectoryPoint> trajectory_points_; 
	// trajectory_points_.front() 轨迹前向点，具体定义不知道
	// 对轨迹上每个点计算距离，比对最小距离值，存储在d_min中
	double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);

	// index_min存储最小距离 是哪个轨迹上的点
	size_t index_min = 0;
	
	for (size_t i = 1; i < trajectory_points_.size(); ++i) {
		double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
		if (d_temp < d_min) {
			d_min = d_temp;
			index_min = i;
		}
	}

	// 以上述计算得到点，前向记录和后向记录
	size_t index_start = index_min == 0 ? index_min : index_min - 1;
	size_t index_end = index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

	// 如果start和end为同一点，则返回TrajectoryPointToPathPoint函数
	// 此处if语句应该是为了判断是否为最近的一点是轨迹最后的一点
	const double kEpsilon = 0.001;
	if (index_start == index_end ||
		std::fabs(trajectory_points_[index_start].path_point().s() - trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
		return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
	}

	// 反之 返回 FindMinDistancePoint函数，根据start与end两点进行差值
	return FindMinDistancePoint(trajectory_points_[index_start],trajectory_points_[index_end], x, y);
}




void LonController::ComputeLongitudinalErrors(
	const TrajectoryAnalyzer* trajectory_analyzer, const double preview_time,
	const double ts, SimpleLongitudinalDebug* debug) {
	// the decomposed vehicle motion onto Frenet frame
	// s: longitudinal accumulated distance along reference trajectory
	// s_dot: longitudinal velocity along reference trajectory
	// d: lateral distance w.r.t. reference trajectory
	// d_dot: lateral distance change rate, i.e. dd/dt
	double s_matched = 0.0;
	double s_dot_matched = 0.0;
	double d_matched = 0.0;
	double d_dot_matched = 0.0;


	// 匹配点是根据 车辆当前位置 按照 最小距离原则 得到的 轨迹上的一点
	// 这里可以看到 该函数的输入参数为 车辆当前的状态VehicleStateProvider所提供的的状态参数
	auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
		VehicleStateProvider::Instance()->x(),
		VehicleStateProvider::Instance()->y());

	trajectory_analyzer->ToTrajectoryFrame(
		VehicleStateProvider::Instance()->x(),
		VehicleStateProvider::Instance()->y(),
		VehicleStateProvider::Instance()->heading(),
		VehicleStateProvider::Instance()->linear_velocity(), matched_point,
		&s_matched, &s_dot_matched, &d_matched, &d_dot_matched);

	double current_control_time = Clock::NowInSeconds();
	double preview_control_time = current_control_time + preview_time;


	// 当前控制时间的  参考点——————绝对时间相近的点，以当前时间为函数的参考值
	TrajectoryPoint reference_point =
		trajectory_analyzer->QueryNearestPointByAbsoluteTime(
			current_control_time);
	// 在当前控制时间基础上 +预瞄时间   预瞄点——————绝对时间相近的点，以当前时间+预瞄时间为参考值
	TrajectoryPoint preview_point =
		trajectory_analyzer->QueryNearestPointByAbsoluteTime(
			preview_control_time);

	// 匹配点赋值
	debug->mutable_current_matched_point()->mutable_path_point()->set_x(
		matched_point.x());
	debug->mutable_current_matched_point()->mutable_path_point()->set_y(
		matched_point.y());
	// 参考点赋值
	debug->mutable_current_reference_point()->mutable_path_point()->set_x(
		reference_point.path_point().x());
	debug->mutable_current_reference_point()->mutable_path_point()->set_y(
		reference_point.path_point().y());
	// 预瞄点赋值
	debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
		preview_point.path_point().x());
	debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
		preview_point.path_point().y());

	ADEBUG << "matched point:" << matched_point.DebugString();
	ADEBUG << "reference point:" << reference_point.DebugString();
	ADEBUG << "preview point:" << preview_point.DebugString();

	// VehicleStateProvider提供的应该是车辆当前的状态参数
	// 航向角偏差————实际点 与 匹配点 差值
	double heading_error = common::math::NormalizeAngle(VehicleStateProvider::Instance()->heading() - matched_point.theta());
	// 纵向速度————实际点 在 匹配点处 纵向速度
	double lon_speed = VehicleStateProvider::Instance()->linear_velocity() *std::cos(heading_error);
	// 纵向加速度偏差————实际点 在 匹配点 纵向加速度
	double lon_acceleration = VehicleStateProvider::Instance()->linear_acceleration() * std::cos(heading_error);
	// 曲率偏差？————与匹配点差值
	double one_minus_kappa_lat_error =
		1 - reference_point.path_point().kappa() *
		VehicleStateProvider::Instance()->linear_velocity() *
		std::sin(heading_error);

	debug->set_station_reference(reference_point.path_point().s());
	debug->set_current_station(s_matched);
	// 参考点位置-匹配点位置=位置偏差，不是参考点-车辆实际位置偏差
	debug->set_station_error(reference_point.path_point().s() - s_matched);

	debug->set_speed_reference(reference_point.v());
	debug->set_current_speed(lon_speed);
	// 参考点速度-匹配点速度=速度偏差
	debug->set_speed_error(reference_point.v() - s_dot_matched);

	debug->set_acceleration_reference(reference_point.a());
	debug->set_current_acceleration(lon_acceleration);

	debug->set_acceleration_error(reference_point.a() -
		lon_acceleration / one_minus_kappa_lat_error);



	double jerk_reference =
		(debug->acceleration_reference() - previous_acceleration_reference_) / ts;
	double lon_jerk =
		(debug->current_acceleration() - previous_acceleration_) / ts;
	debug->set_jerk_reference(jerk_reference);
	debug->set_current_jerk(lon_jerk);
	debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
	previous_acceleration_reference_ = debug->acceleration_reference();
	previous_acceleration_ = debug->current_acceleration();

	// 预瞄点速度以及位置 与匹配点速度位置 偏差
	debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
	debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
	// 预瞄点速度及加速度
	debug->set_preview_speed_reference(preview_point.v());
	debug->set_preview_acceleration_reference(preview_point.a());
}


// 以上的代码可以得到相应的位置误差和速度误差，可以进行相应的PID计算

debug->set_station_error(reference_point.path_point().s() - s_matched);

debug->set_speed_error(reference_point.v() - s_dot_matched);

debug->set_acceleration_error(reference_point.a() -lon_acceleration / one_minus_kappa_lat_error);
