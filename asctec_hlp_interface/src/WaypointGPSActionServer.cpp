/*
 * WaypointGPSActionServer.cpp
 *
 *  Created on: 19 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "asctec_hlp_interface/WaypointGPSActionServer.h"
#include "asctec_hlp_interface/Helper.h"

WaypointGPSActionServer::WaypointGPSActionServer(const std::string& name,
		boost::shared_ptr<AciRemote::AciRemote>& aci):
		n_("~"),
		as_(n_, name, boost::bind(&WaypointGPSActionServer::GpsWaypointAction, this, _1), false),
		action_name_(name),
		iter_rate_(1),
		waypt_max_speed_(100.0),
		waypt_pos_acc_(3.0),
		waypt_timeout_(10000),
		valid_geofence_(false) {

	sendGpsWaypointToHlp =
			boost::bind(&AciRemote::AciRemote::setGpsWaypoint, aci, _1);
	fetchWayptNavStatus =
			boost::bind(&AciRemote::AciRemote::getGpsWayptNavStatus, aci, _1, _2);
	fetchWayptState =
			boost::bind(&AciRemote::AciRemote::getGpsWayptState, aci, _1);
	fetchWayptResultPose =
			boost::bind(&AciRemote::AciRemote::getGpsWayptResultPose, aci, _1);

	n_.param("geofence_service", geofence_srv_name_, std::string("set_geofence"));

	geofence_srv_ = n_.advertiseService(geofence_srv_name_,
			&WaypointGPSActionServer::geofenceServiceCallback, this);
	ROS_INFO_STREAM("Service " << geofence_srv_name_ << " advertised");

	// start action server
	as_.start();

	ROS_INFO_STREAM(action_name_ << ": GPS waypoint action server started");
}

WaypointGPSActionServer::~WaypointGPSActionServer() {

}



//-------------------------------------------------------
//	Public member functions
//-------------------------------------------------------

// TODO: make getters and setters thread-safe

// getters
unsigned int WaypointGPSActionServer::getWaypointIterationRate() const {
	return iter_rate_;
}

double WaypointGPSActionServer::getWaypointMaxSpeed() const {
	return waypt_max_speed_;
}

double WaypointGPSActionServer::getWaypointPositionAccuracy() const {
	return waypt_pos_acc_;
}

double WaypointGPSActionServer::getWaypointTimeout() const {
	return waypt_timeout_;
}

// setters
void WaypointGPSActionServer::setWaypointIterationRate(const unsigned int i) {
	iter_rate_ = i;
}

void WaypointGPSActionServer::setWaypointMaxSpeed(const double s) {
	waypt_max_speed_ = s;
}

void WaypointGPSActionServer::setWaypointPositionAccuracy(const double a) {
	waypt_pos_acc_ = a;
}

void WaypointGPSActionServer::setWaypointTimeout(const double t) {
	waypt_timeout_ = t;
}



//-------------------------------------------------------
//	Private member functions
//-------------------------------------------------------

void WaypointGPSActionServer::GpsWaypointAction(
		const asctec_hlp_comm::WaypointGPSGoalConstPtr& goal) {
	// helper variables
	ros::Rate rate(iter_rate_);
	Waypoint::Action::action_t running = Waypoint::Action::NOT_READY;

	double roll, pitch, yaw;
	unsigned short nav_status, waypt_status;
	double dist_to_goal;
	int flight_mode;

	tf::Quaternion q;
	tf::quaternionMsgToTF(goal->geo_pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	running = verifyGpsWaypoint(goal);

	if (running == Waypoint::Action::VALID) {
		// assign the newly defined GPS waypoint to ACI Engine
		flight_mode = sendGpsWaypointToHlp(goal);
		if (flight_mode < static_cast<int>(Waypoint::State::RESET)) {
			running = Waypoint::Action::WRONG_FLIGHT_MODE;
		}
		else if (flight_mode == static_cast<int>(Waypoint::State::RESET)) {
			running = Waypoint::Action::WRONG_CTRL_MODE;
		}
		else if (flight_mode < static_cast<int>(Waypoint::State::READY)) {
			running = Waypoint::Action::NOT_READY;
		}
		else {
			ROS_INFO_STREAM("Flying to GPS waypoint ("
					<< goal->geo_pose.position.latitude << ", "
					<< goal->geo_pose.position.longitude << ", "
					<< goal->geo_pose.position.altitude << ", "
					<< (yaw * 180.0 / M_PI) << ")");
			running = Waypoint::Action::RUNNING;
		}
	}
	// loop reading GPS waypoint navigation status variables
	while (running == Waypoint::Action::RUNNING) {
		// fetch waypoint navigation status from HLP
		fetchWayptNavStatus(nav_status, dist_to_goal);
		fetchWayptState(waypt_status);
		// fetch current pose as the current result (in case succeeded or aborted)
		// notice that result_.status is also set within the function call
		fetchWayptResultPose(result_);

		// check if preempt has been requested by client (and if ROS is running ok)
		if (as_.isPreemptRequested() || !ros::ok()) {
			running = Waypoint::Action::PREEMPTED;
			break;
		}
		else if (nav_status &  static_cast<unsigned short>(Waypoint::Status::PILOT_ABORT)) {
			running = Waypoint::Action::ABORTED;
			break;
		}
		else if (waypt_status == static_cast<unsigned short>(Waypoint::State::RESET)) {
			running = Waypoint::Action::ABORTED;
			break;
		}
		else if ((nav_status & static_cast<unsigned short>(Waypoint::Status::REACHED_POS))
				||
				(nav_status & static_cast<unsigned short>(Waypoint::Status::REACHED_POS_TIME))) {
			running = Waypoint::Action::SUCCEEDED;
			break;
		}
		else {
			// set and publish feedback at each loop iteration
			feedback_.distance = dist_to_goal;
			feedback_.status = nav_status;
			as_.publishFeedback(feedback_);
		}
		rate.sleep();
	}

	if (running == Waypoint::Action::NOT_READY) {
		ROS_INFO_STREAM(action_name_ << ": Aborted (quadrotor is not ready for waypoint navigation)");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::OUT_OF_GEOFENCE) {
		ROS_INFO_STREAM(action_name_ << ": Invalid waypoint (out of geofence)");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::PREEMPTED) {
		ROS_INFO_STREAM(action_name_ << ": Preempted by action client");
		as_.setPreempted(result_);
	}
	else if (running == Waypoint::Action::ABORTED) {
		ROS_INFO_STREAM(action_name_ << ": Aborted by HLP/safety pilot");
		as_.setAborted(result_);
	}
	else if (running == Waypoint::Action::WRONG_FLIGHT_MODE) {
		ROS_INFO_STREAM(action_name_ << ": Aborted (flight mode is not GPS)");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::WRONG_CTRL_MODE) {
		ROS_INFO_STREAM(action_name_ << ": Aborted (control mode is not correctly set)");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::SUCCEEDED) {
		ROS_INFO_STREAM(action_name_ << ": Waypoint navigation completed");
		as_.setSucceeded(result_);
	}
}

bool WaypointGPSActionServer::geofenceServiceCallback(
		asctec_hlp_comm::GeofenceSrv::Request& req,
		asctec_hlp_comm::GeofenceSrv::Response& res) {
	boost::mutex::scoped_lock s_lock(geo_mtx_);
	if (req.append.data == false) {
		ROS_INFO("Clearing geofence points");
		geofence_.clear();
		geo_points_.clear();
	}

	ROS_INFO_STREAM("Appending " << req.geofence.size() << " new points to geofence");
	for (asctec_hlp_comm::GeofenceSrvRequest::_geofence_type::iterator it = req.geofence.begin();
			it != req.geofence.end(); ++it) {
		// remember that, with Boost functions, this implementation considers
		// a tuple as (longitude, latitude), which illustrates (X-axis, Y-axis)

		//boost::geometry::append(geofence_,
		//		boost::geometry::make<spherical_point_type>(it->longitude, it->latitude));
		geofence_.outer().push_back(
				boost::geometry::make<spherical_point_type>(it->longitude, it->latitude));

		geo_points_.push_back(std::pair<double, double>(it->latitude, it->longitude));
	}

	// verify number of vertices and close polygon if necessary
	verifyGeofence();

	res.curr_geofence.clear();
	for (std::vector<std::pair<double, double> >::iterator it = geo_points_.begin();
			it != geo_points_.end(); ++it) {
		geographic_msgs::GeoPoint pt;
		pt.latitude = it->first;
		pt.longitude = it->second;
		res.curr_geofence.push_back(pt);
	}

	return true;
}

void WaypointGPSActionServer::verifyGeofence() {
	if (geo_points_.size() > 2) {
		// correct (sort) geofence points and make sure the polygon is closed
		boost::geometry::correct(geofence_);

		// the code below once compiled fine, but it is not compiling anymore (do not know why)
		//std::sort(geo_points_.begin(), geo_points_.end(),
		//		boost::geometry::less<boost::tuple<double, double>, -1, std::less<double> >());
		std::sort(geo_points_.begin(), geo_points_.end());

		valid_geofence_ = true;
		ROS_INFO("Current geofence is valid");
	}
	else {
		valid_geofence_ = false;
		ROS_ERROR("Current geofence is invalid");
	}
}

Waypoint::Action::action_t WaypointGPSActionServer::verifyGpsWaypoint(
		const asctec_hlp_comm::WaypointGPSGoalConstPtr& waypt) {
	boost::mutex::scoped_lock s_lock(geo_mtx_);
	if (valid_geofence_) {
		// remember that, with Boost functions, this implementation considers
		// a tuple as (longitude, latitude), which illustrates (X-axis, Y-axis)
		spherical_point_type pt = boost::geometry::make<spherical_point_type>(
				waypt->geo_pose.position.longitude,
				waypt->geo_pose.position.latitude);

		if (boost::geometry::within(pt, geofence_)) {
			return Waypoint::Action::VALID;
		}
		else {
			return Waypoint::Action::OUT_OF_GEOFENCE;
		}
	}
	return Waypoint::Action::OUT_OF_GEOFENCE;
}
