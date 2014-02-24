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

WaypointGPSActionServer::WaypointGPSActionServer(const std::string& name):
	n_("~"),
	as_(n_, name, boost::bind(&WaypointGPSActionServer::GpsWaypointAction, this, _1), false),
	action_name_(name),
	waypt_max_speed_(100.0),
	waypt_pos_acc_(3.0),
	waypt_timeout_(10000) {
	n_.param("geofence_service", geofence_srv_name_, std::string("geofence"));
	// start action server
	as_.start();
}

WaypointGPSActionServer::~WaypointGPSActionServer() {

}



//-------------------------------------------------------
//	Public member functions
//-------------------------------------------------------

// getters
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
	ros::Rate rate(1);
	Waypoint::Action::action_t running;

	double roll, pitch, yaw;
	unsigned short nav_status;
	double dist_to_goal;

	tf::Quaternion q;
	tf::quaternionMsgToTF(goal->geo_pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	running = verifyGpsWaypoint(goal);

	if (running == Waypoint::Action::VALID) {
		ROS_INFO_STREAM("Flying to GPS waypoint (" << goal->geo_pose.position.latitude << ", "
				<< goal->geo_pose.position.longitude << ", "
				<< goal->geo_pose.position.altitude << ", "
				<< (yaw * 180.0 / M_PI) << ")");

		// assign the newly defined GPS waypoint to ACI Engine
		sendGpsWaypointToHlp(goal);
		running = Waypoint::Action::RUNNING;
	}
	// loop reading GPS waypoint navigation status variables
	while (running == Waypoint::Action::RUNNING) {
		// check if preempt has been requested by client (and if ROS is running ok)
		if (as_.isPreemptRequested() || !ros::ok()) {
			running = Waypoint::Action::PREEMPTED;
			break;
		}
		// fetch waypoint navigation status from HLP
		fetchWayptNavStatus(nav_status, dist_to_goal);
		if (nav_status & 0x08 == static_cast<unsigned short>(Waypoint::Status::PILOT_ABORT)) {
			running = Waypoint::Action::ABORTED;
			break;
		}
		else if ((nav_status & 0x01 ==
						static_cast<unsigned short>(Waypoint::Status::REACHED_POS))
				|| (nav_status & 0x02 ==
						static_cast<unsigned short>(Waypoint::Status::REACHED_POS))) {
			// set result
			fetchWayptResultPose(result_);
			result_.status = nav_status;
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
	if (running == Waypoint::Action::OUT_OF_GEOFENCE) {
		ROS_INFO_STREAM(action_name_ << ": Invalid waypoint (out of geofence)");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::PREEMPTED) {
		ROS_INFO_STREAM(action_name_ << ": Preempted by action client");
		as_.setPreempted();
	}
	else if (running == Waypoint::Action::ABORTED) {
		ROS_INFO_STREAM(action_name_ << ": Aborted by HLP/safety pilot");
		as_.setAborted();
	}
	else if (running == Waypoint::Action::SUCCEEDED) {
		ROS_INFO_STREAM(action_name_ << ": Waypoint navigation completed");
		as_.setSucceeded(result_);
	}
}

int WaypointGPSActionServer::verifyGeofence() {
	// TODO: implement equivalent to Boost geometry "within" function (only available from 1.47)
	return 0;
}

Waypoint::Action::action_t WaypointGPSActionServer::verifyGpsWaypoint(
		const asctec_hlp_comm::WaypointGPSGoalConstPtr& waypt) {
	// TODO: implement check to validate GPS waypoint (i.e., within geofence)
	return Waypoint::Action::VALID;
}


