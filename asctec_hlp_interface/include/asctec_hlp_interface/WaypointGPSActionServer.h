/*
 * WaypointGPSActionServer.h
 *
 *  Created on: 19 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#ifndef WAYPOINTGPSACTIONSERVER_H_
#define WAYPOINTGPSACTIONSERVER_H_

#include <boost/function.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <actionlib/server/simple_action_server.h>
#include "asctec_hlp_comm/WaypointGPSAction.h"
#include "asctec_hlp_comm/GeofenceSrv.h"

#include "asctec_hlp_interface/AsctecSDK3.h"
#include "asctec_hlp_interface/AciRemote.h"

namespace Waypoint {
	namespace Action {
		typedef enum {SUCCEEDED = 0, RUNNING, PREEMPTED, ABORTED,
			OUT_OF_GEOFENCE, VALID, WRONG_FLIGHT_MODE} action_t;
	}
	namespace Status {
		typedef enum {REACHED_POS = 0, REACHED_POS_TIME, WITHIN_20M, PILOT_ABORT } status_t;
	}
}

class WaypointGPSActionServer {
public:
	WaypointGPSActionServer(const std::string&, boost::shared_ptr<AciRemote::AciRemote>&);
	~WaypointGPSActionServer();

	// getters
	double getWaypointMaxSpeed() const;
	double getWaypointPositionAccuracy() const;
	double getWaypointTimeout() const;

	// setters
	void setWaypointMaxSpeed(const double);
	void setWaypointPositionAccuracy(const double);
	void setWaypointTimeout(const double);

//protected:
private:
	ros::NodeHandle n_;
	std::string action_name_;
	std::string geofence_srv_name_;

	ros::ServiceServer geofence_srv_;

	actionlib::SimpleActionServer<asctec_hlp_comm::WaypointGPSAction> as_;
	asctec_hlp_comm::WaypointGPSFeedback feedback_;
	asctec_hlp_comm::WaypointGPSResult result_;

	double waypt_max_speed_;
	double waypt_pos_acc_;
	double waypt_timeout_;
	boost::shared_ptr<geographic_msgs::GeoPose> waypoint_;
	std::vector<boost::shared_ptr<geographic_msgs::GeoPoint> > geofence_;

//private:
	void GpsWaypointAction(const asctec_hlp_comm::WaypointGPSGoalConstPtr&);

	bool geofenceServiceCallback(asctec_hlp_comm::GeofenceSrv::Request&,
			asctec_hlp_comm::GeofenceSrv::Response&);

	int verifyGeofence();
	Waypoint::Action::action_t verifyGpsWaypoint(const asctec_hlp_comm::WaypointGPSGoalConstPtr&);

	boost::function<int (const asctec_hlp_comm::WaypointGPSGoalConstPtr&)>
		sendGpsWaypointToHlp;
	boost::function<void (unsigned short&, double&)> fetchWayptNavStatus;
	boost::function<void (asctec_hlp_comm::WaypointGPSResult&)> fetchWayptResultPose;
};

#endif /* WAYPOINTGPSACTIONSERVER_H_ */
