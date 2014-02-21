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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <asctec_hlp_comm/WaypointGPSAction.h>

#include "asctec_hlp_interface/AsctecSDK3.h"

class WaypointGPSActionServer {
public:
	WaypointGPSActionServer(const std::string&);
	~WaypointGPSActionServer();

	void executeCallback(const asctec_hlp_comm::WaypointGPSGoalConstPtr&);

protected:
	ros::NodeHandle n_;
	std::string action_name_;
	actionlib::SimpleActionServer<asctec_hlp_comm::WaypointGPSAction> as_;
	asctec_hlp_comm::WaypointGPSActionFeedback feedback_;
	asctec_hlp_comm::WaypointGPSActionResult result_;

	struct WAYPOINT waypoint_;
};

#endif /* WAYPOINTGPSACTIONSERVER_H_ */
