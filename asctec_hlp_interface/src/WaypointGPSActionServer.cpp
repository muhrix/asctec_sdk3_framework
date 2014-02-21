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
	as_(n_, name, boost::bind(&WaypointGPSActionServer::executeCallback, this, _1), false),
	action_name_(name) {
	// subscribe to the data topic of interest
	as_.start();
}

WaypointGPSActionServer::~WaypointGPSActionServer() {

}

void WaypointGPSActionServer::executeCallback(
		const asctec_hlp_comm::WaypointGPSGoalConstPtr& goal) {

	bool success = true;

	// assign the newly defined GPS waypoint to waypoint_ data structure


	// loop reading GPS waypoint navigation status variables
	// check whether current waypoint should be preempted
	// set and publish feedback at each loop iteration
}


