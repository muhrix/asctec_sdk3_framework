/*
 * hlp_node.cpp
 *
 *  Created on: 21 Jan 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "asctec_hlp_comm/mav_imu.h"
#include "asctec_hlp_comm/mav_rcdata.h"
#include "asctec_hlp_comm/mav_status.h"
#include "asctec_hlp_comm/MotorSpeed.h"
#include "asctec_hlp_comm/GpsCustom.h"

#include "asctec_hlp_interface/AciRemote.h"
#include "asctec_hlp_interface/WaypointGPSActionServer.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "hlp_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	boost::shared_ptr<AciRemote::AciRemote> hlp =
			boost::make_shared<AciRemote::AciRemote>(boost::ref(priv_nh));

	if (hlp->init() < 0) {
		return EXIT_FAILURE;
	}

	if (hlp->initRosLayer() < 0) {
		ROS_ERROR("ROS topics not advertised because data has not yet arrived from HLP");
		return EXIT_FAILURE;
	}

	WaypointGPSActionServer hlp_waypt("pelican_nav", hlp);

	ros::spin();

	return EXIT_SUCCESS;
}
