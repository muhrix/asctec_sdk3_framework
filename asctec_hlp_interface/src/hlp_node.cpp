/*
 * hlp_node.cpp
 *
 *  Created on: 21 Jan 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include <ros/ros.h>
#include "asctec_hlp_interface/SerialComm.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "hlp_node");
	ros::NodeHandle n;

	SerialComm serial;
	if (serial.openPort() < 0) {
		return -1;
	}
	else {
		serial.closePort();
	}

	return 0;
}
