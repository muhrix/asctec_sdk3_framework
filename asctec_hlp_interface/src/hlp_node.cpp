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

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "hlp_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	// variables to store ROS parameters
	std::string serial_port;
	int baudrate;
	std::string frame_id;
	int packet_rate_imu;
	int packet_rate_gps;
	int packet_rate_rcdata;
	int packet_rate_status;
	int aci_engine_throttle;
	int aci_heartbeat;


	// fetch values from ROS parameter server
	priv_nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baudrate", baudrate, 230400);
	priv_nh.param("frame_id", frame_id, std::string("hlp"));
	// parameters below will be moving to dynamic reconfigure in a later release
	priv_nh.param("packet_rate_imu", packet_rate_imu, 50);
	priv_nh.param("packet_rate_gps", packet_rate_gps, 5);
	priv_nh.param("packet_rate_rcdata", packet_rate_rcdata, 10);
	priv_nh.param("packet_rate_status", packet_rate_status, 10);
	priv_nh.param("aci_engine_throttle", aci_engine_throttle, 100);
	priv_nh.param("aci_heartbeat", aci_heartbeat, 10);

	// advertise publishers
	ros::Publisher imu_pub = priv_nh.advertise<sensor_msgs::Imu>("imu", 1);
	ros::Publisher imu_custom_pub = priv_nh.advertise<asctec_hlp_comm::mav_imu>("imu_custom", 1);
	ros::Publisher gps_pub = priv_nh.advertise<sensor_msgs::NavSatFix>("gps", 1);
	ros::Publisher rcdata_pub = priv_nh.advertise<asctec_hlp_comm::mav_rcdata>("rcdata", 1);
	ros::Publisher status_pub = priv_nh.advertise<asctec_hlp_comm::mav_status>("status", 1);
	ros::Publisher motor_pub = priv_nh.advertise<asctec_hlp_comm::MotorSpeed>("motor_speed", 1);
	ros::Publisher gps_custom_pub = priv_nh.advertise<asctec_hlp_comm::GpsCustom>("gps_custom", 1);
	ros::Publisher mag_pub = priv_nh.advertise<geometry_msgs::Vector3Stamped>("mag", 1);

	// setup Asctec ACI
	AciRemote::AciRemote hlp(serial_port, baudrate, aci_engine_throttle, aci_heartbeat);

	if (hlp.Init() < 0) {
		ROS_ERROR("ACI initialisation returned -1");
		return EXIT_FAILURE;
	}

	ros::spin();

	return EXIT_SUCCESS;
}
