/*
 * AciRemote.cpp
 *
 *  Created on: 14 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "asctec_hlp_interface/AciRemote.h"
#include "asctec_hlp_interface/Helper.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "asctec_hlp_comm/mav_imu.h"
#include "asctec_hlp_comm/mav_rcdata.h"
#include "asctec_hlp_comm/mav_status.h"
#include "asctec_hlp_comm/MotorSpeed.h"
#include "asctec_hlp_comm/GpsCustom.h"

namespace AciRemote {

//extern "C" {
//	void transmit(void* bytes, unsigned short len) {
//
//	}
//}

void* aci_obj_ptr = NULL;

AciRemote::AciRemote(ros::NodeHandle& nh):
		SerialComm(), n_(nh), bytes_recv_(0),
		versions_match_(false), var_list_recv_(false),
		cmd_list_recv_(false), par_list_recv_(false) {

	// assign *this pointer of this instanced object to global pointer for use with callbacks
	aci_obj_ptr = static_cast<void*>(this);

	// fetch values from ROS parameter server
	n_.param("serial_port", port_name_, std::string("/dev/ttyUSB0"));
	n_.param("baudrate", baud_rate_, 230400);
	n_.param("frame_id", frame_id_, std::string(n_.getNamespace() + "_base_link"));
	// parameters below will (that is, should) be moving to dynamic reconfigure in a later release
	n_.param("packet_rate_imu_mag", imu_rate_, 50);
	n_.param("packet_rate_gps", gps_rate_, 5);
	n_.param("packet_rate_rcdata_status_motors", rc_status_rate_, 10);
	n_.param("aci_engine_throttle", aci_rate_, 100);
	n_.param("aci_heartbeat", aci_heartbeat_, 10);

	// fetch topic names from ROS parameter server
	n_.param("imu_topic", imu_topic_, std::string("imu"));
	n_.param("imu_custom_topic", imu_custom_topic_, std::string("imu_custom"));
	n_.param("mag_topic", mag_topic_, std::string("mag"));
	n_.param("gps_topic", gps_topic_, std::string("gps"));
	n_.param("gps_custom_topic", gps_custom_topic_, std::string("gps_custom"));
	n_.param("rcdata_topic", rcdata_topic_, std::string("rcdata"));
	n_.param("status_topic", status_topic_, std::string("status"));
	n_.param("motor_speed_topic", motor_topic_, std::string("motor_speed"));

	// TODO: Initialise Asctec SDK3 data structures!!!
}

AciRemote::~AciRemote() {
	// interrupt all running threads...
	aci_throttle_thread_->interrupt();
	imu_mag_thread_->interrupt();
	gps_thread_->interrupt();
	rc_status_thread_->interrupt();
	// ... and wait for them to return
	aci_throttle_thread_->join();
	imu_mag_thread_->join();
	gps_thread_->join();
	rc_status_thread_->join();

	// assign NULL to global object pointer
	aci_obj_ptr = static_cast<void*>(NULL);
}



//-------------------------------------------------------
//	Public member functions
//-------------------------------------------------------

int AciRemote::Init() {
	// open and configure serial port
	if (openPort() < 0) {
		return -1;
	}
	// initialise ACI Remote
	aciInit();
	ROS_INFO("Asctec ACI initialised");

	// set callbacks
	aciSetSendDataCallback(AciRemote::transmit);
	aciInfoPacketReceivedCallback(AciRemote::versions);
	aciSetVarListUpdateFinishedCallback(AciRemote::varListUpdateFinished);
	aciSetCmdListUpdateFinishedCallback(AciRemote::cmdListUpdateFinished);
	aciSetParamListUpdateFinishedCallback(AciRemote::paramListUpdateFinished);
	aciSetEngineRate(aci_rate_, aci_heartbeat_);

	try {
		aci_throttle_thread_ = boost::shared_ptr<boost::thread>
			(new boost::thread(boost::bind(&AciRemote::throttleEngine, this)));
	}
	catch (boost::system::system_error::exception& e) {
		ROS_ERROR_STREAM("Could not create ACI Engine thread. " << e.what());
	}

	cond_.notify_one();

	// request version info and lists of commands, parameters and variables to HLP
	aciCheckVerConf();
	aciGetDeviceCommandsList();
	aciGetDeviceParametersList();
	aciGetDeviceVariablesList();

	return 0;
}

int AciRemote::advertiseRosTopics() {
	int c = 0;
	boost::asio::io_service io;
	while (++c < 4) {
		boost::unique_lock<boost::mutex> u_lock(mtx_);
		if (versions_match_ && var_list_recv_ && cmd_list_recv_ && par_list_recv_) {
			// advertise ROS topics
			imu_pub_ = n_.advertise<sensor_msgs::Imu>(imu_topic_, 1);
			imu_custom_pub_ = n_.advertise<asctec_hlp_comm::mav_imu>(imu_custom_topic_, 1);
			mag_pub_ = n_.advertise<geometry_msgs::Vector3Stamped>(mag_topic_, 1);
			gps_pub_ = n_.advertise<sensor_msgs::NavSatFix>(gps_topic_, 1);
			gps_custom_pub_ = n_.advertise<asctec_hlp_comm::GpsCustom>(gps_custom_topic_, 1);
			rcdata_pub_ = n_.advertise<asctec_hlp_comm::mav_rcdata>(rcdata_topic_, 1);
			status_pub_ = n_.advertise<asctec_hlp_comm::mav_status>(status_topic_, 1);
			motor_pub_ = n_.advertise<asctec_hlp_comm::MotorSpeed>(motor_topic_, 1);

			// spawn publisher threads
			try {
				imu_mag_thread_ = boost::shared_ptr<boost::thread>
					(new boost::thread(boost::bind(&AciRemote::publishImuMagData, this)));
			}
			catch (boost::system::system_error::exception& e) {
				ROS_ERROR_STREAM("Could not create IMU publisher thread. " << e.what());
			}
			try {
				gps_thread_ = boost::shared_ptr<boost::thread>
					(new boost::thread(boost::bind(&AciRemote::publishGpsData, this)));
			}
			catch (boost::system::system_error::exception& e) {
				ROS_ERROR_STREAM("Could not create GPS publisher thread. " << e.what());
			}
			try {
				rc_status_thread_ = boost::shared_ptr<boost::thread>
					(new boost::thread(boost::bind(&AciRemote::publishStatusMotorsRcData, this)));
			}
			catch (boost::system::system_error::exception& e) {
				ROS_ERROR_STREAM("Could not create Status publisher thread. " << e.what());
			}

			return 0;
		}
		else {
			// release mutex, wait 5 seconds and try again (no more than 3 times)
			u_lock.unlock();
			boost::asio::deadline_timer t(io, boost::posix_time::seconds(5));
			t.wait();
		}
	}
	return -1;
}



//-------------------------------------------------------
//	Protected member functions
//-------------------------------------------------------

void AciRemote::checkVersions(struct ACI_INFO aciInfo) {
	ROS_INFO("Received versions list from HLP");
	bool match = true;

	ROS_INFO("Type\t\t\tDevice\t\tRemote");
	ROS_INFO("Major version\t\t%d\t=\t\%d",aciInfo.verMajor,ACI_VER_MAJOR);
	ROS_INFO("Minor version\t\t%d\t=\t\%d",aciInfo.verMinor,ACI_VER_MINOR);
	ROS_INFO("MAX_DESC_LENGTH\t\t%d\t=\t\%d",aciInfo.maxDescLength,MAX_DESC_LENGTH);
	ROS_INFO("MAX_NAME_LENGTH\t\t%d\t=\t\%d",aciInfo.maxNameLength,MAX_NAME_LENGTH);
	ROS_INFO("MAX_UNIT_LENGTH\t\t%d\t=\t\%d",aciInfo.maxUnitLength,MAX_UNIT_LENGTH);
	ROS_INFO("MAX_VAR_PACKETS\t\t%d\t=\t\%d",aciInfo.maxVarPackets,MAX_VAR_PACKETS);

	if (aciInfo.verMajor != ACI_VER_MAJOR ||
			aciInfo.verMinor != ACI_VER_MINOR ||
			aciInfo.maxDescLength != MAX_DESC_LENGTH ||
			aciInfo.maxNameLength != MAX_NAME_LENGTH ||
			aciInfo.maxUnitLength != MAX_UNIT_LENGTH ||
			aciInfo.maxVarPackets != MAX_VAR_PACKETS) {
		match = false;
	}

	if (match) {
		boost::mutex::scoped_lock lock(mtx_);
		versions_match_ = true;
	}
	else {
		ROS_ERROR_STREAM("ACI versions do not match. Must abort now.");
	}
}

void AciRemote::setupVarPackets() {
	ROS_INFO("Received variables list from HLP");
	// setup variables packets to be received
	// along with reception rate (not more than ACI Engine rate)

	// packet ID 0 containing: status, motor speed and RC data
	// status data
	aciAddContentToVarPacket(0, 0x0001, &RO_ALL_Data_.UAV_status);
	aciAddContentToVarPacket(0, 0x0002, &RO_ALL_Data_.flight_time);
	aciAddContentToVarPacket(0, 0x0003, &RO_ALL_Data_.battery_voltage);
	aciAddContentToVarPacket(0, 0x0004, &RO_ALL_Data_.HL_cpu_load);
	aciAddContentToVarPacket(0, 0x0005, &RO_ALL_Data_.HL_up_time);
	// motor speed data
	aciAddContentToVarPacket(0, 0x0100, &RO_ALL_Data_.motor_rpm[0]);
	aciAddContentToVarPacket(0, 0x0101, &RO_ALL_Data_.motor_rpm[1]);
	aciAddContentToVarPacket(0, 0x0102, &RO_ALL_Data_.motor_rpm[2]);
	aciAddContentToVarPacket(0, 0x0103, &RO_ALL_Data_.motor_rpm[3]);
	// Rc data
	aciAddContentToVarPacket(0, 0x0600, &RO_ALL_Data_.channel[0]);
	aciAddContentToVarPacket(0, 0x0601, &RO_ALL_Data_.channel[1]);
	aciAddContentToVarPacket(0, 0x0602, &RO_ALL_Data_.channel[2]);
	aciAddContentToVarPacket(0, 0x0603, &RO_ALL_Data_.channel[3]);
	aciAddContentToVarPacket(0, 0x0604, &RO_ALL_Data_.channel[4]);
	aciAddContentToVarPacket(0, 0x0605, &RO_ALL_Data_.channel[5]);
	aciAddContentToVarPacket(0, 0x0606, &RO_ALL_Data_.channel[6]);
	aciAddContentToVarPacket(0, 0x0607, &RO_ALL_Data_.channel[7]);

	// packet ID 1 containing: GPS data
	aciAddContentToVarPacket(1, 0x0106, &RO_ALL_Data_.GPS_latitude);
	aciAddContentToVarPacket(1, 0x0107, &RO_ALL_Data_.GPS_longitude);
	aciAddContentToVarPacket(1, 0x0108, &RO_ALL_Data_.GPS_height);
	aciAddContentToVarPacket(1, 0x0109, &RO_ALL_Data_.GPS_speed_x);
	aciAddContentToVarPacket(1, 0x010A, &RO_ALL_Data_.GPS_speed_y);
	aciAddContentToVarPacket(1, 0x010B, &RO_ALL_Data_.GPS_heading);
	aciAddContentToVarPacket(1, 0x010C, &RO_ALL_Data_.GPS_position_accuracy);
	aciAddContentToVarPacket(1, 0x010D, &RO_ALL_Data_.GPS_height_accuracy);
	aciAddContentToVarPacket(1, 0x010E, &RO_ALL_Data_.GPS_speed_accuracy);
	aciAddContentToVarPacket(1, 0x010F, &RO_ALL_Data_.GPS_sat_num);
	aciAddContentToVarPacket(1, 0x0110, &RO_ALL_Data_.GPS_status);
	aciAddContentToVarPacket(1, 0x0111, &RO_ALL_Data_.GPS_time_of_week);
	aciAddContentToVarPacket(1, 0x0112, &RO_ALL_Data_.GPS_week);
	aciAddContentToVarPacket(1, 0x0303, &RO_ALL_Data_.fusion_latitude);
	aciAddContentToVarPacket(1, 0x0304, &RO_ALL_Data_.fusion_longitude);
	aciAddContentToVarPacket(1, 0x0305, &RO_ALL_Data_.fusion_height);
	aciAddContentToVarPacket(1, 0x0306, &RO_ALL_Data_.fusion_dheight);
	aciAddContentToVarPacket(1, 0x0307, &RO_ALL_Data_.fusion_speed_x);
	aciAddContentToVarPacket(1, 0x0308, &RO_ALL_Data_.fusion_speed_y);
	// TODO: double check whether the variables below are really necessary
	// TODO: use 0x100C and 0x100D below (should change HLP firmware as well)
	aciAddContentToVarPacket(1, 0x1012, &wpCtrlNavStatus_);
	aciAddContentToVarPacket(1, 0x1013, &wpCtrlDistToWp_);


	// packet ID 2 containing: IMU + magnetometer
	aciAddContentToVarPacket(2, 0x0200, &RO_ALL_Data_.angvel_pitch);
	aciAddContentToVarPacket(2, 0x0201, &RO_ALL_Data_.angvel_roll);
	aciAddContentToVarPacket(2, 0x0202, &RO_ALL_Data_.angvel_yaw);
	aciAddContentToVarPacket(2, 0x0203, &RO_ALL_Data_.acc_x);
	aciAddContentToVarPacket(2, 0x0204, &RO_ALL_Data_.acc_y);
	aciAddContentToVarPacket(2, 0x0205, &RO_ALL_Data_.acc_z);
	aciAddContentToVarPacket(2, 0x0206, &RO_ALL_Data_.Hx);
	aciAddContentToVarPacket(2, 0x0207, &RO_ALL_Data_.Hy);
	aciAddContentToVarPacket(2, 0x0208, &RO_ALL_Data_.Hz);
	aciAddContentToVarPacket(2, 0x0300, &RO_ALL_Data_.angle_pitch);
	aciAddContentToVarPacket(2, 0x0301, &RO_ALL_Data_.angle_roll);
	aciAddContentToVarPacket(2, 0x0302, &RO_ALL_Data_.angle_yaw);

	// set transmission rate for packets, update and send configuration
	aciSetVarPacketTransmissionRate(0, rc_status_rate_);
	aciSetVarPacketTransmissionRate(1, gps_rate_);
	aciSetVarPacketTransmissionRate(2, imu_rate_);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(0);
	aciSendVariablePacketConfiguration(1);
	aciSendVariablePacketConfiguration(2);

	ROS_INFO_STREAM("Variables packets configured");

	boost::mutex::scoped_lock lock(mtx_);
	var_list_recv_ = true;
}

void AciRemote::setupCmdPackets() {
	ROS_INFO("Received commands list from HLP");
	// setup commands packets to be sent over to the HLP
	// along with configuration to whether or not receive ACK

	// packet ID 0 containing: control mode
	aciAddContentToCmdPacket(0, 0x0600, &WO_SDK_.ctrl_mode);
	aciAddContentToCmdPacket(0, 0x0601, &WO_SDK_.ctrl_enabled);
	aciAddContentToCmdPacket(0, 0x0602, &WO_SDK_.disable_motor_onoff_by_stick);

	// packet ID 1 containing: DMC + CTRL -- direct individual motor control not used here
	aciAddContentToCmdPacket(1, 0x0506, &WO_DMC_.pitch);
	aciAddContentToCmdPacket(1, 0x0507, &WO_DMC_.roll);
	aciAddContentToCmdPacket(1, 0x0508, &WO_DMC_.yaw);
	aciAddContentToCmdPacket(1, 0x0509, &WO_DMC_.thrust);
	aciAddContentToCmdPacket(1, 0x050A, &WO_CTRL_.pitch);
	aciAddContentToCmdPacket(1, 0x050B, &WO_CTRL_.roll);
	aciAddContentToCmdPacket(1, 0x050C, &WO_CTRL_.yaw);
	aciAddContentToCmdPacket(1, 0x050D, &WO_CTRL_.thrust);
	aciAddContentToCmdPacket(1, 0x050E, &WO_CTRL_.ctrl);

	// packet ID 2 containing: single waypoint data structure
	aciAddContentToCmdPacket(2, 0x1001, &WO_wpToLL_.wp_activated);
	aciAddContentToCmdPacket(2, 0x1002, &WO_wpToLL_.properties);
	aciAddContentToCmdPacket(2, 0x1003, &WO_wpToLL_.max_speed);
	aciAddContentToCmdPacket(2, 0x1004, &WO_wpToLL_.time);
	aciAddContentToCmdPacket(2, 0x1005, &WO_wpToLL_.pos_acc);
	aciAddContentToCmdPacket(2, 0x1006, &WO_wpToLL_.chksum);
	aciAddContentToCmdPacket(2, 0x1007, &WO_wpToLL_.X);
	aciAddContentToCmdPacket(2, 0x1008, &WO_wpToLL_.Y);
	aciAddContentToCmdPacket(2, 0x1009, &WO_wpToLL_.yaw);
	// TODO: use 0x100A and 0x100B below (should change HLP firmware as well)
	aciAddContentToCmdPacket(2, 0x1010, &WO_wpToLL_.height);
	aciAddContentToCmdPacket(2, 0x1011, &wpCtrlWpCmd_);

	// set whether or not should receive ACK, and send configuration
	aciSendCommandPacketConfiguration(0, 1); // control mode must be set with ACK
	aciSendCommandPacketConfiguration(1, 0);
	aciSendCommandPacketConfiguration(2, 0);

	// send commands to HLP (DANGER: make sure data structures were properly initialised)
	aciUpdateCmdPacket(0);
	aciUpdateCmdPacket(1);
	aciUpdateCmdPacket(2);

	ROS_INFO_STREAM("Commands packets configured");

	boost::mutex::scoped_lock lock(mtx_);
	cmd_list_recv_ = true;
}

void AciRemote::setupParPackets() {
	ROS_INFO("Received parameters list from HLP");
	boost::mutex::scoped_lock lock(mtx_);
	par_list_recv_ = true;
}



//-------------------------------------------------------
//	Private member functions
//-------------------------------------------------------

void AciRemote::transmit(void* bytes, unsigned short len) {
	AciRemote* this_obj = static_cast<AciRemote*>(aci_obj_ptr);
	this_obj->doWrite(bytes, len);
}

void AciRemote::versions(struct ACI_INFO aciInfo) {
	AciRemote* this_obj = static_cast<AciRemote*>(aci_obj_ptr);
	this_obj->checkVersions(aciInfo);
}

void AciRemote::varListUpdateFinished() {
	AciRemote* this_obj = static_cast<AciRemote*>(aci_obj_ptr);
	this_obj->setupVarPackets();
}

void AciRemote::cmdListUpdateFinished() {
	AciRemote* this_obj = static_cast<AciRemote*>(aci_obj_ptr);
	this_obj->setupCmdPackets();
}

void AciRemote::paramListUpdateFinished() {
	AciRemote* this_obj = static_cast<AciRemote*>(aci_obj_ptr);
	this_obj->setupParPackets();
}

void AciRemote::readHandler(const boost::system::error_code& error,
		size_t bytes_transferred) {
	if (!error) {
		// the read_buffer_ should have some data from initial/previous call to
		// async_read_some ( doRead() ), thus feed ACI Engine with received data
		boost::unique_lock<boost::mutex> lock(buf_mtx_);
		for (std::vector<unsigned char>::iterator it = read_buffer_.begin();
				it != (read_buffer_.begin() + bytes_transferred); ++it) {
			aciReceiveHandler(*it);
		}
		lock.unlock();
		// carry on reading more data into the buffer...
		doRead();
	}
	else {
		ROS_ERROR_STREAM("Async read to serial port " << port_name_ << ". " << error.message());
		if (isOpen()) {
			doClose();
		}
	}
}

void AciRemote::throttleEngine() {
	ROS_INFO_STREAM("ACI Engine thread throttling at " << aci_rate_ << " Hz");
	// throttle ACI Engine at aci_rate_ Hz
	int aci_throttle = 1000 / aci_rate_;

	try {
		for (;;) {
			boost::system_time const throttle_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(aci_throttle);

			boost::unique_lock<boost::mutex> u_lock(buf_mtx_);

			if (cond_.timed_wait(u_lock, throttle_timeout) == false) {
				// throttle ACI Engine
				aciEngine();

				// lock shared mutex: get upgradable then exclusive access
				boost::upgrade_lock<boost::shared_mutex> up_lock(shared_mtx_);
				boost::upgrade_to_unique_lock<boost::shared_mutex> un_lock(up_lock);
				// synchronise variables
				aciSynchronizeVars();
			}
		}
	}
	catch (boost::thread_interrupted const&) {

	}
}

void AciRemote::publishImuMagData() {
	sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
	asctec_hlp_comm::mav_imuPtr imu_custom_msg(new asctec_hlp_comm::mav_imu);
	geometry_msgs::Vector3StampedPtr mag_msg(new geometry_msgs::Vector3Stamped);
	int imu_throttle = 1000 / imu_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const throttle_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(imu_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);
			// only publish if someone has already subscribed to topics
			if (imu_pub_.getNumSubscribers() > 0) {

			}
			if (imu_custom_pub_.getNumSubscribers() > 0) {

			}
			if (mag_pub_.getNumSubscribers() > 0) {

			}
		}
	}
	catch (boost::thread_interrupted const&) {

	}
}

void AciRemote::publishGpsData() {
	sensor_msgs::NavSatFixPtr gps_msg(new sensor_msgs::NavSatFix);
	asctec_hlp_comm::GpsCustomPtr gps_custom_msg(new asctec_hlp_comm::GpsCustom);
	int gps_throttle = 1000 / gps_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const throttle_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(gps_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);
			// only publish if someone has already subscribed to topics
			if (gps_pub_.getNumSubscribers() > 0) {

			}
			if (gps_custom_pub_.getNumSubscribers() > 0) {

			}
		}
	}
	catch (boost::thread_interrupted const&) {

	}
}

void AciRemote::publishStatusMotorsRcData() {
	asctec_hlp_comm::mav_rcdataPtr rcdata_msg(new asctec_hlp_comm::mav_rcdata);
	asctec_hlp_comm::mav_statusPtr status_msg(new asctec_hlp_comm::mav_status);
	asctec_hlp_comm::MotorSpeedPtr motor_msg(new asctec_hlp_comm::MotorSpeed);
	int status_throttle = 1000 / rc_status_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const throttle_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(status_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);
			// only publish if someone has already subscribed to topics
			if (rcdata_pub_.getNumSubscribers() > 0) {

			}
			if (status_pub_.getNumSubscribers() > 0) {

			}
			if (motor_pub_.getNumSubscribers() > 0) {

			}
		}
	}
	catch (boost::thread_interrupted const&) {

	}
}


} /* namespace AciRemote */
