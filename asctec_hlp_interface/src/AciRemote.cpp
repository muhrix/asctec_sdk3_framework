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
#include "asctec_hlp_comm/mav_hlp_status.h"
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
		cmd_list_recv_(false), par_list_recv_(false),
		must_stop_engine_(false), must_stop_pub_(false) {

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
	n_.param("stddev_angular_velocity", ang_vel_variance_, 0.013); // taken from experiments
	n_.param("stddev_linear_acceleration", lin_acc_variance_, 0.083); // taken from experiments
	ang_vel_variance_ *= ang_vel_variance_;
	lin_acc_variance_ *= lin_acc_variance_;

	// fetch topic names from ROS parameter server
	n_.param("imu_topic", imu_topic_, std::string("imu"));
	n_.param("imu_custom_topic", imu_custom_topic_, std::string("imu_custom"));
	n_.param("mag_topic", mag_topic_, std::string("mag"));
	n_.param("gps_topic", gps_topic_, std::string("gps"));
	n_.param("gps_custom_topic", gps_custom_topic_, std::string("gps_custom"));
	n_.param("rcdata_topic", rcdata_topic_, std::string("rcdata"));
	n_.param("status_topic", status_topic_, std::string("status"));
	n_.param("motor_speed_topic", motor_topic_, std::string("motor_speed"));

	// TODO: Initialise Asctec SDK3 Command data structures before enabling serial switch
}

AciRemote::~AciRemote() {
	// first of all, close serial port, otherwise pure virtual method would be called
	closePort();
	// interrupt all running threads and wait for them to return
	{
		boost::upgrade_lock<boost::shared_mutex> up_lock(shared_mtx_);
		boost::upgrade_to_unique_lock<boost::shared_mutex> un_lock(up_lock);
		must_stop_pub_ = true;
	}
	if (imu_mag_thread_.get() != NULL)
		imu_mag_thread_->join();
	if (gps_thread_.get() != NULL)
		gps_thread_->join();
	if (rc_status_thread_.get() != NULL)
		rc_status_thread_->join();

	boost::unique_lock<boost::mutex> u_lock(buf_mtx_);
	must_stop_engine_ = true;
	u_lock.unlock();
	if (aci_throttle_thread_.get() != NULL)
		aci_throttle_thread_->join();

	// at this point, ros::spin() will not be called anymore and thus ROS_INFO will not work
	//ROS_INFO("All threads have shutdown");

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
			status_pub_ = n_.advertise<asctec_hlp_comm::mav_hlp_status>(status_topic_, 1);
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

			cond_any_.notify_all();

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
				// check whether thread should terminate (::interrupt() appears to have no effect)
				if (must_stop_engine_)
					return;

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
		ROS_INFO("throttleEngine() thread interrupted");
	}
}

void AciRemote::publishImuMagData() {
	sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
	asctec_hlp_comm::mav_imuPtr imu_custom_msg(new asctec_hlp_comm::mav_imu);
	geometry_msgs::Vector3StampedPtr mag_msg(new geometry_msgs::Vector3Stamped);
	imu_msg->header.frame_id = frame_id_;
	imu_custom_msg->header.frame_id = frame_id_;
	mag_msg->header.frame_id = frame_id_;
	int imu_throttle = 1000 / imu_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const imu_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(imu_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);

			if (cond_any_.timed_wait(s_lock, imu_timeout) == false) {
				// check whether thread should terminate (::interrupt() appears to have no effect)
				if (must_stop_pub_)
					return;
				// TODO: implement flag to put thread into idle mode to save computational resources

				ros::Time time_stamp(ros::Time::now());
				double roll = helper::asctecAttitudeToSI(RO_ALL_Data_.angle_roll);
				double pitch = helper::asctecAttitudeToSI(RO_ALL_Data_.angle_pitch);
				double yaw = helper::asctecAttitudeToSI(RO_ALL_Data_.angle_yaw);
				if (yaw> M_PI) {
					yaw -= 2.0 * M_PI;
				}
				geometry_msgs::Quaternion q;
				helper::angle2quaternion(roll, pitch, yaw, &q.w, &q.x, &q.y, &q.z);

				// only publish if someone has already subscribed to topics
				if (imu_pub_.getNumSubscribers() > 0) {
					imu_msg->header.stamp = time_stamp;
					imu_msg->header.seq = seq;
					imu_msg->linear_acceleration.x = helper::asctecAccToSI(RO_ALL_Data_.acc_x);
					imu_msg->linear_acceleration.y = helper::asctecAccToSI(RO_ALL_Data_.acc_y);
					imu_msg->linear_acceleration.z = helper::asctecAccToSI(RO_ALL_Data_.acc_z);
					imu_msg->angular_velocity.x = helper::asctecAccToSI(RO_ALL_Data_.angle_roll);
					imu_msg->angular_velocity.y = helper::asctecAccToSI(RO_ALL_Data_.angle_pitch);
					imu_msg->angular_velocity.z = helper::asctecAccToSI(RO_ALL_Data_.angle_yaw);
					imu_msg->orientation = q;
					helper::setDiagonalCovariance(imu_msg->angular_velocity_covariance,
							ang_vel_variance_);
					helper::setDiagonalCovariance(imu_msg->linear_acceleration_covariance,
							lin_acc_variance_);
					imu_pub_.publish(imu_msg);
				}
				if (imu_custom_pub_.getNumSubscribers() > 0) {
					double height = static_cast<double>(RO_ALL_Data_.fusion_height) * 0.001;
					double dheight = static_cast<double>(RO_ALL_Data_.fusion_dheight) * 0.001;
					imu_custom_msg->header.stamp = time_stamp;
					imu_custom_msg->header.seq = seq;
					imu_custom_msg->acceleration.x = helper::asctecAccToSI(RO_ALL_Data_.acc_x);
					imu_custom_msg->acceleration.y = helper::asctecAccToSI(RO_ALL_Data_.acc_y);
					imu_custom_msg->acceleration.z = helper::asctecAccToSI(RO_ALL_Data_.acc_z);
					imu_custom_msg->angular_velocity.x =
							helper::asctecAccToSI(RO_ALL_Data_.angle_roll);
					imu_custom_msg->angular_velocity.y =
							helper::asctecAccToSI(RO_ALL_Data_.angle_pitch);
					imu_custom_msg->angular_velocity.z =
							helper::asctecAccToSI(RO_ALL_Data_.angle_yaw);
					imu_custom_msg->height = height;
					imu_custom_msg->differential_height = dheight;
					imu_custom_msg->orientation = q;
					imu_custom_pub_.publish(imu_custom_msg);
				}
				if (mag_pub_.getNumSubscribers() > 0) {
					mag_msg->header.stamp = time_stamp;
					mag_msg->header.seq = seq;
					mag_msg->vector.x = static_cast<double>(RO_ALL_Data_.Hx);
					mag_msg->vector.y = static_cast<double>(RO_ALL_Data_.Hy);
					mag_msg->vector.z = static_cast<double>(RO_ALL_Data_.Hz);
					mag_pub_.publish(mag_msg);
				}
			}
			++seq;
		}
	}
	catch (boost::thread_interrupted const&) {
		ROS_INFO("publishImuMagData() thread interrupted");
	}
}

void AciRemote::publishGpsData() {
	sensor_msgs::NavSatFixPtr gps_msg(new sensor_msgs::NavSatFix);
	asctec_hlp_comm::GpsCustomPtr gps_custom_msg(new asctec_hlp_comm::GpsCustom);
	gps_msg->header.frame_id = frame_id_;
	gps_custom_msg->header.frame_id = frame_id_;
	int gps_throttle = 1000 / gps_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const gps_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(gps_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);

			if (cond_any_.timed_wait(s_lock, gps_timeout) == false) {
				// check whether thread should terminate (::interrupt() appears to have no effect)
				if (must_stop_pub_)
					return;
				// TODO: implement flag to put thread into idle mode to save computational resources

				ros::Time time_stamp(ros::Time::now());
				// TODO: check covariance
				double var_h, var_v;
				var_h = static_cast<double>(RO_ALL_Data_.GPS_position_accuracy) * 1.0e-3 / 3.0;
				var_v = static_cast<double>(RO_ALL_Data_.GPS_height_accuracy) * 1.0e-3 / 3.0;
				var_h *= var_h;
				var_v *= var_v;
				// only publish if someone has already subscribed to topics
				if (gps_pub_.getNumSubscribers() > 0) {
					gps_msg->header.stamp = time_stamp;
					gps_msg->header.seq = seq;
					gps_msg->latitude = static_cast<double>(RO_ALL_Data_.GPS_latitude) * 1.0e-7;
					gps_msg->longitude = static_cast<double>(RO_ALL_Data_.GPS_longitude) * 1.0e-7;
					gps_msg->altitude = static_cast<double>(RO_ALL_Data_.GPS_height) * 1.0e-3;
					gps_msg->position_covariance[0] = var_h;
					gps_msg->position_covariance[4] = var_h;
					gps_msg->position_covariance[8] = var_v;
					gps_msg->position_covariance_type =
							sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

					gps_msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
					// bit 0: GPS lock
					if (RO_ALL_Data_.GPS_status & 0x01)
						gps_msg->status.status =
								sensor_msgs::NavSatStatus::STATUS_FIX;
					else
						gps_msg->status.status =
								sensor_msgs::NavSatStatus::STATUS_NO_FIX;
					gps_pub_.publish(gps_msg);
				}
				if (gps_custom_pub_.getNumSubscribers() > 0) {
					gps_custom_msg->header.stamp = time_stamp;
					gps_custom_msg->header.seq = seq;
					gps_custom_msg->latitude =
							static_cast<double>(RO_ALL_Data_.fusion_latitude) * 1.0e-7;
					gps_custom_msg->longitude =
							static_cast<double>(RO_ALL_Data_.fusion_longitude) * 1.0e-7;
					gps_custom_msg->altitude =
							static_cast<double>(RO_ALL_Data_.GPS_height) * 1.0e-7;
					gps_custom_msg->position_covariance[0] = var_h;
					gps_custom_msg->position_covariance[4] = var_h;
					gps_custom_msg->position_covariance[8] = var_v;
					gps_custom_msg->position_covariance_type =
							sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
					gps_custom_msg->velocity_x =
							static_cast<double>(RO_ALL_Data_.GPS_speed_x) * 1.0e-3;
					gps_custom_msg->velocity_y =
							static_cast<double>(RO_ALL_Data_.GPS_speed_y) * 1.0e-3;
					gps_custom_msg->pressure_height =
							static_cast<double>(RO_ALL_Data_.fusion_height) * 1.0e-3;
					// TODO: check covariance
					double var_vel =
							static_cast<double>(RO_ALL_Data_.GPS_speed_accuracy) * 1.0e-3 / 3.0;
					var_vel *= var_vel;
					gps_custom_msg->velocity_covariance[0] = var_vel;
					gps_custom_msg->velocity_covariance[3] = var_vel;

					gps_custom_msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
					// bit 0: GPS lock
					if (RO_ALL_Data_.GPS_status & 0x01)
						gps_custom_msg->status.status =
								sensor_msgs::NavSatStatus::STATUS_FIX;
					else
						gps_custom_msg->status.status =
								sensor_msgs::NavSatStatus::STATUS_NO_FIX;
					gps_custom_pub_.publish(gps_custom_msg);
				}
			}
			++seq;
		}
	}
	catch (boost::thread_interrupted const&) {
		ROS_INFO("publishGpsData() thread interrupted");
	}
}

void AciRemote::publishStatusMotorsRcData() {
	asctec_hlp_comm::mav_rcdataPtr rcdata_msg(new asctec_hlp_comm::mav_rcdata);
	asctec_hlp_comm::mav_hlp_statusPtr status_msg(new asctec_hlp_comm::mav_hlp_status);
	asctec_hlp_comm::MotorSpeedPtr motor_msg(new asctec_hlp_comm::MotorSpeed);
	rcdata_msg->header.frame_id = frame_id_;
	status_msg->header.frame_id = frame_id_;
	motor_msg->header.frame_id = frame_id_;
	int status_throttle = 1000 / rc_status_rate_;
	static int seq = 0;
	try {
		for (;;) {
			boost::system_time const status_timeout =
					boost::get_system_time() + boost::posix_time::milliseconds(status_throttle);
			// acquire multiple reader shared lock
			boost::shared_lock<boost::shared_mutex> s_lock(shared_mtx_);

			if (cond_any_.timed_wait(s_lock, status_timeout) == false) {
				// check whether thread should terminate (::interrupt() appears to have no effect)
				if (must_stop_pub_)
					return;
				// TODO: implement flag to put thread into idle mode to save computational resources

				ros::Time time_stamp(ros::Time::now());
				// only publish if someone has already subscribed to topics
				if (rcdata_pub_.getNumSubscribers() > 0) {
					rcdata_msg->header.stamp = time_stamp;
					rcdata_msg->header.seq = seq;
					for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
						rcdata_msg->channel[i] = RO_ALL_Data_.channel[i];
					}
					rcdata_pub_.publish(rcdata_msg);
				}
				if (status_pub_.getNumSubscribers() > 0) {
					status_msg->header.stamp = time_stamp;
					status_msg->header.seq = seq;

					status_msg->UAV_status = RO_ALL_Data_.UAV_status;

					if ((RO_ALL_Data_.UAV_status & 0x0F) == HLP_FLIGHTMODE_ATTITUDE)
						status_msg->flight_mode = "Manual";
					else if ((RO_ALL_Data_.UAV_status & 0x0F) == HLP_FLIGHTMODE_HEIGHT)
						status_msg->flight_mode = "Height";
					else if ((RO_ALL_Data_.UAV_status & 0x0F) == HLP_FLIGHTMODE_GPS)
						status_msg->flight_mode = "GPS";

					status_msg->flight_time = static_cast<float>(RO_ALL_Data_.flight_time);
					status_msg->battery_voltage =
							static_cast<float>(RO_ALL_Data_.battery_voltage) * 0.001;
					status_msg->cpu_load = static_cast<float>(RO_ALL_Data_.HL_cpu_load) * 0.001;
					status_msg->up_time = static_cast<float>(RO_ALL_Data_.HL_up_time) * 0.001;
					status_msg->serial_interface_enabled =
							RO_ALL_Data_.UAV_status & SERIAL_INTERFACE_ENABLED;
					status_msg->serial_interface_active =
							RO_ALL_Data_.UAV_status & SERIAL_INTERFACE_ACTIVE;

					bool motor_status = false;
					for (int i = 0; i < NUM_MOTORS; ++i) {
						if (RO_ALL_Data_.motor_rpm[i] > 0) {
							motor_status = true;
							break;
						}
					}
					status_msg->motor_status = motor_status;

					// bit 0: GPS lock
					if (RO_ALL_Data_.GPS_status & 0x01)
						status_msg->gps_status = "GPS fix";
					else
						status_msg->gps_status = "GPS no fix";

					status_msg->gps_num_satellites = RO_ALL_Data_.GPS_sat_num;

					status_pub_.publish(status_msg);
				}
				if (motor_pub_.getNumSubscribers() > 0) {
					motor_msg->header.stamp = time_stamp;
					motor_msg->header.seq = seq;
					for (int i = 0; i < NUM_MOTORS; ++i) {
						motor_msg->motor_speed[i] = RO_ALL_Data_.motor_rpm[i];
					}
					motor_pub_.publish(motor_msg);
				}
				++seq;
			}
		}
	}
	catch (boost::thread_interrupted const&) {
		ROS_INFO("publishStatusMotorsRcData() thread interrupted");
	}
}


} /* namespace AciRemote */
