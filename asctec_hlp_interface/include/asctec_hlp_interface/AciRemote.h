/*
 * AciRemote.h
 *
 *  Created on: 14 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#ifndef ACIREMOTE_H_
#define ACIREMOTE_H_

#include "asctec_hlp_interface/SerialComm.h"
#include "asctec_hlp_interface/AsctecSDK3.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "asctec_hlp_comm/HlpCtrlSrv.h"
#include "aci_remote_v100/asctecDefines.h"
#include "aci_remote_v100/asctecCommIntf.h"

namespace AciRemote {

//extern "C" {
//	void transmit(void*, unsigned short);
//}

//void* aci_obj_ptr;

class AciRemote: protected SerialComm {
public:
	//AciRemote(); // default constructor
	AciRemote(ros::NodeHandle&);
	// non-copyable class, hence = delete (c++11)
	//AciRemote(const AciRemote&) = delete;
	//const AciRemote& operator=(const AciRemote&) = delete;
	~AciRemote();

	int init();
	int initRosLayer();

protected:
	void checkVersions(struct ACI_INFO);
	void setupVarPackets();
	void setupCmdPackets();
	void setupParPackets();

private:
	AciRemote(const AciRemote&);
	const AciRemote& operator=(const AciRemote&);

	static void transmit(void*, unsigned short);
	static void versions(struct ACI_INFO);
	static void varListUpdateFinished();
	static void cmdListUpdateFinished();
	static void paramListUpdateFinished();

	void readHandler(const boost::system::error_code&, size_t);
	void throttleEngine();
	void publishImuMagData();
	void publishGpsData();
	void publishStatusMotorsRcData();

	void ctrlTopicCallback(const geometry_msgs::TwistConstPtr&);
	bool ctrlServiceCallback(asctec_hlp_comm::HlpCtrlSrv::Request&,
			asctec_hlp_comm::HlpCtrlSrv::Response&);

	// debug variables
	unsigned short debug1_;
	unsigned short debug2_;
	unsigned short debug3_;

	// variables to store ROS parameters
	std::string frame_id_;
	int imu_rate_;
	int gps_rate_;
	int rc_status_rate_;
	int aci_rate_;
	int aci_heartbeat_;
	int bytes_recv_;
	double ang_vel_variance_;
	double lin_acc_variance_;

	std::string imu_topic_;
	std::string imu_custom_topic_;
	std::string mag_topic_;
	std::string gps_topic_;
	std::string gps_custom_topic_;
	std::string rcdata_topic_;
	std::string status_topic_;
	std::string motor_topic_;
	std::string ctrl_topic_;
	std::string ctrl_srv_name_;

	ros::NodeHandle n_;

	ros::Publisher imu_pub_;
	ros::Publisher imu_custom_pub_;
	ros::Publisher mag_pub_;
	ros::Publisher gps_pub_;
	ros::Publisher gps_custom_pub_;
	ros::Publisher rcdata_pub_;
	ros::Publisher status_pub_;
	ros::Publisher motor_pub_;
	ros::Subscriber ctrl_sub_;
	ros::ServiceServer ctrl_srv_;

	bool versions_match_;
	bool var_list_recv_;
	bool cmd_list_recv_;
	bool par_list_recv_;
	volatile bool must_stop_engine_;
	volatile bool must_stop_pub_;

	boost::mutex mtx_, buf_mtx_, ctrl_mtx_;
	boost::shared_mutex shared_mtx_;
	boost::condition_variable cond_;
	boost::condition_variable_any cond_any_;
	boost::shared_ptr<boost::thread> aci_throttle_thread_;
	boost::shared_ptr<boost::thread> imu_mag_thread_;
	boost::shared_ptr<boost::thread> gps_thread_;
	boost::shared_ptr<boost::thread> rc_status_thread_;

	// Asctec SDK 3.0 data structures
	struct WO_SDK_STRUCT WO_SDK_;
	struct RO_ALL_DATA RO_ALL_Data_;
	//struct RO_RC_DATA RO_RC_Data_;
	struct WO_DIRECT_INDIVIDUAL_MOTOR_CONTROL WO_DIMC_;
	//struct WO_DIRECT_MOTOR_CONTROL WO_DMC_;
	struct WO_CTRL_INPUT WO_CTRL_;
	struct WAYPOINT WO_wpToLL_;

	// Asctec SDK 3.0 variables
	//choose actual waypoint command from WP_CMD_* defines
	unsigned char wpCtrlWpCmd_;
	//send current waypoint command to LLP
	unsigned char wpCtrlWpCmdUpdated_;
	//acknowledge from LL processor that waypoint was accepted
	unsigned char wpCtrlAckTrigger_;
	//check navigation status with WP_NAVSTAT_* defines
	unsigned short wpCtrlNavStatus_;
	//current distance to the current waypoint in dm (=10 cm)
	unsigned short wpCtrlDistToWp_;

	//emergency mode variables
	unsigned char emergencyMode_;
	unsigned char emergencyModeUpdate_;
};

} /* namespace AciRemote */
#endif /* ACIREMOTE_H_ */
