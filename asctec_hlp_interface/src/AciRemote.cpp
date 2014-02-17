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

namespace AciRemote {

//extern "C" {
//	void transmit(void* bytes, unsigned short len) {
//
//	}
//}

void* aci_obj_ptr = NULL;

AciRemote::AciRemote(const std::string& port, int baud, int rate, int hb):
		SerialComm(port, baud), aci_rate_(rate), aci_heartbeat_(hb), bytes_recv_(0),
		versions_match_(false), var_list_recv_(false),
		cmd_list_recv_(false), par_list_recv_(false) {

	aci_obj_ptr = static_cast<void*>(this);
}

AciRemote::~AciRemote() {

}

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
	// lock mutex
	boost::unique_lock<boost::mutex> lock(buf_mtx_);
	quit_ = false;
	wait_ = false;

	lock.unlock();

	cond_.notify_one();

	// request version info and lists of commands, parameters and variables to HLP
	aciCheckVerConf();
	aciGetDeviceCommandsList();
	aciGetDeviceParametersList();
	aciGetDeviceVariablesList();

	return 0;
}

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


	boost::mutex::scoped_lock lock(mtx_);
	var_list_recv_ = true;
}

void AciRemote::setupCmdPackets() {
	ROS_INFO("Received commands list from HLP");
	boost::mutex::scoped_lock lock(mtx_);
	cmd_list_recv_ = true;
}

void AciRemote::setupParPackets() {
	ROS_INFO("Received parameters list from HLP");
	boost::mutex::scoped_lock lock(mtx_);
	par_list_recv_ = true;
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
				// synchronise variables
				aciSynchronizeVars();
			}
		}
	}
	catch (boost::thread_interrupted const&) {

	}
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


} /* namespace AciRemote */
