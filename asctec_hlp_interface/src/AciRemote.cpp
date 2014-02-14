/*
 * AciRemote.cpp
 *
 *  Created on: 14 Feb 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "asctec_hlp_interface/AciRemote.h"

namespace AciRemote {

//extern "C" {
//	void transmit(void* bytes, unsigned short len) {
//
//	}
//}

AciRemote::AciRemote(const std::string& port): SerialComm(port),
		verListRecv_(false), varListRecv_(false), cmdListRecv_(false), parListRecv_(false) {

}

AciRemote::~AciRemote() {

}

void AciRemote::Init(int rate, int heartbeat) {
	boost::mutex::scoped_lock lock(mtx_);
	aciInit();
	ROS_INFO("Asctec ACI initialised");
	aciSetSendDataCallback(&transmit);
	//aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	//aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	//aciSetParamListUpdateFinishedCallback(&paramListUpdateFinished);
	aciSetEngineRate(rate, heartbeat);
}

void AciRemote::transmit(void* bytes, unsigned short len) {
	txCallback(bytes, len);
}

void AciRemote::versions(struct ACI_INFO) {
	boost::mutex::scoped_lock lock(mtx_);
	verListRecv_ = true;
	ROS_INFO("Received versions list from HLP");
}

void AciRemote::varListUpdateFinished() {
	boost::mutex::scoped_lock lock(mtx_);
	varListRecv_ = true;
	ROS_INFO("Received variables list from HLP");
}

void AciRemote::cmdListUpdateFinished() {
	boost::mutex::scoped_lock lock(mtx_);
	cmdListRecv_ = true;
	ROS_INFO("Received commands list from HLP");
}

void AciRemote::paramListUpdateFinished() {
	boost::mutex::scoped_lock lock(mtx_);
	parListRecv_ = true;
	ROS_INFO("Received parameters list from HLP");
}

} /* namespace AciRemote */
