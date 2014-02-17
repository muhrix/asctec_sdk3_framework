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

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
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
	AciRemote(const std::string&, int, int, int);
	// non-copyable class, hence = delete (c++11)
	//AciRemote(const AciRemote&) = delete;
	//const AciRemote& operator=(const AciRemote&) = delete;
	~AciRemote();

	int Init();

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

	void throttleEngine();
	void readHandler(const boost::system::error_code&, size_t);

	int aci_rate_;
	int aci_heartbeat_;
	int bytes_recv_;

	bool versions_match_;
	bool var_list_recv_;
	bool cmd_list_recv_;
	bool par_list_recv_;

	volatile bool quit_;
	volatile bool wait_;

	boost::mutex mtx_, buf_mtx_;
	boost::condition_variable cond_;
	boost::shared_ptr<boost::thread> aci_throttle_thread_;
};

} /* namespace AciRemote */
#endif /* ACIREMOTE_H_ */
