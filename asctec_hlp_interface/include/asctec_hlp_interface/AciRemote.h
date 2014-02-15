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
	AciRemote(const std::string&);
	// non-copyable class, hence = delete (c++11)
	//AciRemote(const AciRemote&) = delete;
	//const AciRemote& operator=(const AciRemote&) = delete;
	~AciRemote();

	void Init(int, int);

private:
	AciRemote(const AciRemote&);
	const AciRemote& operator=(const AciRemote&);

	static void transmit(void*, unsigned short);

	void versions(struct ACI_INFO);
	void varListUpdateFinished();
	void cmdListUpdateFinished();
	void paramListUpdateFinished();

	bool verListRecv_;
	bool varListRecv_;
	bool cmdListRecv_;
	bool parListRecv_;

	boost::mutex mtx_;
};

} /* namespace AciRemote */
#endif /* ACIREMOTE_H_ */
