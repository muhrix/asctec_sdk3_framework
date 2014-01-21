/*
 * SerialComm.h
 *
 *  Created on: 21 Jan 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <string>
#include <vector>

#include <ros/ros.h>

#include "aci_remote_v100/asctecCommIntf.h"

typedef boost::shared_ptr<boost::asio::serial_port> SerialPortPtr;

#define SERIAL_PORT_READ_BUF_SIZE 256

class SerialComm {
public:
	SerialComm();
	virtual ~SerialComm();

	int openPort();
	int openPort(const std::string& port, uint32_t baud);
	void closePort();

private:
	SerialPortPtr port_;
	boost::asio::io_service uart_service_;
	boost::asio::deadline_timer rx_timeout_;
	bool rx_timeout_occurred_;
	boost::thread uart_thread_;

	std::string port_name_;
	uint32_t baud_rate_;

	uint8_t buffer_[SERIAL_PORT_READ_BUF_SIZE];

	int configurePort(uint32_t baud);
};

#endif /* SERIALCOMM_H_ */
