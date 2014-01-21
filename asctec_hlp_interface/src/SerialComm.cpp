/*
 * SerialComm.cpp
 *
 *  Created on: 21 Jan 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "asctec_hlp_interface/SerialComm.h"

SerialComm::SerialComm(): rx_timeout_(uart_service_), port_name_("/dev/ttyUSB0"),
	baud_rate_(57600) {

}

SerialComm::~SerialComm() {

}

int SerialComm::openPort() {
	return openPort(port_name_, baud_rate_);
}

int SerialComm::openPort(const std::string& port, uint32_t baud) {
	port_name_ = port;

	try {
		port_.reset(new boost::asio::serial_port(uart_service_));
		port_->open(port_name_);
	}
	catch (boost::system::system_error::exception& e) {
		ROS_ERROR_STREAM("Could not open serial port " << port_name_ << ". " << e.what());
		return -1;
	}

	if (configurePort(baud) < 0) {
		return -1;
	}
	return 0;
}

int SerialComm::configurePort(uint32_t baud) {
	/*
	int32_t baudrates[] = {9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
	uint32_t best_baudrate = 57600;
	uint32_t min_diff = 1e6;

	for (uint32_t i = 0; i < sizeof(baudrates) / sizeof(uint32_t); ++i) {
		uint32_t diff = std::abs(baudrates[i] - baud);
		if (diff < min_diff) {
			min_diff = diff;
			best_baudrate = baudrates[i];
		}
	}

	if (best_baudrate != baud) {
		ROS_WARN_STREAM("Unsupported baud rate. Choosing closest supported value: " << best_baudrate);
	}
	baud_rate_ = best_baudrate;
	*/
	if (baud != 57600) {
		ROS_WARN_STREAM("Baud rate is currently hardcoded to 57600 and cannot be changed.");
	}
	baud_rate_ = 57600;

	try {
		port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
		port_->set_option(boost::asio::serial_port_base::character_size(8));
		port_->set_option(boost::asio::serial_port_base::stop_bits(
				boost::asio::serial_port_base::stop_bits::one));
		port_->set_option(boost::asio::serial_port_base::parity(
				boost::asio::serial_port_base::parity::none));
	}
	catch (boost::system::system_error::exception& e) {
		ROS_ERROR_STREAM("Could not configure serial port " << port_name_ << ". " << e.what());
		return -1;
	}
	return 0;
}

void SerialComm::closePort() {
	uart_service_.post(boost::bind(&boost::asio::deadline_timer::cancel, &rx_timeout_));
	uart_service_.post(boost::bind(&boost::asio::serial_port::close, port_));
	uart_thread_.join();
}
