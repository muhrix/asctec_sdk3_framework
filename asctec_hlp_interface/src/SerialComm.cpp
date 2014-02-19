/*
 * SerialComm.cpp
 *
 *  Created on: 21 Jan 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include "asctec_hlp_interface/SerialComm.h"

SerialComm::SerialComm(): port_name_("/dev/ttyUSB0"), baud_rate_(230400), open_(false) {
	read_buffer_.assign(SERIAL_PORT_READ_BUF_SIZE, 0);
}

SerialComm::~SerialComm() {
	std::cout << "SerialComm destructor" << std::endl;
	std::cout.flush();
	if (isOpen()) {
		try {
			closePort();
		}
		catch (...) {
			// do not throw exceptions from a destructor
		}
	}
}

int SerialComm::openPort() {
	if (!open_) {
		try {
			port_.reset(new boost::asio::serial_port(io_service_));
			port_->open(port_name_);
		}
		catch (boost::system::system_error::exception& e) {
			ROS_ERROR_STREAM("Could not open serial port " << port_name_ << ". " << e.what());
			return -1;
		}
		ROS_INFO_STREAM("Port " << port_name_ << " open");

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
		ROS_INFO_STREAM("Baud rate set to " << baud_rate_);

		doRead();

		// TODO: do I really need this thread?! Or is this the one which dies according to gdb?
		try {
			io_thread_ = boost::shared_ptr<boost::thread>
				(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_)));
		}
		catch (boost::system::system_error::exception& e) {
			ROS_ERROR_STREAM("Could not create Boost IO thread. " << e.what());
		}

		open_ = true;
	}
	else {
		ROS_WARN_STREAM("Serial port " << port_name_ << " is already open");
	}
	return 0;
}

void SerialComm::closePort() {
	if (!isOpen())
		return;

	open_ = false;
	io_service_.post(boost::bind(&SerialComm::doClose, this));
	io_thread_->join();
	io_service_.reset();
}

bool SerialComm::isOpen() const {
	return open_;
}

void SerialComm::doClose() {
	try {
		port_->cancel();
		port_->close();
	}
	catch (boost::system::system_error& e) {
		ROS_ERROR_STREAM("Could not close serial port " << port_name_ << ". " << e.what());
	}
}

void SerialComm::doWrite(void* bytes, unsigned short len) {
	unsigned char* ucharBuf = static_cast<unsigned char*>(bytes);
	// make a copy of the buffer because it must exist and not change whilst being sent
	// and owner of void* bytes may not guarantee existence nor "const-ness"
	boost::shared_ptr<std::vector<unsigned char> >
			vecBuf(new std::vector<unsigned char>(ucharBuf, ucharBuf + len));

	// since vecBuf is a shared_ptr, passing the ptr to the handler will guarantee
	// it does not get freed once this function returns
	boost::asio::async_write(*port_, boost::asio::buffer(*vecBuf),
			boost::bind(&SerialComm::writeHandler, this, vecBuf,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
}

void SerialComm::doRead() {
	// call async_read_some for the first time so that something is received in the buffer
	port_->async_read_some(boost::asio::buffer(read_buffer_),
			boost::bind(&SerialComm::readHandler, this, boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
}

void SerialComm::writeHandler(boost::shared_ptr<std::vector<unsigned char> > bytesVec,
		const boost::system::error_code& error, size_t bytes_transferred) {
	if ( error || (bytes_transferred != bytesVec->size()) ) {
		ROS_ERROR_STREAM("Async write to serial port " << port_name_ << ". " << error.message());
		doClose();
	}
}





