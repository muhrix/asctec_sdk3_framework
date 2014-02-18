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
#include <boost/asio/buffer.hpp>

#include <string>
#include <vector>

#include <ros/ros.h>

#define SERIAL_PORT_READ_BUF_SIZE 256

class SerialComm {
	typedef boost::shared_ptr<boost::asio::serial_port> SerialPortPtr;

public:
	SerialComm(); // default constructor
	//SerialComm(const std::string&, int);
	// non-copyable class, hence = delete (c++11)
	//SerialComm(const SerialComm&) = delete;
	//const SerialComm& operator=(const SerialComm&) = delete;
	virtual ~SerialComm();

	// methods to be used as callbacks of ACI
	void doWrite(void*, unsigned short);
	//boost::function<void (const unsigned char*, size_t)> receive;

	//boost::shared_ptr<const boost::system::error_code&> errorStatus() const;

	int openPort();
	void closePort();
	bool isOpen() const;

private:
	SerialComm(const SerialComm&);
	const SerialComm& operator=(const SerialComm&);

protected:
	SerialPortPtr port_;
	//boost::asio::serial_port port_;
	boost::asio::io_service io_service_;
	// TODO: do I really need this thread?! Or is this the one which dies according to gdb?
	boost::shared_ptr<boost::thread> io_thread_;
	//boost::shared_ptr<const boost::system::error_code&> io_error_;

	std::string port_name_;
	int baud_rate_;

	//boost::array<unsigned char, SERIAL_PORT_READ_BUF_SIZE> buffer_;
	std::vector<unsigned char> read_buffer_;

	bool open_;

    /**
     * Callback called to start an asynchronous read operation.
     * This callback is called by the io_service in the spawned thread.
     */
    void doRead();

    /**
     * Callback called at the end of the asynchronous operation.
     * This callback is called by the io_service in the spawned thread.
     */
    virtual void readHandler(const boost::system::error_code&, size_t) = 0;

    /**
     * Callback called at the end of an asynchronous write operation,
     * if there is more data to write, restarts a new write operation.
     * This callback is called by the io_service in the spawned thread.
     */
    void writeHandler(boost::shared_ptr<std::vector<unsigned char> >,
    		const boost::system::error_code&, size_t);

    /**
     * Callback to close serial port
     */
    void doClose();
};

#endif /* SERIALCOMM_H_ */
