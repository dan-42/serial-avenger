#include <iostream>
#include <thread>
#include <atomic>
#include <string>
#include <iomanip>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

uint32_t BUFFER_SIZE = 65000;

class Port {

public:
	Port(boost::asio::io_service &io_service, const std::string & device) :
			serial_port_(io_service, device) {

	}

	void echo() {
		std::vector<uint8_t> buffer(BUFFER_SIZE);
		serial_port_.async_read_some(boost::asio::buffer(buffer),
				boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
	}

	void send_test(const std::vector<uint8_t>& test_patern) {

		std::vector<uint8_t> buffer = test_patern;
		serial_port_.async_write_some(boost::asio::buffer(buffer),
				boost::bind(&Port::handler_send_test, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

	}

private:

	void handler_echo_read(std::vector<uint8_t> buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {

		if (!ec) {
			serial_port_.async_write_some(boost::asio::buffer(buffer, bytes_transferd),
					boost::bind(&Port::handler_echo_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

		}
	}

	void handler_echo_write(const boost::system::error_code &ec, const std::size_t bytes_transferd) {

		std::vector<uint8_t> buffer(BUFFER_SIZE);
		serial_port_.async_read_some(boost::asio::buffer(buffer),
				boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

	}

	void handler_send_test(std::vector<uint8_t> send_buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {
		if (!ec) {
			std::vector<uint8_t> receive_buffer(BUFFER_SIZE);
			serial_port_.async_read_some(boost::asio::buffer(receive_buffer, bytes_transferd),
					boost::bind(&Port::handler_receive_test, this, send_buffer, receive_buffer, boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred));

		}
	}

	void handler_receive_test(std::vector<uint8_t> send_buffer, std::vector<uint8_t> receive_buffer, const boost::system::error_code &ec,
			const std::size_t bytes_transferd) {
		if (!ec) {
			std::cout << std::endl;
			for (auto i : send_buffer) {
				std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int) i;
			}
			std::cout << std::endl;
			for (auto i : receive_buffer) {
				std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int) i;
			}
			std::cout << std::endl;
			std::cout << std::endl;
		}

	}

	boost::asio::serial_port serial_port_;

}
;

int main() {

	std::string port_name_a("/dev/ttyUSB0");
	std::string port_name_b("/dev/ttyUSB1");

// 0xCAFEBABE
	std::vector<uint8_t> test_pattern;
	test_pattern.push_back(0xCA);
	test_pattern.push_back(0xFE);
	test_pattern.push_back(0xBA);
	test_pattern.push_back(0xBE);

	boost::asio::io_service io_service;
	auto port_a = Port(io_service, port_name_a);
	auto port_b = Port(io_service, port_name_b);

	port_b.echo();
	port_a.send_test(test_pattern);


//xxx set options

	return 0;
}
