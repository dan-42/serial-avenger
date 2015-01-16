#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <iomanip>
#include <memory>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/steady_timer.hpp>

static const uint32_t DEFAULT_BAUD_RATE = 115200;
static const auto DEFAULT_FLOW_CONTROL = boost::asio::serial_port_base::flow_control::none;
static const auto DEFAULT_STOP_BITS = boost::asio::serial_port_base::stop_bits::one;
static const auto DEFAULT_PARITY = boost::asio::serial_port_base::parity::even;

uint32_t BUFFER_SIZE = 65000;

class Port {

public:
  Port(boost::asio::io_service &io_service, const std::string & device) :
      serial_port_(io_service, device) {

    boost::asio::serial_port_base::baud_rate br(DEFAULT_BAUD_RATE);
    boost::asio::serial_port_base::flow_control fc(DEFAULT_FLOW_CONTROL);
    boost::asio::serial_port_base::stop_bits sb(DEFAULT_STOP_BITS);
    boost::asio::serial_port_base::parity p(DEFAULT_PARITY);

    serial_port_.set_option(br);
    serial_port_.set_option(fc);
    serial_port_.set_option(sb);
    serial_port_.set_option(p);

  }

  void echo() {
    std::cout << "echo()" << std::endl;
    std::shared_ptr<std::vector<uint8_t> > buffer = std::make_shared < std::vector<uint8_t> > (BUFFER_SIZE);

    serial_port_.async_read_some(boost::asio::buffer(*buffer),
        boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void send_test(const std::vector<uint8_t>& test_patern) {

    std::shared_ptr<std::vector<uint8_t> > buffer = std::make_shared < std::vector<uint8_t> > (BUFFER_SIZE);
    serial_port_.async_write_some(boost::asio::buffer(*buffer),
        boost::bind(&Port::handler_send_test, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  }

private:

  void handler_echo_read(std::shared_ptr<std::vector<uint8_t> > buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {
    // std::cout << "handler_echo_read()" << std::endl;
    if (!ec) {
      std::cout << "handler_echo_read() msg:" << ec.message() << std::endl;
      serial_port_.async_write_some(boost::asio::buffer(*buffer, bytes_transferd),
          boost::bind(&Port::handler_echo_write, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    }

    else {
      std::cout << "handler_echo_read() " << ec.message() << std::endl;
    }
  }

  void handler_echo_write(std::shared_ptr<std::vector<uint8_t> > buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {
    // std::cout << "handler_echo_read()" << std::endl;
    if (!ec) {

      std::cout << std::endl;
      for (int i = 0; i < bytes_transferd; i++) {
        int value = (int) (*buffer)[i];
        std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << value;
      }

      buffer.reset();
      buffer = std::make_shared < std::vector<uint8_t> > (BUFFER_SIZE);

      serial_port_.async_read_some(boost::asio::buffer(*buffer),
          boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    } else {
      std::cout << "handler_echo_write() " << ec.message() << std::endl;
    }

  }

  void handler_send_test(std::shared_ptr<std::vector<uint8_t> > send_buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {
    if (!ec) {
      std::shared_ptr<std::vector<uint8_t> > receive_buffer = std::make_shared < std::vector<uint8_t> > (BUFFER_SIZE);

      serial_port_.async_read_some(boost::asio::buffer(*receive_buffer, bytes_transferd),
          boost::bind(&Port::handler_receive_test, this, send_buffer, bytes_transferd, receive_buffer, boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred));

    }
  }

  void handler_receive_test(std::shared_ptr<std::vector<uint8_t> > send_buffer, std::size_t bytes_transfered,
      std::shared_ptr<std::vector<uint8_t> > receive_buffer, const boost::system::error_code &ec, const std::size_t bytes_recived) {
    if (!ec) {
      std::cout << std::endl;
      for (auto i : (*send_buffer)) {
        std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int) i;
      }
      std::cout << std::endl;
      for (auto i : (*receive_buffer)) {
        std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int) i;
      }
      std::cout << std::endl;
      std::cout << std::endl;
    }

  }

  boost::asio::serial_port serial_port_;

}
;

static Port* port;
static std::vector<uint8_t> p;
static boost::asio::io_service io_service;
static boost::asio::steady_timer timer(io_service);

void sendTest(const boost::system::error_code &ec) {
  if (!ec && port != 0) {
    port->send_test(p);
    timer.expires_from_now(std::chrono::seconds(1));
    timer.async_wait(sendTest);
  }

}

int main(int argc, char** argv) {

  if (std::strcmp(argv[1], "echo") == 0 ) {

    std::cout << argv[1] << std::endl;
    std::cout << argv[2] << std::endl;

    std::string port_name_a(argv[2]);
    port = new Port(io_service, port_name_a);

    port->echo();

  } else {
    std::string port_name_a(argv[1]);
    port = new Port(io_service, port_name_a);

    // 0xCAFEBABE
    std::vector<uint8_t> test_pattern;
    test_pattern.push_back(0xCA);
    test_pattern.push_back(0xFE);
    test_pattern.push_back(0xBA);
    test_pattern.push_back(0xBE);

    std::vector<uint8_t> p = test_pattern;
    for (uint32_t i = 0; i < 20; i++) {
      p.insert(p.end(), test_pattern.begin(), test_pattern.end());
    }

    timer.expires_from_now(std::chrono::seconds(1));
    timer.async_wait(sendTest);

  }
  io_service.run();

  return 0;
}
