#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <iomanip>
#include <memory>

#include <sys/ioctl.h>
#include <linux/serial.h>
#include <termios.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/steady_timer.hpp>

void sendTest(const boost::system::error_code &ec);

static const uint32_t DEFAULT_BAUD_RATE = 115200;
static const auto DEFAULT_FLOW_CONTROL = boost::asio::serial_port_base::flow_control::none;
static const auto DEFAULT_STOP_BITS = boost::asio::serial_port_base::stop_bits::one;
static const auto DEFAULT_PARITY = boost::asio::serial_port_base::parity::even;

static boost::asio::io_service io_service;
static boost::asio::steady_timer timer(io_service);

struct TestData {
  std::size_t send_size = 0;
  std::size_t rec_size = 0;
  std::vector<uint8_t> send_data;
  std::vector<uint8_t> rec_data;
};

typedef std::shared_ptr<TestData> TestDataPtr;

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


    auto nativeHandler = serial_port_.lowest_layer().native_handle();

    serial_rs485 rs485conf;

    /*
    rs485conf.flags = (!SER_RS485_ENABLED) | (!SER_RS485_RTS_ON_SEND);
    rs485conf.delay_rts_before_send = 0;
    rs485conf.delay_rts_after_send = 0;
    */
    rs485conf.flags = (SER_RS485_ENABLED) | (SER_RS485_RTS_ON_SEND);
    rs485conf.delay_rts_before_send = 1;
    rs485conf.delay_rts_after_send = 1;

    // set rs485 settings on given tty

    if (ioctl(nativeHandler, TIOCSRS485, &rs485conf) < 0) {
      std::cout << "SerialPort ioctl()  ERROR" << std::endl;
    }


    /*
    struct termios uartSettings;
    memset(&uartSettings, 0, sizeof(uartSettings));
    auto ret = tcgetattr(nativeHandler, &uartSettings);

    //uartSettings.c_iflag = 0;
    //uartSettings.c_oflag = 0;
    //uartSettings.c_cflag = CS8 | CREAD | CLOCAL;           // 8n1, see termios.h for more information
    //uartSettings.c_lflag = 0;
    uartSettings.c_cc[VMIN] = 0; //min blocking read. Set to 0, so read is nonblocking.
    uartSettings.c_cc[VTIME] = 1; //readtimeout, divide by 10 for seconds.
    tcsetattr(nativeHandler,TCSANOW,&uartSettings);

    // */

  }

  void echo() {
    std::cout << "echo()" << std::endl;
    std::shared_ptr<std::vector<uint8_t> > buffer = std::make_shared<std::vector<uint8_t> >(1);

    serial_port_.async_read_some(boost::asio::buffer(*buffer, buffer->size()),
        boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void send_test(const std::vector<uint8_t>& pattern) {

    TestDataPtr data = std::make_shared<TestData>();
    data->send_data = std::vector<uint8_t>(pattern);

    serial_port_.async_write_some(boost::asio::buffer(data->send_data, data->send_data.size()),
        boost::bind(&Port::handler_send_test, this, data, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  }

private:

  void handler_echo_read(std::shared_ptr<std::vector<uint8_t> > buffer, const boost::system::error_code &ec, const std::size_t bytes_recived) {
    if (!ec) {
      std::cout << "handler_echo_read() msg:" << ec.message() << "\t bytes_read " << std::to_string(bytes_recived) << "\t";
      for (int i = 0; i < bytes_recived; i++) {
        int value = (int) (*buffer)[i];
        std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << value;
      }
      std::cout << std::endl;

      serial_port_.async_write_some(boost::asio::buffer(*buffer, bytes_recived),
          boost::bind(&Port::handler_echo_write, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    }

    else {
      std::cout << "handler_echo_read() " << ec.message() << std::endl;
    }
  }

  void handler_echo_write(std::shared_ptr<std::vector<uint8_t> > buffer, const boost::system::error_code &ec, const std::size_t bytes_transferd) {

    if (!ec) {

      //buffer.reset();
      //buffer = std::make_shared < std::vector<uint8_t> > (BUFFER_SIZE);
      std::cout << "handler_echo_write() \t written " << std::to_string(bytes_transferd) << std::endl;

      serial_port_.async_read_some(boost::asio::buffer(*buffer, buffer->size()),
          boost::bind(&Port::handler_echo_read, this, buffer, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    } else {
      std::cout << "handler_echo_write() " << ec.message() << std::endl;
    }

  }

  void handler_send_test(TestDataPtr testData, const boost::system::error_code &ec, const std::size_t bytes_transferd) {
    if (!ec) {
      testData->send_size = bytes_transferd;
      std::cout << "handler_send_test(): bytes_send: \t" << std::to_string(bytes_transferd) << " waiting to receive " << std::to_string(bytes_transferd)
          << " bytes" << std::endl;

      global_read_buffer = std::vector<uint8_t>(bytes_transferd);

      ///*
      serial_port_.async_read_some(boost::asio::buffer(global_read_buffer, bytes_transferd),
          boost::bind(&Port::handler_read_all, this, testData, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
      //  */

      /*
       boost::asio::async_read(serial_port_, boost::asio::buffer(*receive_buffer, bytes_transferd),
       boost::bind(&Port::handler_receive_test, this, send_buffer, bytes_transferd, receive_buffer, boost::asio::placeholders::error,
       boost::asio::placeholders::bytes_transferred));
       // */
    }
  }

  void handler_read_all(TestDataPtr testData, const boost::system::error_code &ec, const std::size_t bytes_recived) {

    if (!ec) {
      // std::cout << "handler_read_all(): bytes_recived: \t" << std::to_string(bytes_recived) << std::endl;
      testData->rec_size += bytes_recived;

      for (int i = 0; i < bytes_recived; i++) {
        testData->rec_data.push_back(global_read_buffer[i]);
      }

      if (testData->send_size > testData->rec_size) {
        std::size_t missing = testData->send_size - testData->rec_size;

        global_read_buffer = std::vector<uint8_t>(missing);
        serial_port_.async_read_some(boost::asio::buffer(global_read_buffer, missing),
            boost::bind(&Port::handler_read_all, this, testData, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

      } else {
        handler_receive_test(testData);
      }

    } else {
      std::cout << "handler_read_all() " << ec.message() << std::endl;
    }

  }

  void handler_receive_test(TestDataPtr testData) {

    //std::cout << "handler_receive_test()" << std::endl;
    bool error = false;

    if (testData->send_size != testData->rec_size) {

      std::cout << "snd: \t" << std::to_string(testData->send_size) << std::endl;
      std::cout << "rec: \t" << std::to_string(testData->rec_size) << std::endl;

    }

    for (int i = 0; i < testData->send_size; i++) {
      uint8_t sendByte = testData->send_data[i];
      uint8_t recivedByte = testData->rec_data[i];

      if (sendByte != recivedByte || error) {
        error = true;
        std::cout << "snd: " << std::hex << std::setfill('0') << std::setw(2) << (int) sendByte;
        std::cout << "\trec: " << std::hex << std::setfill('0') << std::setw(2) << (int) recivedByte;
        std::cout << std::endl;
      } else {

      }
    }
    if (!error) {
      std::cout << "OK ";
      std::cout << "snd: " << std::to_string(testData->send_size);
      std::cout << "\t rec: " << std::to_string(testData->rec_size) << std::endl;
      std::cout << std::endl;

      //timer.expires_from_now(std::chrono::microseconds(250));
      //timer.async_wait(sendTest);

      boost::system::error_code ec(boost::system::errc::success, boost::system::system_category());
      sendTest(ec);


    } else {
      std::cout << "ERROR ";
      std::cout << "snd: " << std::to_string(testData->send_size);
      std::cout << "\t rec: " << std::to_string(testData->rec_size) << std::endl;
      std::cout << std::endl;

      std::cout << "PROGRAMM STOPPED! ";
    }

  }

  boost::asio::serial_port serial_port_;

  std::vector<uint8_t> global_read_buffer;

};
static std::vector<uint8_t> basic_pattern;      //6 bytes
static std::vector<uint8_t> test_pattern;
static Port* port;

void sendTest(const boost::system::error_code &ec) {

  //apennd new testpatterm
  for (int i = 0; i < 7; i++) {
    int8_t v = test_pattern[test_pattern.size() - 1];
    test_pattern.push_back((v - 1));
  }

  //test_pattern.insert(test_pattern.end(), basic_pattern.begin(), basic_pattern.end());

  //if to long reset to basic size
  if (test_pattern.size() > 250) {
    test_pattern = basic_pattern;
  }

  if (!ec && port != 0) {
    port->send_test(test_pattern);

  } else {
    std::cout << "sendTest() " << ec.message() << std::endl;
  }

}

int main(int argc, char** argv) {

  if (std::strcmp(argv[1], "echo") == 0) {

    std::string port_name_a(argv[2]);
    port = new Port(io_service, port_name_a);

    port->echo();

  }

  else {
    std::string port_name_a(argv[1]);
    port = new Port(io_service, port_name_a);

    basic_pattern.push_back(0xFF);

    for (int i = 0; i < 99; i++) {
      int8_t v = basic_pattern[basic_pattern.size() - 1];
      basic_pattern.push_back((v - 1));
    }

    test_pattern.insert(test_pattern.end(), basic_pattern.begin(), basic_pattern.end());

    std::cout << "send_test()" << std::endl;
    std::cout << std::endl;
    for (auto i : (test_pattern)) {
      std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int) i;
    }
    std::cout << std::endl;

    timer.expires_from_now(std::chrono::seconds(1));
    timer.async_wait(sendTest);

  }
  io_service.run();

  return 0;
}
