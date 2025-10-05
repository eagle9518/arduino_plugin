#ifndef ARDUINO_PLUGIN_ARDUINO_COMMS_HPP_
#define ARDUINO_PLUGIN_ARDUINO_COMMS_HPP_

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600; } }

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void send_command(const float *commands, size_t length)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    const uint8_t* p = (uint8_t*)commands;
    // DataBuffer is just an alias for a vector
    LibSerial::DataBuffer buffer(p, p+length*sizeof(float));
    serial_conn_.Write(buffer);
  }

  // TODO Expect array to be static float array and parse accordingly
  // void read_encoder_values(int &val_1, int &val_2)
  // {
  //   std::string response = send_msg("e\r");
  //
  //   std::string delimiter = " ";
  //   size_t start = 0;
  //   size_t end = 0;
  //   std::vector<std::string> tokens;
  //
  //   // split by spaces
  //   while ((end = response.find(delimiter, start)) != std::string::npos) {
  //     tokens.push_back(response.substr(start, end - start));
  //     start = end + delimiter.length();
  //   }
  //   tokens.push_back(response.substr(start));
  //
  //   val_1 = std::atoi(tokens[0].c_str());
  //   val_2 = std::atoi(tokens[1].c_str());
  //   val_3 = std::atoi(tokens[2].c_str());
  //   val_4 = std::atoi(tokens[3].c_str());
  // }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // ARDUINO_PLUGIN_ARDUINO_COMMS_HPP_
