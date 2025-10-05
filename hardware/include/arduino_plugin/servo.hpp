#ifndef ARDUINO_PLUGIN_SERVO_HPP_
#define ARDUINO_PLUGIN_SERVO_HPP_

#include <string>
#include <cmath>


class Servo
{
    public:
      std::string name = "";
      double cmd = 0;
      double pos = 0;

      Servo() = default;

      Servo(const std::string &servo_name)
      {
        setup(servo_name);
      }

      void setup(const std::string &servo_name)
      {
        name = servo_name;
      }
};


#endif // ARDUINO_PLUGIN_SERVO_HPP_

