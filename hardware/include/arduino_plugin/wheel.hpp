#ifndef ARDUINO_PLUGIN_WHEEL_HPP_
#define ARDUINO_PLUGIN_WHEEL_HPP_

#include <string>
#include <cmath>


class Wheel
{
    public:
      std::string name = "";
      int enc = 0;
      double cmd = 0;
      double pos = 0;
      double vel = 0;
      double rads_per_count = 0;

      Wheel() = default;

      Wheel(const std::string &wheel_name, int counts_per_rev)
      {
        setup(wheel_name, counts_per_rev);
      }

      // Here -1 indicates this is an open-loop control and we are to
      // provide "dummy" encoder values in the form of the sent command
      void setup(const std::string &wheel_name, int counts_per_rev=-1)
      {
        name = wheel_name;
        rads_per_count = (2*M_PI)/counts_per_rev;
      }

      double calc_enc_angle()
      {
        return enc * rads_per_count;
      }
};


#endif // ARDUINO_PLUGIN_WHEEL_HPP_
