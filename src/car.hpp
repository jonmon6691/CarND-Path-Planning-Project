#ifndef _CAR_HPP
#define _CAR_HPP
#include "json.hpp"

class car
{
public:
  double current_s;
  void update_state(nlohmann::json telemetry);
};

#endif /* _CAR_HPP */
