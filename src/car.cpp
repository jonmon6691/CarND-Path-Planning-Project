#include "car.hpp"

void car::update_state(nlohmann::json telemetry)
{
    current_s = telemetry["s"];
}