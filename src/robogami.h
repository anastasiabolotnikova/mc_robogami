#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>

#include <utility>

namespace mc_robots
{
    struct MC_ROBOTS_DLLAPI RobogamiRobotModule : public mc_rbdyn::RobotModule
{
public:
  RobogamiRobotModule();

  // Default leg angle, in degrees
  static constexpr double defaultLegAngleDeg = 10.0; // deg

  // Min distance constraint parameters (self-collision avoidance)
  static constexpr double iDistMin = 0.01; // 1cm
  static constexpr double sDistMin = 0.001; // 1mm
  // Max distance constraint parameters
  static constexpr double iDistMax = 0.015; // 1.5cm
  static constexpr double sDistMax = 0.02; // 2cm

  // Body pairs (lower corner, top corner) for the distance constraints
  static inline const std::vector<std::pair<std::string, std::string>> distanceLimitCorners = {
      {"leg1lowerLeftConner", "leg1topLeftConner"},
      {"leg1lowerRightConner", "leg1topRightConner"},
      {"leg2lowerLeftConner", "leg2topLeftConner"},
      {"leg2lowerRightConner", "leg2topRightConner"},
      {"leg3lowerLeftConner", "leg3topLeftConner"},
      {"leg3lowerRightConner", "leg3topRightConner"},
  };
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"robogami"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    std::string seqName = n.substr(0, n.size()-1);

    if(n == "robogami")
    {
      return new mc_robots::RobogamiRobotModule();
    }
    else
    {
      mc_rtc::log::error("RobogamiRobotModule cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
