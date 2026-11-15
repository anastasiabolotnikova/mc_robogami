#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>

#include <algorithm>
#include <utility>

namespace mc_robots
{
    struct MC_ROBOTS_DLLAPI RobogamiRobotModule : public mc_rbdyn::RobotModule
{
public:
  RobogamiRobotModule(const std::string & name = "robogami");

  // Baseline "robogami" default leg angle, in degrees
  static constexpr double defaultLegAngleDeg = 35.0; // deg

  // Top hexagon height above the bottom hexagon for default stance
  static constexpr double stanceTopHeight = 0.093; // m

  // Min distance constraint parameters (self-collision avoidance)
  static constexpr double iDistMin = 0.01; // 1cm
  static constexpr double sDistMin = 0.001; // 0.1cm
  // Max distance constraint: spherical joint security distance
  static constexpr double maxDistTipFraction = 0.95; // 95%
  static constexpr double maxDistInteractionMargin = 0.005; // 0.5cm

  // Detailed models: top plate vs base plate self-collision avoidance, offset by the motor box height (from the URDF)
  // so that the top plate stays clear of the motor boxes standing on the base plate
  static constexpr double iDistTopMotor = 0.005; // 0.5cm above the motor boxes
  static constexpr double sDistTopMotor = 0.003; // 0.3cm above the motor boxes

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
    names = {"robogami", "robogami_small", "robogami_medium", "robogami_big"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    static const std::vector<std::string> valid = {"robogami", "robogami_small", "robogami_medium",
                                                    "robogami_big"};
    if(std::find(valid.begin(), valid.end(), n) != valid.end())
    {
      return new mc_robots::RobogamiRobotModule(n);
    }
    else
    {
      mc_rtc::log::error("RobogamiRobotModule cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
