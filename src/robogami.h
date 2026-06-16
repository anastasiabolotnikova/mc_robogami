#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>


namespace mc_robots
{
    struct MC_ROBOTS_DLLAPI RobogamiRobotModule : public mc_rbdyn::RobotModule
{
public:
  RobogamiRobotModule();
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
