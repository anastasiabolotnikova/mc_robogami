#include "robogami.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{

// This is set by CMake, see CMakeLists.txt
static const std::string ROBOGAMI_DESCRIPTION_PATH = "@ROBOGAMI_DESCRIPTION_PATH@";

} // namespace

namespace mc_robots
{

RobogamiRobotModule::RobogamiRobotModule() : mc_rbdyn::RobotModule(ROBOGAMI_DESCRIPTION_PATH, "robogami")
{
  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  mc_rtc::log::info("Using {} URDF file to initialize Robogami", urdf_path);
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

    // Default joint configuration
  _stance["l1"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//mc_rtc::constants::PI / 4.0};
  _stance["l2"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//{mc_rtc::constants::PI / 4.0};
  _stance["l3"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//{mc_rtc::constants::PI / 4.0};

  _stance["x_l3roty"] = {17.0 * mc_rtc::constants::PI / 18.0}; //{3.054326};//{mc_rtc::constants::PI / 2.0};
  _stance["x_l2roty"] = {17.0 * mc_rtc::constants::PI / 18.0}; //{3.054326};//{mc_rtc::constants::PI / 2.0};
  _stance["x_l1roty"] = {17.0 * mc_rtc::constants::PI / 18.0}; //{3.054326};//{mc_rtc::constants::PI / 2.0};

  _stance["x_l1topTop"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//{mc_rtc::constants::PI / 4.0};
  _stance["x_l2TopMove"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//{mc_rtc::constants::PI / 4.0};
  _stance["x_l3TopMove"] = {5.0 * mc_rtc::constants::PI / 180.0}; //{0.08726646};//{mc_rtc::constants::PI / 4.0};
}

}