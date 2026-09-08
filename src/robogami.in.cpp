#include "robogami.h"

#include <RBDyn/parsers/urdf.h>

namespace fs = std::filesystem;

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
  double default_leg_angle = 5.0 * mc_rtc::constants::PI / 180.0; // 5 degrees
  double default_sphereY_angle = mc_rtc::constants::PI - default_leg_angle*2.0;

  _stance["l1"] = {default_leg_angle};
  _stance["l2"] = {default_leg_angle};
  _stance["l3"] = {default_leg_angle};
  _stance["x_l1roty"] = {default_sphereY_angle};
  _stance["x_l2roty"] = {default_sphereY_angle};
  _stance["x_l3roty"] = {default_sphereY_angle};
  _stance["l1topBase"] = {default_leg_angle};
  _stance["l2TopMove"] = {default_leg_angle};
  _stance["l3TopMove"] = {default_leg_angle};

  // Min and Max distance constraints for testing
  _minimalSelfCollisions = {{"leg1lowerLeftConner", "leg1topLeftConner", 0.005, 0.009, 0.},
                            {"leg1lowerLeftConner", "leg1topLeftConner", 0.013, 0.001, 0.}};

  // TODO declare _minimalDistanceLimits vector in mc_rbdyn::RobotModule structure

  // Convex collision shapes
  std::string convexPath = path + "/convex/" + name + "/";
  fs::path p(convexPath);
  if(fs::exists(p) && fs::is_directory(p))
  {
    std::vector<fs::path> files;
    std::copy(fs::directory_iterator(p), fs::directory_iterator(), std::back_inserter(files));
    for(const fs::path & file : files)
    {
      size_t off = file.filename().string().rfind("-ch.txt");
      if(off != std::string::npos)
      {
        std::string name = file.filename().string();
        name.replace(off, 7, "");
        _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }
}

}