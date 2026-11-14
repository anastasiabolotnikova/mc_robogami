#include "robogami.h"

#include <RBDyn/parsers/urdf.h>

namespace fs = std::filesystem;

namespace
{

static const std::string ROBOGAMI_DESCRIPTION_PATH = "@ROBOGAMI_DESCRIPTION_PATH@";

} // namespace

namespace mc_robots
{

namespace
{

/** Vertical size (base z) of the motor1 box, from its inertia (uniform box) and its pose in the base; 0 if absent */
double motorBoxHeight(const rbd::MultiBody & mb)
{
  if(!mb.jointIndexByName().count("motor1_joint")) { return 0.0; }
  const auto & body = mb.body(mb.bodyIndexByName("motor1"));
  double m = body.inertia().mass();
  if(m <= 0) { return 0.0; }
  // Inertia at the center of mass (the box center), in the link frame
  Eigen::Vector3d c = body.inertia().momentum() / m;
  Eigen::Matrix3d I = body.inertia().inertia() - m * (c.squaredNorm() * Eigen::Matrix3d::Identity() - c * c.transpose());
  // Box sizes along the link axes: I_xx = m (y^2 + z^2) / 12, ...
  Eigen::Vector3d d(std::sqrt(std::max(0.0, 6.0 / m * (I(1, 1) + I(2, 2) - I(0, 0)))),
                    std::sqrt(std::max(0.0, 6.0 / m * (I(0, 0) + I(2, 2) - I(1, 1)))),
                    std::sqrt(std::max(0.0, 6.0 / m * (I(0, 0) + I(1, 1) - I(2, 2)))));
  // Extent along the base z axis: sva rotations map parent to child coordinates, so the box axes in the base frame
  // are the rows of E and their z components the last column
  Eigen::Matrix3d E = mb.transform(mb.jointIndexByName("motor1_joint")).rotation();
  return E.col(2).cwiseAbs().dot(d);
}

} // namespace

RobogamiRobotModule::RobogamiRobotModule(const std::string & name) : mc_rbdyn::RobotModule(ROBOGAMI_DESCRIPTION_PATH, name)
{
  bool fixed = false;
  mc_rtc::log::info("Using {} URDF file to initialize Robogami", urdf_path);
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

  // Default joint configuration
  if(name == "robogami")
  {
    double default_leg_angle = defaultLegAngleDeg * mc_rtc::constants::PI / 180.0;
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
  }
  else
  {
    // Default stance from the top hexagon height
    const auto & p0 = mb.transform(mb.jointIndexByName("l1")).translation();
    const auto & o = mb.transform(mb.jointIndexByName("leg1_joint")).translation();
    double L = mb.transform(mb.jointIndexByName("l1topBase")).translation().norm();
    double top_r = mb.transform(mb.jointIndexByName("topCenter")).translation().norm();
    const auto & tp = mb.transform(mb.jointIndexByName("leg1TopPlate_joint")).translation();
    double d = top_r - std::hypot(p0.x(), p0.y());
    double H = stanceTopHeight + tp.z() - p0.z();
    double D = std::hypot(d, H);
    double arm = L - o.x();
    double R = std::hypot(arm, o.z());
    double cos_g = (D * D + R * R - L * L) / (2.0 * D * R);
    if(std::abs(cos_g) > 1.0)
    {
      mc_rtc::log::error_and_throw("[{}] stanceTopHeight {} m is out of reach", name, stanceTopHeight);
    }
    double leg_angle = std::atan2(H, d) - std::acos(cos_g) - std::atan2(o.z(), arm);
    double b = std::atan2(H - arm * std::sin(leg_angle) - o.z() * std::cos(leg_angle),
                          d - arm * std::cos(leg_angle) + o.z() * std::sin(leg_angle));
    double sphereY_angle = b - leg_angle;
    double top_base_angle = mc_rtc::constants::PI - b;

    _stance["l1"] = {leg_angle};
    _stance["l2"] = {leg_angle};
    _stance["l3"] = {leg_angle};
    _stance["x_l1roty"] = {sphereY_angle};
    _stance["x_l2roty"] = {sphereY_angle};
    _stance["x_l3roty"] = {sphereY_angle};
    _stance["l1topBase"] = {top_base_angle};
    _stance["l2TopMove"] = {0.0};
    _stance["l3TopMove"] = {0.0};
  }

  // Max distance limits from the leg tip geometry
  const auto & corner = mb.transform(mb.jointIndexByName("leg1lowerLeftConner_joint")).translation();
  const auto & tip = mb.transform(mb.jointIndexByName("x_l1rotx")).translation();
  double tipLength = std::abs(corner.x() - tip.x()); // leg_length - leg_side_length
  double sDistMax = 2.0 * tipLength * maxDistTipFraction;
  double iDistMax = sDistMax - maxDistInteractionMargin;

  std::vector<mc_rbdyn::DistanceLimit> selfCollisionAvoidanceLimits;
  std::vector<mc_rbdyn::DistanceLimit> maxDistanceLimits;
  for(const auto & corners : distanceLimitCorners)
  {
    selfCollisionAvoidanceLimits.push_back({corners.first, corners.second, iDistMin, sDistMin, 0.});
    maxDistanceLimits.push_back({corners.first, corners.second, iDistMax, sDistMax, 0.});
  }
  std::vector<mc_rbdyn::DistanceLimit> distanceLimits = selfCollisionAvoidanceLimits;
  distanceLimits.insert(distanceLimits.end(), maxDistanceLimits.begin(), maxDistanceLimits.end());

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

  // Detailed models: keep the top plate above the motor boxes with one top/base distance limit, offset by the motor
  // box height. The boxes stand on the base plate; their size comes from the motor link's inertia (a uniform box).
  if(name != "robogami")
  {
    double h = motorBoxHeight(mb);
    if(h > 0 && _convexHull.count("top") && _convexHull.count("base"))
    {
      distanceLimits.push_back({"top", "base", h + iDistTopMotor, h + sDistTopMotor, 0.});
      mc_rtc::log::info("[{}] top/base self-collision: motor box height {:.1f} mm, security distance {:.1f} mm", name,
                        h * 1000.0, (h + sDistTopMotor) * 1000.0);
    }
    else
    {
      mc_rtc::log::warning("[{}] no motor box or no top/base convex hull, top plate self-collision not avoided", name);
    }
  }
  essentialDistanceLimits(distanceLimits);
}

}