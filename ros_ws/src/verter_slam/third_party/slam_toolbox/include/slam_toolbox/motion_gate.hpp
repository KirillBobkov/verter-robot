// Test candidate: keep scans after sufficient translation OR rotation.
#ifndef SLAM_TOOLBOX__MOTION_GATE_HPP_
#define SLAM_TOOLBOX__MOTION_GATE_HPP_

#include <cmath>

namespace slam_toolbox
{
inline bool hasMovedEnough(
  double squared_distance, double heading_delta,
  double minimum_squared_distance, double minimum_heading)
{
  const double heading = std::abs(std::atan2(std::sin(heading_delta), std::cos(heading_delta)));
  return squared_distance >= minimum_squared_distance || heading >= minimum_heading;
}
}  // namespace slam_toolbox
#endif
