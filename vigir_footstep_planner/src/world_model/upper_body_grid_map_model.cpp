#include <vigir_footstep_planner/world_model/upper_body_grid_map_model.h>

namespace vigir_footstep_planning
{
UpperBodyGridMapModel::UpperBodyGridMapModel(const std::string& name, const ParameterSet& params, ros::NodeHandle& nh, const std::string& topic)
  : GridMapModel(name, params, UPPER_BODY, nh, topic)
{
  // get upper body dimensions
  getUpperBodySize(nh, upper_body_size);
  getUpperBodyOriginShift(nh, upper_body_origin_shift);
}

UpperBodyGridMapModel::UpperBodyGridMapModel(const std::string& name, ros::NodeHandle& nh, const std::string& topic)
  : GridMapModel(name, UPPER_BODY, nh, topic)
{
  // get upper body dimensions
  getUpperBodySize(nh, upper_body_size);
  getUpperBodyOriginShift(nh, upper_body_origin_shift);
}

bool UpperBodyGridMapModel::isAccessible(const State& /*s*/) const
{
  // We can't make any checks with a single foot pose
  return true;
}

bool UpperBodyGridMapModel::isAccessible(const State& next, const State& current) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  if (!occupancy_grid_map)
  {
    ROS_ERROR_THROTTLE(10, "[UpperBodyGridMapModel] No body level grid map available yet.");
    return true;
  }

  if (next.getLeg() == current.getLeg())
  {
    ROS_ERROR_THROTTLE(10, "[UpperBodyGridMapModel] Swing foot can't equal support foot.");
    return false;
  }

  const State& left = next.getLeg() == LEFT ? next : current;
  const State& right = next.getLeg() == RIGHT ? next : current;

  // approximate upper body dimensions
  float x = right.getX() + 0.5 * (left.getX() - right.getX());
  float y = right.getY() + 0.5 * (left.getY() - right.getY());

  float theta = right.getYaw() + 0.5 * (left.getYaw() - right.getYaw());

  // determine shift of polygon based on foot orientation
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);
  float shift_x = cos_theta * upper_body_origin_shift.x - sin_theta * upper_body_origin_shift.y;
  float shift_y = sin_theta * upper_body_origin_shift.x + cos_theta * upper_body_origin_shift.y;

  // shift pose
  x += shift_x;
  y += shift_y;

  return !collision_check(x, y, cos_theta, sin_theta, upper_body_size.x, upper_body_size.y);
}
}
