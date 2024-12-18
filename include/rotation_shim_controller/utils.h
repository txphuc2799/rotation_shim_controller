#ifndef ROTATION_SHIM_CONTROLLER__MATH_HELPER_H_
#define ROTATION_SHIM_CONTROLLER__MATH_HELPER_H_

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace rotation_shim_controller
{

/**
 * @brief Clamps a value within a specified range.
 * @param value          The value to be clamped.
 * @param low            The lower bound of the range.
 * @param high           The upper bound of the range.
 * @return const T&      The clamped value within the specified range.
 */
template <typename T>
const T& clamp(const T& value, const T& low, const T& high)
{
    return std::max(low, std::min(value, high));
}

double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

inline bool transformPoseInTargetFrame(
  tf2_ros::Buffer * tf_buffer,
  const geometry_msgs::PoseStamped & input_pose,
  geometry_msgs::PoseStamped & transformed_pose,
  const std::string target_frame,
  const double transform_timeout)
{
  std::string controller_name = "RotationShimController";
  try {
    transformed_pose = tf_buffer->transform(
      input_pose, target_frame,
      ros::Duration(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    ROS_ERROR(
      "%s: "
      "No Transform available Error looking up target frame: %s\n", controller_name.c_str(), ex.what());
  } catch (tf2::ConnectivityException & ex) {
    ROS_ERROR(
      "%s: "
      "Connectivity Error looking up target frame: %s\n", controller_name.c_str(), ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    ROS_ERROR(
      "%s: "
      "Extrapolation Error looking up target frame: %s\n", controller_name.c_str(), ex.what());
  } catch (tf2::TimeoutException & ex) {
    ROS_ERROR(
      "%s: "
      "Transform timeout with tolerance: %.4f", controller_name.c_str(), transform_timeout);
  } catch (tf2::TransformException & ex) {
    ROS_ERROR(
      "%s: " "Failed to transform from %s to %s", controller_name.c_str(),
      input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::Pose & pos1,
  const geometry_msgs::Pose & pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  return std::hypot(dx, dy);
}

inline bool invalidPathSegment(std::vector<geometry_msgs::PoseStamped> &global_path, double length_threshold)
{
  double path_segment_length = 0.0;
  for (size_t i = 0; i < global_path.size() - 1; ++i) {
    path_segment_length += euclidean_distance(global_path[i].pose, global_path[i+1].pose);
    if (path_segment_length >= length_threshold) {
      return true;
    }
  }
  return false;
}



} // namespace rotation_shim_controller

#endif // ROTATION_SHIM_CONTROLLER__MATH_HELPER_H_