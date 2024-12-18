#include <rotation_shim_controller/rotation_shim_controller.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rotation_shim_controller::RotationShimController, nav_core::BaseLocalPlanner)

namespace rotation_shim_controller
{
RotationShimController::RotationShimController():
  lp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
  controller_(nullptr),
  initialized_(false),
  path_updated_(false){}

RotationShimController::~RotationShimController()
{
  controller_.reset();

  if (dynamic_srv_) {
    delete dynamic_srv_;
    dynamic_srv_ = nullptr;  // Optional: Helps to avoid dangling pointer issues.
  }
}

void RotationShimController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_){
    ros::NodeHandle nh("~/" + name);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    has_new_goal_ = true;
    goal_changed_ = true;

    // Initialize parameters
    initParams(nh);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(odom_topic_);

    try{
      controller_ = lp_loader_.createUniqueInstance(primary_controller_);
      std::size_t pos = primary_controller_.find_last_of("/");
      std::string primary_controller_name = primary_controller_.substr(pos + 1);
      controller_->initialize(name + "/" + primary_controller_name, tf, costmap_ros);
      ROS_INFO("Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller_.c_str());
    }
    catch(const pluginlib::PluginlibException &ex){
      ROS_DEBUG("%s: Failed to create internal controller for rotation shimming. Exception: %s", plugin_name_.c_str(), ex.what());
    }

    // initialize collision checker and set costmap
    collision_checker_ = std::make_unique<
      FootprintCollisionChecker<costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());

    // Set up parameter reconfigure
    dynamic_srv_ = new ParamterConfigServer(nh);
    CallbackType cb = boost::bind(&RotationShimController::reconfigureCB, this, _1, _2);
    dynamic_srv_->setCallback(cb);

    ROS_INFO("Initialized %s plugin", plugin_name_.c_str());
    initialized_ = true;
  }
}

void RotationShimController::initParams(ros::NodeHandle& nh)
{
  nh.param("primary_controller", primary_controller_, std::string("teb_local_planner/TebLocalPlannerROS"));
  nh.param("odom_topic", odom_topic_, std::string("odom"));
  nh.param("controller_frequency", controller_frequency_, 15.0);
  nh.param("forward_sampling_distance", forward_sampling_distance_, 0.5);
  nh.param("angular_dist_threshold", angular_dist_threshold_, 0.55);
  nh.param("angle_threshold", angle_threshold_, 0.35);
  nh.param("max_angular_vel", max_angular_vel_, 0.4);
  nh.param("max_angular_accel", max_angular_accel_, 0.7);
  nh.param("max_angular_deccel", max_angular_deccel_, 0.3);
  nh.param("min_angular_vel", min_angular_vel_, 0.1);
  nh.param("transform_tolerance", transform_tolerance_, 0.5);
  nh.param("simulate_ahead_time", simulate_ahead_time_, 1.0);
  control_duration_ = 1/controller_frequency_;
}

void RotationShimController::reconfigureCB(Config& config, uint32_t level)
{
  if (!initialized_) {
    return;
  }
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  ROS_INFO("%s: Got a new reconfigure.", plugin_name_.c_str());
  controller_frequency_ = config.controller_frequency;
  control_duration_ = 1.0 / controller_frequency_;
  forward_sampling_distance_ = config.forward_sampling_distance;
  angular_dist_threshold_ = config.angular_dist_threshold;
  angle_threshold_ = config.angle_threshold;
  max_angular_vel_ = config.max_angular_vel;
  max_angular_accel_ = config.max_angular_accel;
  max_angular_deccel_ = config.max_angular_deccel;
  min_angular_vel_ = config.min_angular_vel;
  simulate_ahead_time_ = config.simulate_ahead_time;
}

bool RotationShimController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if(!initialized_) {
    ROS_ERROR("%s has not been initialized,"
              " please call initialize() before using this planner", plugin_name_.c_str());
    return false;
  }
  current_path_.clear();
  current_path_ = orig_global_plan;

  if (current_path_.empty()) {
    ROS_ERROR("%s: Received plan with zero length", plugin_name_.c_str());
    return false;
  }

  // Save goal pose
  goal_pose_.header.frame_id = current_path_[0].header.frame_id;
  goal_pose_.header.stamp = current_path_[0].header.stamp;
  goal_pose_.pose = current_path_.back().pose;

  if (has_new_goal_ || hasGoalChanged(goal_pose_))
  {
    last_goal_ = goal_pose_;
    has_new_goal_ = false;
    goal_changed_ = true;
    path_updated_ = true;
  }

  return controller_->setPlan(orig_global_plan);
}

bool RotationShimController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  
  if (current_path_.size() >= 2) {
    try {
      geometry_msgs::Pose sampled_pt_base = transformPoseToBaseFrame(getSampledPathPt());
      double angular_distance_to_heading =
            std::atan2(sampled_pt_base.position.y, sampled_pt_base.position.x);

      if (path_updated_) {
        std::lock_guard<std::mutex> lock_reinit(mutex_);
        if (shouldRotateToPath(angular_distance_to_heading)){
          if (isSafeRotate(robot_pose, angular_distance_to_heading)){
            if (computeRotateToHeadingCommand(cmd_vel, angular_distance_to_heading, robot_vel_, robot_pose)) {
              return true;
            }
          }
        }
      }
    }
    catch(const std::runtime_error& e) {
      std::cerr << e.what() << '\n';
      return false;
    }
  }
  path_updated_ = false;
  return controller_->computeVelocityCommands(cmd_vel);
}

bool RotationShimController::hasGoalChanged(
  const geometry_msgs::PoseStamped &new_goal)
{
  if (last_goal_.header.frame_id != new_goal.header.frame_id) {
      return true;
  }
  return last_goal_.pose.position.x != new_goal.pose.position.x
         || last_goal_.pose.position.y != new_goal.pose.position.y
         || tf2::getYaw(last_goal_.pose.orientation) != tf2::getYaw(new_goal.pose.orientation);
}

bool RotationShimController::isGoalReached()
{   
  if (controller_->isGoalReached()){
    has_new_goal_ = true;
    goal_changed_ = true;
    return true;
  }
  return false;
}

geometry_msgs::PoseStamped RotationShimController::getSampledPathPt()
{
  geometry_msgs::Pose start = current_path_.front().pose;
  double dx, dy;

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.size(); i++) {
    dx = current_path_[i].pose.position.x - start.position.x;
    dy = current_path_[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= forward_sampling_distance_) {
      current_path_[i].header.frame_id = current_path_.back().header.frame_id;
      current_path_[i].header.stamp = ros::Time::now();  // Get current time transformation
      return current_path_[i];
    }
  }
  auto goal = current_path_.back();
  goal.header.frame_id = current_path_.back().header.frame_id;
  goal.header.stamp = ros::Time::now();
  return goal;
}

bool RotationShimController::shouldRotateToPath(
  double & angular_distance_to_heading)
{
  double angle_to_path = angular_distance_to_heading;

  angular_distance_to_heading = angle_to_path >= 0.0 ? angle_to_path - angular_dist_threshold_ :
                                                       angle_to_path + angular_dist_threshold_;

  // Whether we should rotate robot to rough path heading
  return (fabs(angle_to_path) > angular_dist_threshold_);
}

bool RotationShimController::computeRotateToHeadingCommand(
  geometry_msgs::Twist& cmd_vel,
  const double & angular_distance_to_heading,
  geometry_msgs::Twist& curr_vel,
  const geometry_msgs::PoseStamped & robot_pose)
{
  // Rotate in place using max angular velocity / acceleration possible
  cmd_vel.linear.x = 0.0;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  cmd_vel.angular.z = sign * max_angular_vel_;
  const double & dt = control_duration_;

  const double min_feasible_angular_speed = curr_vel.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_vel.angular.z + max_angular_accel_ * dt;
  cmd_vel.angular.z = clamp(cmd_vel.angular.z, min_feasible_angular_speed, max_feasible_angular_speed);

  if (fabs(angular_distance_to_heading) < angle_threshold_) {
    cmd_vel.angular.z = 
      sign * clamp(max_angular_vel_ * fabs(angular_distance_to_heading) / max_angular_deccel_,
                   min_angular_vel_, max_angular_vel_);
  }

  if (!isCollisionFree(cmd_vel, angular_distance_to_heading, robot_pose)) {
    cmd_vel.angular.z = 0.0;
    return false;
  }

  return true;
}

bool RotationShimController::isCollisionFree(
  const geometry_msgs::Twist & cmd_vel,
  const double & angular_distance_to_heading,
  const geometry_msgs::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw = 0.0;
  double footprint_cost = 0.0;
  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - angular_dist_threshold_;

  while (simulated_time < simulate_ahead_time_) {
    simulated_time += control_duration_;
    yaw = initial_yaw + cmd_vel.angular.z * simulated_time;

    // Stop simulating past the point it would be passed onto the primary controller
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

    using namespace costmap_2d;  // NOLINT
    footprint_cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      yaw, costmap_ros_->getRobotFootprint());

    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      ROS_ERROR("%s: Detected a potential collision ahead!", plugin_name_.c_str());
      return false;
    }

    if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
      ROS_ERROR("%s: Detected collision ahead!", plugin_name_.c_str());
      return false;
    }
  }
  return true;
}

bool RotationShimController::isSafeRotate(
  const geometry_msgs::PoseStamped & pose,
  double yaw)
{
  if (!goal_changed_) {
    return true;
  }

  const double step_size = 0.1;
  const int steps = static_cast<int>(std::ceil(std::abs(yaw) / step_size));
  const double step_sign = (yaw > 0) ? 1.0 : -1.0;

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  for (int i = 0; i <= steps; ++i)
  {
    double current_angle = tf2::getYaw(pose.pose.orientation) + i * step_sign * step_size;
    if (i == steps) current_angle = tf2::getYaw(pose.pose.orientation) + yaw;
      std::vector<geometry_msgs::Point> rotated_footprint;
      for (const auto& point : footprint)
      {
        geometry_msgs::Point rotated_point;
        double new_x = std::cos(current_angle) * point.x - std::sin(current_angle) * point.y;
        double new_y = std::sin(current_angle) * point.x + std::cos(current_angle) * point.y;
        rotated_point.x = new_x + pose.pose.position.x;
        rotated_point.y = new_y + pose.pose.position.y;
        rotated_footprint.push_back(rotated_point);
      }
      for (const auto& point : rotated_footprint)
      {
        unsigned int mx, my;
        if (costmap_ros_->getCostmap()->worldToMap(point.x, point.y, mx, my))
        {
          unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
          if (cost == costmap_2d::LETHAL_OBSTACLE) {
            ROS_WARN("RotationShimController: Not safe for rotate!");
            return false;
          }
        }
      }
  }
  goal_changed_ = false;
  ROS_INFO("RotationShimController: No collision detected during rotation.");
  return true;
}

geometry_msgs::Pose
RotationShimController::transformPoseToBaseFrame(const geometry_msgs::PoseStamped & pt)
{
  geometry_msgs::PoseStamped pt_base;
  if (!transformPoseInTargetFrame(tf_, pt, pt_base, costmap_ros_->getBaseFrameID(), transform_tolerance_)) {
    throw std::runtime_error("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}

} // namespace rotation_shim_controller