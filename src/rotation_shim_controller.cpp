#include <rotation_shim_controller/rotation_shim_controller.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rotation_shim_controller::RotationShimController, nav_core::BaseLocalPlanner)

namespace rotation_shim_controller
{
RotationShimController::RotationShimController():
  lp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
  primary_controller_(nullptr),
  initialized_(false),
  path_updated_(false){}

RotationShimController::~RotationShimController()
{
  primary_controller_.reset();
}

void RotationShimController::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_){

    ros::NodeHandle nh("~/" + name);

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    
    has_new_goal_ = true;

    // Initialize parameters
    initParams(nh);

    try{
      primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
      std::size_t pos = primary_controller.find_last_of("/");
      std::string primary_controller_name = primary_controller.substr(pos + 1);
      primary_controller_->initialize(name + "/" + primary_controller_name, tf, costmap_ros);
      ROS_INFO("Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller.c_str());
    }
    catch(const pluginlib::PluginlibException &ex){
      ROS_DEBUG("Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    }

    // initialize collision checker and set costmap
    collision_checker_ = std::make_unique<
      FootprintCollisionChecker<costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());

    // Create subscriber:
    run_controller_sub_ = nh.subscribe<std_msgs::Bool>("/run_rs_controller", 5,
                                                       &RotationShimController::runControllerCallback, this);

    ROS_INFO("Initialized %s plugin", plugin_name_.c_str());
    initialized_ = true;
  }
}

void RotationShimController::initParams(ros::NodeHandle& nh)
{
  nh.param("primary_controller", primary_controller, std::string("teb_local_planner/TebLocalPlannerROS"));
  nh.param("forward_sampling_distance", forward_sampling_distance_, 0.5);
  nh.param("angular_dist_threshold", angular_dist_threshold_, 0.55);
  nh.param("goal_angular_vel_scaling_angle", goal_angular_vel_scaling_angle_, 0.55);
  nh.param("goal_angle_scaling_factor", goal_angle_scaling_factor_, 1.2);
  nh.param("rotate_to_goal_max_angular_vel", rotate_to_goal_max_angular_vel_, 0.5);
  nh.param("rotate_to_goal_min_angular_vel", rotate_to_goal_min_angular_vel_, 0.05);
  nh.param("transform_tolerance", transform_tolerance_, 0.5);
  nh.param("simulate_ahead_time", simulate_ahead_time_, 1.0);
  nh.param("controller_frequency", controller_frequency, 15.0);
  control_duration_ = 1/controller_frequency;
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

  return primary_controller_->setPlan(orig_global_plan);
}

bool RotationShimController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  std::lock_guard<std::mutex> lock_reinit(mutex_);
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  
  try {
    geometry_msgs::Pose sampled_pt_base = transformPoseToBaseFrame(getSampledPathPt());
    double angle_to_path =
          std::atan2(sampled_pt_base.position.y, sampled_pt_base.position.x);

    if (has_new_goal_ || hasGoalChanged(goal_pose_)) {
      last_goal_ = goal_pose_;
      has_new_goal_ = false;
      path_updated_ = true;
    }

    if (path_updated_) {
      if (shouldRotateToPath(angle_to_path)){
        ROS_DEBUG("%s: Rotating to path heading...", plugin_name_.c_str());
        if (rotateToHeading(cmd_vel.linear.x,
                            cmd_vel.angular.z,
                            angle_to_path,
                            robot_pose)) {
          return true;
        }
      }
    }
  } catch(const std::runtime_error & e) {
    ROS_DEBUG("%s: %s", plugin_name_.c_str(), e.what());
  }
  path_updated_ = false;
  
  return primary_controller_->computeVelocityCommands(cmd_vel);
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
  if (primary_controller_->isGoalReached()){
    has_new_goal_ = true;
    return true;
  }
  return false;
}

geometry_msgs::PoseStamped RotationShimController::getSampledPathPt()
{
  if (current_path_.size() < 2) {
    throw std::runtime_error("Path is too short to find a valid sampled path point for rotation.");
  }

  // Save goal pose
  goal_pose_.header.frame_id = current_path_[0].header.frame_id;
  goal_pose_.header.stamp = current_path_[0].header.stamp;
  goal_pose_.pose = current_path_.back().pose;

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

bool RotationShimController::shouldRotateToPath(double angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  return (fabs(angle_to_path) > angular_dist_threshold_);
}

bool RotationShimController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  double angle_to_path, const geometry_msgs::PoseStamped & robot_pose)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  double factor;
  bool is_stopped = fabs(angle_to_path) <= angular_dist_threshold_;
      
  if (std::abs(angle_to_path) < (goal_angular_vel_scaling_angle_ + angular_dist_threshold_)) {
      factor = std::abs(angle_to_path) / goal_angle_scaling_factor_;
  } else {
      factor = 1.0;
  }

  double rotate_to_goal_angular_vel = rotate_to_goal_max_angular_vel_;
  double unbounded_angular_vel = rotate_to_goal_angular_vel * factor;

  if (unbounded_angular_vel < rotate_to_goal_min_angular_vel_) {
      rotate_to_goal_angular_vel = rotate_to_goal_min_angular_vel_;
  } else {
      rotate_to_goal_angular_vel = unbounded_angular_vel;
  }
  if (!isCollisionFree(linear_vel, angular_vel, is_stopped, robot_pose)) {
    angular_vel = 0.0;
    return false;
  }

  angular_vel = sign*clamp(rotate_to_goal_angular_vel,
                           rotate_to_goal_min_angular_vel_,
                           rotate_to_goal_max_angular_vel_);
  return true;
}

bool RotationShimController::isCollisionFree(
  double & linear_vel, double & angular_vel,
  bool is_stopped,
  const geometry_msgs::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw = 0.0;
  double footprint_cost = 0.0;

  while (simulated_time < simulate_ahead_time_) {
    simulated_time += control_duration_;
    yaw = initial_yaw + angular_vel * simulated_time;

    // Stop simulating past the point it would be passed onto the primary controller
    if (is_stopped) {
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

geometry_msgs::Pose
RotationShimController::transformPoseToBaseFrame(const geometry_msgs::PoseStamped & pt)
{
  geometry_msgs::PoseStamped pt_base;
  if (!transformPoseInTargetFrame(pt, pt_base, tf_, costmap_ros_->getBaseFrameID(), transform_tolerance_)) {
    throw std::runtime_error("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}

void RotationShimController::runControllerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  path_updated_ = msg->data;
}
    
} // namespace rotation_shim_controller