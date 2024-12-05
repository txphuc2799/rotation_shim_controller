#ifndef __ROTATION_SHIM_CONTROLLER_H__
#define __ROTATION_SHIM_CONTROLLER_H__

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <pluginlib/class_loader.h>
#include <geometry_msgs/TwistStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Bool.h>
#include <rotation_shim_controller/utils.h>
#include <rotation_shim_controller/footprint_collision_checker.hpp>
#include <rotation_shim_controller/RotationShimControllerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace rotation_shim_controller
{

class RotationShimController : public nav_core::BaseLocalPlanner
{

typedef rotation_shim_controller::RotationShimControllerConfig Config;
typedef dynamic_reconfigure::Server<Config> ParamterConfigServer;
typedef dynamic_reconfigure::Server<Config>::CallbackType CallbackType;

public:
    /**
    * @brief Default constructor of the plugin
    */
    RotationShimController();

    /**
    * @brief  Destructor of the plugin
    */
    ~RotationShimController();

    /**
    * @brief Initializes the plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param costmap_ros Cost map representing occupied and free space
    */
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief Set the plan that the local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
    * @brief  Check if the goal pose has been achieved
    * @return True if achieved, false otherwise
    */
    bool isGoalReached();

protected:
    /**
    * @brief Finds the point on the path that is roughly the sampling
    * point distance away from the robot for use.
    * May throw exception if a point at least that far away cannot be found
    * @return pt location of the output point
    */
    geometry_msgs::PoseStamped getSampledPathPt();

    /**
     * @brief Uses TF to find the location of the sampled path point in base frame
     * @param pt location of the sampled path point
     * @return location of the pose in base frame
     */
    geometry_msgs::Pose transformPoseToBaseFrame(const geometry_msgs::PoseStamped & pt);
    
    bool shouldRotateToPath(double & angular_distance_to_heading);

    bool computeRotateToHeadingCommand(
        geometry_msgs::Twist& cmd_vel,
        const double & angular_distance_to_heading,
        geometry_msgs::Twist& curr_vel,
        const geometry_msgs::PoseStamped & robot_pose);

    bool hasGoalChanged(const geometry_msgs::PoseStamped &new_goal);
<<<<<<< HEAD

=======
    
>>>>>>> 20af4a4e7897805e33a2e35ac59b56698f25d4b0
    void initParams(ros::NodeHandle& nh);

    /**
     * @brief Checks if rotation is safe
     * @param cmd_vel Velocity to check over
     * @param angular_distance_to_heading Angular distance to heading requested
     * @param pose Starting pose of robot
     */
    bool isCollisionFree(
        const geometry_msgs::Twist & cmd_vel,
        const double & angular_distance_to_heading,
        const geometry_msgs::PoseStamped & pose);
    
    void reconfigureCB(Config& config, uint32_t level);

protected:
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> lp_loader_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> controller_;
    base_local_planner::OdometryHelperRos odom_helper_;
    std::unique_ptr<FootprintCollisionChecker<costmap_2d::Costmap2D *>>
    collision_checker_;

    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::string primary_controller_;
    std::string plugin_name_ = "RotationShimController";
    std::string odom_topic_;
    double forward_sampling_distance_, angular_dist_threshold_;
    double angle_threshold_;
    double max_angular_vel_;
    double max_angular_accel_;
    double max_angular_deccel_;
    double min_angular_vel_;
    double simulate_ahead_time_;
    double transform_tolerance_;
    double control_duration_, controller_frequency_;

    geometry_msgs::Twist robot_vel_;
    std::vector<geometry_msgs::PoseStamped> current_path_;
    geometry_msgs::PoseStamped goal_pose_;
    geometry_msgs::PoseStamped last_goal_;

    bool initialized_;
    bool has_new_goal_;
    bool path_updated_;

    // Dynamic parameters handler
    std::mutex mutex_;
    ParamterConfigServer* dynamic_srv_;
};
} // namespace rotation_shim_controller

#endif  // __ROTATION_SHIM_CONTROLLER_H__