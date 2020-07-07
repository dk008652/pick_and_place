#ifndef KUKA_ARM_TRAJECTORY_PLANNER_H
#define KUKA_ARM_TRAJECTORY_PLANNER_H

#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>

#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TrajectoryPlanner{

public:
	explicit TrajectoryPlanner(ros::NodeHandle nh);   //constructor
	~TrajectoryPlanner();	//destructor
private:
	ros::NodeHandle nh_;
	const std::string PLANNING_GROUP = "kuka_arm";
	const std::string EEF_GROUP = "gripper";
	
	moveit::planning_interface::MoveGroupInterface move_group;	
	moveit::planning_interface::MoveGroupInterface eef_group;

	const robot_state::JointModelGroup *joint_model_group;
	const robot_state::JointModelGroup *gripper_joint_model_group;

	const std::string SHELF_MESH_PATH = "package://kuka_arm/models/kinematics_shelf/kinematics_shelf.dae"; 
	const std::string BIN_MESH_PATH = "package://kuka_arm/models/kinematics_bin/kinematics_bin.dae";

	bool OperateGripper(const bool &close_gripper);
  	bool OpenGripper();
  	bool CloseGripper();

  	bool SetupCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::Pose &object_pose,
                            moveit_msgs::CollisionObject &collision_object);

};

#endif
