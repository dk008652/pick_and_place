#include <kuka_arm/trajectory_planner.h>

// TrajectoryPlanner is a constructor of class defined in header file.
TrajectoryPlanner::TrajectoryPlanner (ros::NodeHandle nh)
	 : nh_(nh), 
	   move_group(PLANNING_GROUP),
	   eef_group(EEF_GROUP)
{

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

	planning_scene::PlanningScene planning_scene(kinematic_model);
	robot_state::RobotState robot_State(kinematic_model);

	joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	gripper_joint_model_group = eef_group.getCurrentState()->getJointModelGroup(EEF_GROUP);
	
	namespace rvt = rviz_visual_tools;

  	// set RRT as the planner and set allowed planning time
  	move_group.setPlannerId("RRTkConfigDefault");
  	move_group.setPlanningTime(10.0);
 	eef_group.setPlannerId("RRTkConfigDefault");
 	eef_group.setPlanningTime(5.0);


	//Adding collision objects
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	moveit_msgs::CollisionObject bin_collision_object, shelf_collision_object, ground_collision_object;

	ground_collision_object.header.frame_id = move_group.getPlanningFrame();
	ground_collision_object.id = "ground";

	shape_msgs::SolidPrimitive ground_prim;
	ground_prim.type = ground_prim.BOX;
 	ground_prim.dimensions.resize(3);
        ground_prim.dimensions[0] = 6.0;
	ground_prim.dimensions[1] = 6.0;
	ground_prim.dimensions[2] = 0.05;

	geometry_msgs::Pose bin_pose, shelf_pose, ground_pose;
	ground_pose.orientation.w = 1.0;
	ground_pose.position.x = 0.0; 
	ground_pose.position.y = 0.0;
	ground_pose.position.z = -0.025;

	shelf_pose.position.x = 2.7;
	shelf_pose.position.y = 0.0;
	shelf_pose.position.z = 0.84;
	shelf_pose.orientation.w = 0.707;
	shelf_pose.orientation.x = 0.0;
	shelf_pose.orientation.y = 0.0;
	shelf_pose.orientation.z = 0.707;
	
	bin_pose.position.x = 0.0;
	bin_pose.position.y = 2.5;
	bin_pose.position.z = 0.0;
	bin_pose.orientation.w = 1.0;
	bin_pose.orientation.x = 0.0;
	bin_pose.orientation.y = 0.0;
	bin_pose.orientation.z = 0.0;
	
	//PATH OF MESHES FILES TO BE ADDED LATER.
	//const std::string SHELF_MESH_PATH = "package://kuka_arm/models/kinematics_shelf/kinematics_shelf.dae"; 
	//const std::string BIN_MESH_PATH = "package://kuka_arm/models/kinematics_bin/kinematics_bin.dae";
	SetupCollisionObject("shelf", SHELF_MESH_PATH, shelf_pose, shelf_collision_object);
	SetupCollisionObject("bin", BIN_MESH_PATH, bin_pose, bin_collision_object);

	ground_collision_object.primitives.push_back(ground_prim);
	ground_collision_object.primitive_poses.push_back(ground_pose);
	ground_collision_object.operation = ground_collision_object.ADD;

	collision_objects.push_back(ground_collision_object);
	collision_objects.push_back(shelf_collision_object);
	collision_objects.push_back(bin_collision_object);
	
	//setting collision_objects to the world
	planning_scene_interface.addCollisionObjects(collision_objects);
	ROS_INFO("collision objects added into the planning scene");

	ros::Duration(2.0).sleep(); //let ros add collision objects into the world

	while(ros::ok()){
	
		//collision objects and trajectory visualization in rviZ
		//setting up moveit_visual_tools
		moveit_visual_tools::MoveItVisualTools visual_tools("world");
		visual_tools.deleteAllMarkers();

		visual_tools.loadRemoteControl();
		
		Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
		text_pose.translation().z() = 4.0;
		visual_tools.publishText(text_pose,"welcome to the pick and place project", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
		visual_tools.prompt("next step");

		// there are two option to get pick location
		// 1. load, pick and drop location to parameter server by .yaml file and access these location from parameter server itself
		// 2. define location here only , we are choosing first one

		//location of pick item(coke cane)
		//ros::param::get("/target_location/x", target_x);
		//ros::param::get("/target_location/y", target_y);
		//ros::param::get("/target_location/z", target_z);
		float target_x, target_y, target_z;
		target_x = 2.500087;
		target_y = 0.9;
		target_z = 1.552183;

		//location of drop()
		//ros::param::get("target_drop_location/x", bin_x);
		//ros::param::get("target_drop_location/y", bin_y);
		//ros::param::get("target_drop_location/z", bin_z);
		
		float bin_x, bin_y, bin_z;
		bin_x = 0.0;
		bin_y = 2.5;
		bin_z = 0.0;

		//rospy.set_param('target_drop_location', {'x': 0.0, 'y': 2.5, 'z': 0.0})

		geometry_msgs::Pose cane_pose, drop_pose, target_reach;
		// location to be defined later
		cane_pose.orientation.w = 1.0;
    		cane_pose.position.x = target_x - 0.4;
    		cane_pose.position.y = target_y;
    		cane_pose.position.z = target_z - 0.1;

    		target_reach.orientation.w = 1.0;
    		target_reach.position.x = target_x - 0.2;
    		target_reach.position.y = target_y;
    		target_reach.position.z = target_z - 0.1;

    		bin_pose.orientation.w = 1.0;
    		bin_pose.position.x = bin_x - 0.1;
    		bin_pose.position.y = bin_y;
    		bin_pose.position.z = bin_z + 1.6;

		//now we move to cane position(target position)
		move_group.setStartStateToCurrentState(); // current state of arm_group
		move_group.setPoseTarget(cane_pose);
				
		// to move from current state to target pose, robot follow some trajectory that to be saved (in plan object)
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		
		//if plan succeded or not
		bool success = static_cast<bool>(move_group.plan(my_plan));
		ROS_INFO("visualizing plan from current state to target pose (cane pose): %s", success? "SUCCEEDED": "FAILED");

		visual_tools.publishAxisLabeled(cane_pose, "target_pose or cane position");
		visual_tools.publishText(text_pose, "Displaying trajectory from current state to target pose", rvt::WHITE, rvt::XLARGE);
		
		//Convert plan to a set of eef poses for IK calculation
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		visual_tools.prompt("go for next step");
		
		
		//robot_state, which contain informatic snapshot of robot, (position/velocity/acceleration of jionts at a perticular time)
		moveit::core::RobotStatePtr robot_current_state;
		std::vector<double> robot_joint_positions; //hold the robot joint angles

		std::vector<geometry_msgs::Pose> path; // hold the eef position, all eef positions, while moving from current state to cane position(target pose)
		
		//std::size_t path_size;
		//bool demo;
		//ros::param::get("trajectory_sampler/demo", demo);
		// based on the demo value we decide which inverse kinematic algo to choose
		// where from we get demo value?
		
		//if demo == 1; we use moveit for inverse kinemaics else create our own inverse kinematics file 
		//
		//as of now we only use moveit IK
		bool demo = 1;
		if(demo){
			visual_tools.publishText(text_pose, "moving to target location (cane position)");
			visual_tools.trigger();
			success = static_cast<bool>(move_group.execute(my_plan));
			ROS_INFO("if moving to cane location succeded or not: %s", success ? "SUCCESS" : "FALIED");

		}	
	

		//we have left out the manually derived inverse kinematics algo
  
		// visualizing current state
		visual_tools.publishText(text_pose, "reached to target position", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
		visual_tools.prompt("keep going on, u r fucking moving in right direction");
	
		//once we have reached near to the target now its time to approach the target, Same like once we reached near to crush, proposing must be the next step, if one have guts, i dont have.
		//approach the target
		visual_tools.publishText(text_pose, "trying to reach the target", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
	

		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(target_reach);
		success = static_cast<bool>(move_group.move());
		ROS_INFO("HAVE WE REACHED TO THE TARGET: %s", success? "YES, YOU GOT SUCCESS" : "NO MY DEAR FRIEND, U R STUCKED SOMEWHERE");
		visual_tools.prompt("move to next step u basterd");


		// NOW GRASP THE TARGET
		// Let's relate it to some story, lets say u got guts and approached ur crush and she accepted ur proposal
		// so now grasp her hand and smooch her hand
		// here we reached to the target, and going to grasp the cane(target)
	
		visual_tools.publishText(text_pose, "grasp the target", rvt::WHITE, rvt::XLARGE);
		CloseGripper();
		ros::Duration(5.0).sleep();
		visual_tools.prompt("if not tired than move to next step, if so, still u have to move to next step, u r left with no option");


		//pullback the gripper, once cane(target) grasped
		// visualizing this whole stuff
		visual_tools.publishText(text_pose, "retriveing the cane", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
	
		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(cane_pose);  //target_pose
		success = static_cast<bool>(move_group.move());
		ROS_INFO("Not neccessary to success everytime u try, but still lets try this out (TARGET RETRIVED OR NOT): %s", success ? "SUCCESS" : "BADLY FAILED, U DUMBASS");	
		visual_tools.prompt("ohhhk, hope u do good in next step too, like previous stages");

	
		// Now plan to place cane(target) in Dustbin
		// Its not neccesary, ur proposal get accepted, if rejected than find a place in dustbin atleast for a while,
		// so now we plan to drop the cane into dustbin. (bin_pose)
	

		//plan for bin_pose		
		move_group.setStartStateToCurrentState();
		move_group.setPoseTarget(bin_pose);
		success = static_cast<bool>(move_group.plan(my_plan));
		ROS_INFO("VISUALINNG FOR DROP LOCATION: %s", success ? "YES YES, WE ROCKED" : " GO BACK TO CHECK UR DUMPYARD");
		
		
		//Visualizing plan for drop location
		visual_tools.publishText(text_pose, "I have Plan to move at dumping yard (drop location)", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		visual_tools.prompt("finished droping task, Now its time to move-on");


		demo = 1;
		if(demo){
			visual_tools.publishText(text_pose, "my plan to move to dumping yard is finally coming true, Atleast hoping so", rvt::WHITE, rvt::XLARGE);
			visual_tools.trigger();
			success = static_cast<bool>(move_group.execute(my_plan));
			ROS_INFO("HAVE we reached to dumping yard, means drop location: %s", success ? "SUCCEEDED" : "YES, U HAVE FAILED AT UR ALMOST FINAL STEP");

		}


		ROS_INFO("moving to drop location: %s", success ? "SUCCEEDED" : "BADLY FAILED");

		//displaying current state, not current state of robot, but in this task where we have reached
		visual_tools.publishText(text_pose, "reached to drop location", rvt::WHITE, rvt::XLARGE);
		visual_tools.trigger();
		visual_tools.prompt("Next Step");

		// Open the gripper and release cane
    		// Display current state
    		visual_tools.publishText(text_pose, "Releasing target(cane)",
                             rvt::WHITE, rvt::XXXXLARGE);
    		visual_tools.trigger();
    		OpenGripper();
    		ros::Duration(5.0).sleep();

		// Update cycle cycle_counter
    		// cycle_counter++;
		// display current state
    		visual_tools.publishText(text_pose, "  End of Pick-Place cycle\nPress Next to begin a new cycle",
                             rvt::WHITE, rvt::XXXXLARGE);
    		visual_tools.trigger();
    		visual_tools.prompt("next step");

		robot_current_state = move_group.getCurrentState();

		//get the current angles of the joint
		robot_joint_positions.clear();
		robot_current_state->copyJointGroupPositions(joint_model_group, robot_joint_positions);


		for(std::size_t i = 0; i < robot_joint_positions.size(); ++i){
			robot_joint_positions[i] = 0; //moving robot to initial position or ideal position
		}


		move_group.setJointValueTarget(robot_joint_positions);
		success = static_cast<bool>(move_group.move());
	ROS_INFO("HAVE WE MOVED TO IDEAL STATE, CAUSE NOW I WANT TO TAKE SOME REST, ITS BEEN QUITE A LONG TIME SINCE I STARTED WORKING ON UR BULLSHIT TASK: %s", success ? "successfully, we hav reached to ideal state" : "No, im still stuck somewhere, I HAVE FAILED U"); 

	}

}



bool TrajectoryPlanner::OperateGripper(const bool &close_gripper)
{
  // RobotState contains the current position/velocity/acceleration data
  moveit::core::RobotStatePtr gripper_current_state =
    eef_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group,
      gripper_joint_positions);

  ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());

  // Set finger joint values
  if (close_gripper)
  {
    gripper_joint_positions[0] = 0.02;  // radians
    gripper_joint_positions[1] = 0.02;  // radians
  }
  else
  {
    gripper_joint_positions[0] = -0.01;  // radians
    gripper_joint_positions[1] = -0.01;  // radians
  }

  eef_group.setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.5).sleep();

  bool success = static_cast<bool>(eef_group.move());
  return success;
}


bool TrajectoryPlanner::OpenGripper()
{
  bool success = OperateGripper(false);
  ROS_INFO("Gripper actuation: Opening %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}

bool TrajectoryPlanner::CloseGripper()
{
  bool success = OperateGripper(true);
  ROS_INFO("Gripper actuation: Closing %s", success ? "SUCCEEDED" : "FAILED");
  return success;
}


bool TrajectoryPlanner::SetupCollisionObject(const std::string &object_id, const std::string &mesh_path, const geometry_msgs::Pose &object_pose, moveit_msgs::CollisionObject &collision_object)
{
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

  ROS_DEBUG_STREAM(object_id << " mesh loaded");

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1);
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;
}


TrajectoryPlanner::~TrajectoryPlanner() {}

int main(int argc, char **argv){

	ros::init(argc, argv, "trajectory_planner");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(5);
	spinner.start();
	TrajectoryPlanner trajectoryPlanner(nh); // An Object of TrajectoryPlanner class created.

	return 0;

}






