#include<ros/ros.h>

//moveit

#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_state/robot_state.h>
#include<moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv){
ros::init(argc, argv, "robot_model_and_robot_state");
ros::NodeHandle nh;
ros::AsyncSpinner spinner(1);
spinner.start();


/*robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

ROS_INFO("Model frame: %s ", kinematic_model->getModelFrame().c_str());

robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");

const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
std::vector<double> joint_values;
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

for(std::size_t i = 0; i<joint_names.size(); ++i){
	ROS_INFO("Joint %s : %d", joint_names[i].c_str(), joint_values[i]);
}
*/

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "panda_arm" of the Panda
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }



joint_values[0] = 5.0;

kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

//ROS_INFO("current state is :", kinematic_state->satisfiesBounds() ? "valid" : "not valid");
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

kinematic_state->enforceBounds();
//ROS_INFO("Current state is :", kinematic_state->satisfiesBounds() ? "valid" : "not valid");
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

kinematic_state->setToRandomPositions(joint_model_group);

const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");

ROS_INFO_STREAM(" end effector Translation : \n"<< end_effector_state.translation() << "\n");
ROS_INFO_STREAM(" end effector Rotation :\n" << end_effector_state.rotation() << "\n");

double time_out = 0.1;
bool found_ik =  kinematic_state->setFromIK(joint_model_group, end_effector_state, time_out);


if(found_ik){
ROS_INFO("IK found");
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

}
else ROS_INFO("IK, did not found");


//Finding Jacobians


Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
Eigen::MatrixXd jacobian;

kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

ROS_INFO_STREAM("jacobian :\n " << jacobian << "\n"); 
 


ros::shutdown();
return 0;

}
