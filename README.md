# pick_and_place
it's said "Documentation is a love letter that you write to your future self" so let's start.

1. To start working with any manipulator create a workspace 
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
$ catkin build   // This cmd build catkin_ws workspace and endup creating 3 new directories, log, devel, and build.
 
To build workspace we require catkin-tools package installed in our system, if not installed....
run $................... 
$ cd src

2. We create a package which will have files related to robotic manipulator like .urdf, .dae, and manymore.

$ catkin_create_pkg kuka_arm {list of all dependencies of this package}

To find all dependecies, open package.xml which resides in kuka_arm package.

3. Sourcing environment
$ cd ~/catkin_ws
$ source devel/setup.bash   //this cmd needed to run everytime we make any changes in package or open new terminal. so better update .bashrc file.

I. Go to home directory and press ctrl+h, which show all hidden files.
II. Add "source ~/catkin_ws/source/devel/setup.bash" at the end of .bashrc file.   
  
4. Add urdf file in kuka_arm/urdf and meshes file in kuka_arm/meshe directory.

5. Create launch file in kuka_arm/launch to spawn manipulator in gazebo. "pid gain not found" error can be ignored as of now.

6. Create moveit_package. 
Follow this link.
http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

 Be cautious, finger joints(gripper) have effortJointInterface and link joints have PositionJointInterface.
 Aviod step 10, as we create our own controller file specifically for kuka210.

7. Once moveit package created, we add controllers.yaml file in kuka210_movevit/config directory. another controller file for gazebo also added in kuka210_gazebo named gazebo_controllers.yaml.

modifing ros_controllers.launch avail in kuka210_moveit/launch
{
1. commenting ros_controllers.yaml 
2. adding controllers.yaml 
3. adding gazebo_controllers.yaml
4. "arg" parameter modified (args="joint_state_controller arm_controller gripper_controller"), ns ="/kuka_arm" arg added.
}  

8. git clone ros-control in catkin_ws/src directory.
"ros-control installation"
http://wiki.ros.org/ros_control.

9.  kuka_arm_moveit_controller_manager.launch.xml file of kuka210_moveit/launch file modified 
this launch file used to upload ros_controller.yaml to parameter server, Now this .yaml file changed by controller.yaml (resides in kuka210_moveit/config directory).

10. create .world file to launch world in gazebo. .world file is created using .sdf file by adding sdf content into .world file inside <world> tag.
World includes shelf, bin, and bottle.

11. Adding .sdf, .config and .dae files of shelf, bin and bottle into .gazebo directory. if these files are not added in .gazebo directory we wont be seeing these models in gazebo simulation environment. 

12. Adding a node in any package and modification in cmakelist.txt file. To run any node, we create launch file.
what modification to make in cmakelist.txt ? visit https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Creating-a-ROS-Package-and-Node.html

13. Adding a header file and changes to be made in cmakelist.txt
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})  


