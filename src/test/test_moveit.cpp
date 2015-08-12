#include <iostream>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <Eigen/Geometry>

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

using namespace std;

int
main (int argc,
      char **argv)
{
  cout << "Doing planning here 11 ..." << endl;

  bool success;
  ros::init (argc, argv, "main_planning");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner (1);
  spinner.start ();

  cout << "Doing planning here 22 ..." << endl;
  moveit::planning_interface::MoveGroup group ("right_arm");

  cout << "Doing planning here 33 ..." << endl;

}



