#include <ros/ros.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector<double> joint_values;
  moveit::planning_interface::MoveGroupInterface rng_group_interface("rocket_and_groot");
  moveit::planning_interface::MoveGroupInterface rocket_group_interface("rocket");
  moveit::planning_interface::MoveGroupInterface groot_group_interface("groot");

  rng_group_interface.setMaxVelocityScalingFactor(1.0);
  rng_group_interface.setMaxAccelerationScalingFactor(1.0);
  rng_group_interface.setPlanningTime(15.0);
  rng_group_interface.setNumPlanningAttempts(20.0);
  
  moveit::core::RobotModelConstPtr kinematic_model = rocket_group_interface.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = rng_group_interface.getCurrentState();
  const moveit::core::JointModelGroup* rocket_joint_model_group = kinematic_model->getJointModelGroup("rocket");
  const moveit::core::JointModelGroup* groot_joint_model_group = kinematic_model->getJointModelGroup("groot");

  const std::vector<std::string>& rocket_joint_names = rocket_joint_model_group->getVariableNames();
  const std::vector<std::string>& groot_joint_names = groot_joint_model_group->getVariableNames();
  std::vector<double> rocket_joint_values;
  std::vector<double> groot_joint_values;

  std::random_device rd; 
  std::mt19937 gen(rd()); 
  std::uniform_int_distribution<> distr(-10, 10);
  std::uniform_int_distribution<> rad_distr(-30, 30);

  geometry_msgs::Pose rocket_pose;
  geometry_msgs::Pose groot_pose;
 
  // w x y z
  Eigen::Quaternionf rocket_q = Eigen::Quaternionf(0.0044319521005895665 , -0.0018064082028716572, 0.714190127940822, -0.6999353940485185);
  Eigen::Quaternionf groot_q = Eigen::Quaternionf(0.7171097271676862 , -0.6959453209354478, -0.029260144371181365, -0.02361341612136324);

  for(int i; i < 100; i++){

    float random_x = ( ((float) distr(gen)) * 0.01);
    float random_y = ( ((float) distr(gen)) * 0.01);
    float random_z = ( ((float) distr(gen)) * 0.01);

    rocket_pose.position.x = -0.015463195119993365 + random_x;
    rocket_pose.position.y = 0.02029402510664674 + random_y;
    rocket_pose.position.z = 1.658157440477098 + random_z;
    groot_pose.position.x = -0.01565011581780207 + random_x;
    groot_pose.position.y = -0.019683543216663102 + random_y;
    groot_pose.position.z = 1.657396455658871 + random_z;

    float x_rotation = rad_distr(gen) * 0.01;
    float y_rotation = rad_distr(gen) * 0.01;
    float z_rotation = rad_distr(gen) * 0.01;

    rocket_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                rocket_q;

    groot_q = Eigen::AngleAxisf(x_rotation, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(y_rotation, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(z_rotation, Eigen::Vector3f::UnitZ()) *
                groot_q;

    rocket_pose.orientation.w = rocket_q.w();
    rocket_pose.orientation.x = rocket_q.x();
    rocket_pose.orientation.y = rocket_q.y();
    rocket_pose.orientation.z = rocket_q.z();
    groot_pose.orientation.w = groot_q.w();
    groot_pose.orientation.x = groot_q.x();
    groot_pose.orientation.y = groot_q.y();
    groot_pose.orientation.z = groot_q.z();

    double timeout = 0.1;
    bool rocket_found_ik = kinematic_state->setFromIK(rocket_joint_model_group, rocket_pose, timeout);
    bool groot_found_ik = kinematic_state->setFromIK(groot_joint_model_group, groot_pose, timeout);

    if (rocket_found_ik && groot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(rocket_joint_model_group, rocket_joint_values);
      kinematic_state->copyJointGroupPositions(groot_joint_model_group, groot_joint_values);

      for (std::size_t i = 0; i < rocket_joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", rocket_joint_names[i].c_str(), rocket_joint_values[i]);
        ROS_INFO("Joint %s: %f", groot_joint_names[i].c_str(), groot_joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    rng_group_interface.setJointValueTarget(rocket_joint_names, rocket_joint_values);
    rng_group_interface.setJointValueTarget(groot_joint_names, groot_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (rng_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success){
      ROS_INFO("Plan did not successed");
    }
    rng_group_interface.execute(my_plan);
  }

  ros::shutdown();
}