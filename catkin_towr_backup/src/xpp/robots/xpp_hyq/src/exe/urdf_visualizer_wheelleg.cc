#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_hyq/inverse_kinematics_wheelleg.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;  // LF=0, RF=1, LH=2, RH=3으로 지정됨  endeffector_mappings.h에서..
 
int main(int argc, char *argv[]) // int argc : main함수에 전달되는 데이터의 개수,  char *argv[] : main함수에 전달되는 실제적인 데이터로 char형 포인터 배열로 구성됨.
{
  ::ros::init(argc, argv, "wheelleg_urdf_visualizer");  // argc, argv 이후에 오는 3번째 " " 는 node의 이름이다.

  const std::string joint_desired_wheelleg = "xpp/joint_wheelleg_des";

  auto wheelleg_ik = std::make_shared<InverseKinematicsWheelleg>();
  CartesianJointConverter inv_kin_converter(wheelleg_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_wheelleg);

  // urdf joint names
  int n_ee = wheelleg_ik->GetEECount();
  int n_j  = WheellegJointCount;
  // int n_ee = 4;
  // int n_j = 4;

  std::vector<UrdfVisualizer::URDFName> joint_names(12);
  
  // joint_names.at(0) = "LF_hr_joint";
  // joint_names.at(1) = "RF_hr_joint";
  // joint_names.at(2) = "LH_hr_joint";
  // joint_names.at(3) = "RH_hr_joint";

  // joint_names.at(4) = "LF_hp2_joint";
  // joint_names.at(5) = "RF_hp2_joint";
  // joint_names.at(6) = "LH_hp2_joint";
  // joint_names.at(7) = "RH_hp2_joint";

  // joint_names.at(8) = "LF_k_joint";
  // joint_names.at(9) = "RF_k_joint";
  // joint_names.at(10) = "LH_k_joint";
  // joint_names.at(11) = "RH_k_joint";

  // joint_names.at(12) = "LF_w_joint";
  // joint_names.at(13) = "RF_w_joint";
  // joint_names.at(14) = "LH_w_joint";
  // joint_names.at(15) = "RH_w_joint";

  joint_names.at(0) = "LF_hr_joint";
  joint_names.at(3) = "RF_hr_joint";
  joint_names.at(6) = "LH_hr_joint";
  joint_names.at(9) = "RH_hr_joint";

  joint_names.at(1) = "LF_hp2_joint";
  joint_names.at(4) = "RF_hp2_joint";
  joint_names.at(7) = "LH_hp2_joint";
  joint_names.at(10) = "RH_hp2_joint";

  joint_names.at(2) = "LF_k_joint";
  joint_names.at(5) = "RF_k_joint";
  joint_names.at(8) = "LH_k_joint";
  joint_names.at(11) = "RH_k_joint";

  // joint_names.at(3) = "LF_w_joint";
  // joint_names.at(7) = "RF_w_joint";
  // joint_names.at(11) = "LH_w_joint";
  // joint_names.at(15) = "RH_w_joint";

  // joint_names.at(0) = "LF_hr_joint";
  // joint_names.at(1) = "LF_hp2_joint";
  // joint_names.at(2) = "LF_k_joint";
  // joint_names.at(3) = "LF_w_joint";

  // joint_names.at(4) = "RF_hr_joint";
  // joint_names.at(5) = "RF_hp2_joint";
  // joint_names.at(6) = "RF_k_joint";
  // joint_names.at(7) = "RF_w_joint";

  // joint_names.at(8) = "LH_hr_joint";
  // joint_names.at(9) = "LH_hp2_joint";
  // joint_names.at(10) = "LH_k_joint";
  // joint_names.at(11) = "LH_w_joint";

  // joint_names.at(12) = "RH_hr_joint";
  // joint_names.at(13) = "RH_hp2_joint";
  // joint_names.at(14) = "RH_k_joint";  
  // joint_names.at(15) = "RH_w_joint";
  

  std::string urdf = "wheelleg_rviz_urdf_robot_description";
  UrdfVisualizer wheelleg_desired(urdf, joint_names, "base", "world",
			     joint_desired_wheelleg, "wheelleg_des");         

  ::ros::spin();

  return 1;
}