#include <xpp_hyq/inverse_kinematics_wheelleg.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffector_mappings.h>

#include </home/hyeongu/catkin_workspace/src/towr/towr/include/towr/variables/euler_converter.h>


namespace xpp {

using namespace towr;

Joints
InverseKinematicsWheelleg::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();

  // std::cout << "X_B is : " << pos_B.front().transpose() << std::endl;

  for (const auto& b : pos_B) {                 // print global position
    std::cout << "X_B is : ";
    for (int i = 0; i < b.size(); ++i) {
        std::cout << b[i] << " ";
    }
    std::cout << std::endl;
    } 

  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {

    WheellegInverseKinematics::LegNum bend;

    
    using namespace quad;
    switch (ee) {
      case LF:
        std::cout << "LF leg" << std::endl;
        ee_pos_H = pos_B.at(ee); // world좌표 기준 x y z 좌표의 양수 음수 값 (+ + +)
        bend = WheellegInverseKinematics::Leg1;
        
        // std::cout << "LF world : " <<ee_pos_H.transpose() ;
        // std::cout << std::endl;
        
        // ee_pos_H -= b_R_w * base2hip_LF_;
        ee_pos_H -= base2hip_LF_;

        // std::cout << "LF local : " << ee_pos_H.transpose() ;
        // std::cout << std::endl;
        break;
      case RF:
        std::cout << "RF leg" << std::endl;
        ee_pos_H = pos_B.at(ee); // (+ - +)
        bend = WheellegInverseKinematics::Leg2;

        // std::cout << "RF world : " <<ee_pos_H.transpose() ;
        // std::cout << std::endl;

        // ee_pos_H -= b_R_w * base2hip_LF_.cwiseProduct(Eigen::Vector3d(1,-1,1));
        ee_pos_H -= base2hip_LF_.cwiseProduct(Eigen::Vector3d(1,-1,1));

        // std::cout << "RF local : " << ee_pos_H.transpose() ;
        // std::cout << std::endl;
        break;
      case LH:
        std::cout << "LH leg" << std::endl;
        ee_pos_H = pos_B.at(ee); // (- + +)        
        bend = WheellegInverseKinematics::Leg3;
        
        // std::cout << "LH world : " <<ee_pos_H.transpose() ;
        // std::cout << std::endl;

        // ee_pos_H -= b_R_w * base2hip_LF_.cwiseProduct(Eigen::Vector3d(-1,1,1));
        ee_pos_H -= base2hip_LF_.cwiseProduct(Eigen::Vector3d(-1,1,1));

        // std::cout << "LH local : " << ee_pos_H.transpose() ;
        // std::cout << std::endl;

        break;
      case RH:
        std::cout << "RH leg" << std::endl;
        ee_pos_H = pos_B.at(ee); // (- - +)  
        bend = WheellegInverseKinematics::Leg4;

        // std::cout << "RH world : " <<ee_pos_H.transpose();
        // std::cout << std::endl;

        // ee_pos_H -= b_R_w * base2hip_LF_.cwiseProduct(Eigen::Vector3d(-1,-1,1));
        ee_pos_H -= base2hip_LF_.cwiseProduct(Eigen::Vector3d(-1,-1,1));

        // std::cout << "RH local : " << ee_pos_H.transpose() ;
        // std::cout << std::endl;

        break;
      default: // joint angles for this foot do not exist
        break;
    }
 
    // base to wheel 에서 base to q_hr을 뺀 값  이 값이 IK에 들어갈 예정 
    q_vec.push_back(leg.GetJointAngles(ee_pos_H, bend));
    std::cout << q_vec.size() << std::endl;

    for (const auto& q : q_vec) {                 // print joint angle

      for (int i = 0; i < q.size(); ++i) {
          std::cout << "Joint Angle : " << q[i] << " ";
      }
      std::cout << std::endl;
    } 


  }

  //  for (const auto& worldpos : pos_B) {                 
  //   std::cout << "End-effector Position : ";
  //   for (int i = 0; i < worldpos.size(); ++i) {
  //       std::cout << worldpos[i] << " ";
  //   }
  //   std::cout << std::endl;
  //   }

  return Joints(q_vec);

 
}

} /* namespace xpp */


// namespace towr
// {
//   EulerConverter::MatrixSXd b_R_w = base_angular_.GetRotationMatrixBaseToWorld(t).transpose();
// }