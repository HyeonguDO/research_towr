#ifndef XPP_VIS_WHEELLEG_INVERSE_KINEMATICS_H_
#define XPP_VIS_WHEELLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

namespace xpp {

enum WheellegJointID {hr=0, hp1, hp2, k, w, WheellegJointCount};

/**
 * @brief Converts a wheelleg foot position to joint angles.
 */
class WheellegInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum LegNum { Leg1, Leg2, Leg3, Leg4 }; 
  

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  WheellegInverseKinematics () = default;
  virtual ~WheellegInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached   ->  hr조인트 프레임에 대한 end-effector의 좌표를 IK에 넣어 사용.
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, LegNum bend=Leg1) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (hr, hp, k) this value represents.
   */
  void EnforceLimits(double& q, WheellegJointID joint) const;

  
  
  private:
  double length_l1 = 0.0975;  // q_hr q_hp 세로 방향 길이
  double length_l2 = 0.3; // length of upper leg  matlab상에서 l_2
  double length_l3 = 0.3; // length of lower leg  matlab상에서 l_3

  double length_lb = 0.25142; // body 세로 길이
  double length_db = 0.170; // body 가로 길이
  double length_dZ = 0.066; // body height

  double length_d1 = 0.06785; // q_hr 이랑 q_hp 가로 방향 길이차이
  double length_d2 = 0.029; // q_hp q_k 가로 방향 길이

};

} /* namespace xpp */

#endif /* XPP_VIS_WHEELLEG_INVERSE_KINEMATICS_H_ */