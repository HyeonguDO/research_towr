#ifndef XPP_VIS_INVERSEKINEMATICS_WHEELLEG_H_
#define XPP_VIS_INVERSEKINEMATICS_WHEELLEG_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_hyq/wheelleg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the wheelleg robot.
 */
class InverseKinematicsWheelleg : public InverseKinematics {
public:
  InverseKinematicsWheelleg() = default;
  virtual ~InverseKinematicsWheelleg() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };

private:
  Vector3d base2hip_LF_ = Vector3d(0.25142, 0.170, 0.066);
  WheellegInverseKinematics leg;   // dlrj tnwjd
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSEKINEMATICS_WHEELLEG_H_ */