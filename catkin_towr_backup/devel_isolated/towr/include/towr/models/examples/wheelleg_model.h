#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_WHEELLEG_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_WHEELLEG_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

#include <cmath>
#include <Eigen/Dense>
#include <iostream>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot HyQ.
 */
class WheellegKinematicModel : public KinematicModel {
public:
  WheellegKinematicModel () : KinematicModel(4)
  {
    // const double x_nominal_b = 0.464229;
    // const double y_nominal_b = 0.381505;
    // const double z_nominal_b = -0.491763;

    const double x_nominal_b = 0.35;
    const double y_nominal_b = 0.2669;
    const double z_nominal_b = -0.5;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.25, 0.20, 0.15;
  }
};

/**
 * @brief The Dynamics of the quadruped robot HyQ.
 */
class WheellegDynamicModel : public SingleRigidBodyDynamics {
public:
  WheellegDynamicModel() : SingleRigidBodyDynamics(83,
                      4.26, 8.97, 9.88, -0.0063, 0.193, 0.0126,
                      4) {}
};

} /* namespace towr */

#endif 