/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_

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
class HyqKinematicModel : public KinematicModel {
public:
  HyqKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.31;                // origin
    const double y_nominal_b = 0.29;
    const double z_nominal_b = -0.5;

    

    // const double x_nominal_b = 0.323;                // wheelleg
    // const double y_nominal_b = 0.202;
    // const double z_nominal_b = -0.61;


    double phi = -0.1;     // roll angle
    double theta = 0.1;   // ptich angle

    // x축을 기준으로 phi만큼 회전하는 변환
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // y축을 기준으로 theta만큼 회전하는 변환
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // x축 회전 뒤 y축 회전을 순차적으로 적용
    Eigen::Matrix3d R = Ry * Rx;

    Vector3d xyz(x_nominal_b, y_nominal_b, z_nominal_b);
    Vector3d rot_xyz = R*xyz;

    std::cout << rot_xyz << std::endl;


    // nominal_stance_.at(LF) <<  rot_xyz(0),   rot_xyz(1), ( rot_xyz(2) );
    // nominal_stance_.at(RF) <<  rot_xyz(0),  -rot_xyz(1), ( rot_xyz(2)  );
    // nominal_stance_.at(LH) << -rot_xyz(0),   rot_xyz(1), ( rot_xyz(2) );
    // nominal_stance_.at(RH) << -rot_xyz(0),  -rot_xyz(1), ( rot_xyz(2)  );

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.25, 0.20, 0.1;
  }
};

/**
 * @brief The Dynamics of the quadruped robot HyQ.
 */
class HyqDynamicModel : public SingleRigidBodyDynamics {
public:
  HyqDynamicModel() : SingleRigidBodyDynamics(40,
                      4.26, 8.97, 9.88, -0.0063, 0.193, 0.0126,
                      4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_ */
