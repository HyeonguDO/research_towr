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

#include <towr/initialization/gait_generator.h>
#include <towr_ros/towr_ros_interface.h>


namespace towr {

/**
 * @brief An example application of using TOWR together with ROS.
 *
 * Build your own application with your own formulation using the building
 * blocks provided in TOWR and following the example below.
 */
class TowrRosApp : public TowrRosInterface {
public:
  /**
   * @brief Sets the feet to nominal position on flat ground and base above.
   */
  void SetTowrInitialState() override
  {
    auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();

    double z_ground = 0.575;
    formulation_.initial_ee_W_ = nominal_stance_B;

    // double phi = -0.1;     // roll angle

    double phi = 0.38;     // roll angle                                                     pitch앵글일지도?
    double theta = -0.4;   // ptich angle  original = 0.1  //  test revise = -0.1
    double mu = 0.5;      // friction coef = 0.5

    // x축을 기준으로 phi만큼 회전하는 변환
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // y축을 기준으로 theta만큼 회전하는 변환
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // x축 회전 뒤 y축 회전을 순차적으로 적용
    Eigen::Matrix3d R = Ry * Rx;     // pitch roll
    // Eigen::Matrix3d R = Rx * Ry;     // roll pitch

    // std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
    //               [&](Vector3d& p){ p.z() = z_ground; } // feet at 0 height
    // );

    std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
              [&](Vector3d& p) {p.z() = z_ground; p = R*p;}
    );

    // std::for_each(nominal_stance_B.begin(), nominal_stance_B.end(),
    //           [&](Vector3d& pp) {pp = R*pp;}
    // );

    // formulation_.initial_base_.lin.at(kPos) = R*formulation_.initial_base_.lin.at(kPos);

    formulation_.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;              // 초기 자세 position 정의 부분
    formulation_.initial_base_.lin.at(kPos).y() = - nominal_stance_B.front().y();              // 초기 자세 position 정의 부분
    formulation_.initial_base_.lin.at(kPos).x() = - nominal_stance_B.front().x();              // 초기 자세 position 정의 부분

    std::cout << "init_base is : " << formulation_.initial_base_.lin.at(kPos).transpose() << std::endl;
    // std::cout << "nomial is x: " << nominal_stance_B.front().x() << std::endl;
    // std::cout << "nomial is y: " << nominal_stance_B.front().y() << std::endl;
    // std::cout << "nomial is z: " << nominal_stance_B.front().z() << std::endl;

    formulation_.initial_base_.ang.at(kPos) << 0.4, -0.4, 0;      // roll pitxch yaw?

    double m = 40.0;
    double g = 9.81;

    

    Vector3d gravity_force(0.0, 0.0, -m*g);             // mg in world frame 
    Vector3d tilt_force = R * gravity_force;            // tilted gravity force
    Vector3d f, PointA, PointB, ascender_offset, diff_vector, ori_vector;            // 이 부분을 각도에 따라서 장력값 변경 가능하게 계산하기 
    f(0) = m*g*cos(phi)*( sin(theta) - mu*cos(theta) );
    // f(0) = m*g*cos(phi)*( mu*cos(theta) - sin(theta) );
    f(1) = -m*g*sin(phi);
    // f(1) = m*g*sin(phi);
    f(2) = 0;
    Vector3d f_stance = R.transpose()*f;
    f_stance = -f_stance;
    // std::cout << f_stance << std::endl;

    ascender_offset(0) = 0.156;                       // ascender_offset x
    ascender_offset(1) = 0.0;                         // ascender_offset y
    ascender_offset(2) = 0.1;                      // ascender_offset z    0.2915
    PointA = formulation_.initial_base_.lin.at(kPos) + ascender_offset;
    std::cout << "towr_ros PointA is : " << PointA.transpose() << std::endl;   
    PointB(0) = 3.0;                                  // 로프 고정점 위치 x
    PointB(1) = 0.0;                                  // 로프 고정점 위치 y
    PointB(2) = 3.0;                                  // 로프 고정점 위치 z

    diff_vector = PointB - PointA;
    std::cout << diff_vector.transpose() << std::endl;
    double vec_mag = sqrt(pow(diff_vector(0),2) + pow(diff_vector(1),2) + pow(diff_vector(2),2));
    double tension_mag = sqrt(pow(f_stance(0),2) + pow(f_stance(1),2) + pow(f_stance(2),2));

    ori_vector = diff_vector/vec_mag;                // 로프와 로봇 베이스 사이의 방향벡터
    f_stance = ori_vector*tension_mag;               // 방향벡터 * 텐션 크기
    std::cout << "towr_ros_app" << f_stance.transpose() << std::endl;

    formulation_.initial_tension_.lin.at(kPos) = f_stance;
  }

  /**
   * @brief Sets the parameters required to formulate the TOWR problem.
   */
  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    Parameters params;

    // Instead of manually defining the initial durations for each foot and
    // step, for convenience we use a GaitGenerator with some predefined gaits
    // for a variety of robots (walk, trot, pace, ...).
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    auto id_gait   = static_cast<GaitGenerator::Combos>(msg.gait);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      params.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(msg.total_duration, ee));
      params.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }

    // Here you can also add other constraints or change parameters
    // params.constraints_.push_back(Parameters::BaseRom);

    // increases optimization time, but sometimes helps find a solution for
    // more difficult terrain.
    if (msg.optimize_phase_durations)
      params.OptimizePhaseDurations();

    return params;
  }

  /**
   * @brief Sets the paramters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
    // the HA-L solvers are alot faster, so consider installing and using
    solver_->SetOption("linear_solver", "mumps"); // ma27, ma57

    // Analytically defining the derivatives in IFOPT as we do it, makes the
    // problem a lot faster. However, if this becomes too difficult, we can also
    // tell IPOPT to just approximate them using finite differences. However,
    // this uses numerical derivatives for ALL constraints, there doesn't yet
    // exist an option to turn on numerical derivatives for only some constraint
    // sets.
    solver_->SetOption("jacobian_approximation", "exact"); // finite difference-values

    // This is a great to test if the analytical derivatives implemented in are
    // correct. Some derivatives that are correct are still flagged, showing a
    // deviation of 10e-4, which is fine. What to watch out for is deviations > 10e-2.
    // solver_->SetOption("derivative_test", "first-order");

    solver_->SetOption("max_cpu_time", 100.0);
    solver_->SetOption("print_level", 5);

    if (msg.play_initialization)
      solver_->SetOption("max_iter", 0);
    else
      solver_->SetOption("max_iter", 500);
  }
};

} // namespace towr


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_towr_ros_app");
  towr::TowrRosApp towr_app;
  ros::spin();

  return 1;
}
