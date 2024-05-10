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

#include <towr/nlp_formulation.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/phase_durations.h>

#include <towr/constraints/tension_constraint.h>          // NEW 

#include <towr/constraints/base_motion_constraint.h>
#include <towr/constraints/dynamic_constraint.h>
#include <towr/constraints/force_constraint.h>
#include <towr/constraints/range_of_motion_constraint.h>
#include <towr/constraints/swing_constraint.h>
#include <towr/constraints/terrain_constraint.h>
#include <towr/constraints/total_duration_constraint.h>
#include <towr/constraints/spline_acc_constraint.h>

#include <towr/costs/node_cost.h>
#include <towr/variables/nodes_variables_all.h>


#include <towr/constraints/base_motion_get.h>             // NEW
#include <towr/costs/base_node_cost.h>                    // NEW
#include <towr/costs/pitch_node_cost.h>                   // NEW 
// #include <towr_ros/towr_ros_interface.h>    // NEW   NEW 

#include <iostream>

namespace towr {

NlpFormulation::NlpFormulation ()
{
  using namespace std;
  cout << "\n";
  cout << "************************************************************\n";
  cout << " TOWR - Trajectory Optimization for Walking Robots (v1.4)\n";
  cout << "                \u00a9 Alexander W. Winkler\n";
  cout << "           https://github.com/ethz-adrl/towr\n";
  cout << "************************************************************";
  cout << "\n\n";
}

NlpFormulation::VariablePtrVec
NlpFormulation::GetVariableSets (SplineHolder& spline_holder)
{
  VariablePtrVec vars;

  auto base_motion = MakeBaseVariables();
  vars.insert(vars.end(), base_motion.begin(), base_motion.end());

  auto ee_motion = MakeEndeffectorVariables();
  vars.insert(vars.end(), ee_motion.begin(), ee_motion.end());

  auto ee_force = MakeForceVariables();
  vars.insert(vars.end(), ee_force.begin(), ee_force.end());

  auto contact_schedule = MakeContactScheduleVariables();
  // can also just be fixed timings that aren't optimized over, but still added
  // to spline_holder.

  // auto ee_tension = MakeTensionVariables();                                          // NEW -> add rope tension
  // vars.insert(vars.end(), ee_tension.begin(), ee_tension.end());

  auto ee_tension_test = MakeTensionVariablesTEST();                                          // NEW TEST  TEST
  vars.insert(vars.end(), ee_tension_test.begin(), ee_tension_test.end());


  if (params_.IsOptimizeTimings()) {
    vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
  }

  // stores these readily constructed spline
  spline_holder = SplineHolder(base_motion.at(0), // linear
                               base_motion.at(1), // angular
                               params_.GetBasePolyDurations(),
                               ee_motion,
                               ee_force,
                               contact_schedule,
                              //  ee_tension,              // rope tension           NEW
                               ee_tension_test.at(0),              // rope tension           NEW   TEST TEST TEST
                              //  ee_tension_test.at(1),              // rope tension           NEW   TEST TEST TEST
                               params_.IsOptimizeTimings());
  return vars;
}

std::vector<NodesVariables::Ptr>
NlpFormulation::MakeBaseVariables () const
{
  std::vector<NodesVariables::Ptr> vars;

  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  auto spline_lin = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);

  double x = final_base_.lin.p().x();
  double y = final_base_.lin.p().y();
  double z = terrain_->GetHeight(x,y) - model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x, y, z);

  spline_lin->SetByLinearInterpolation(initial_base_.lin.p(), final_pos, params_.GetTotalTime());
  spline_lin->AddStartBound(kPos, {X,Y,Z}, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, {X,Y,Z}, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, params_.bounds_final_lin_pos_,   final_base_.lin.p());
  spline_lin->AddFinalBound(kVel, params_.bounds_final_lin_vel_, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_ang_nodes);
  spline_ang->SetByLinearInterpolation(initial_base_.ang.p(), final_base_.ang.p(), params_.GetTotalTime());
  spline_ang->AddStartBound(kPos, {X,Y,Z}, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, {X,Y,Z}, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos, params_.bounds_final_ang_pos_, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, params_.bounds_final_ang_vel_, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeEndeffectorVariables () const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  // Endeffector Motions
  double T = params_.GetTotalTime();
  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto nodes = std::make_shared<NodesVariablesEEMotion>(
                                              params_.GetPhaseCount(ee),
                                              params_.ee_in_contact_at_start_.at(ee),
                                              id::EEMotionNodes(ee),
                                              params_.ee_polynomials_per_swing_phase_);

    // initialize towards final footholds
    double yaw = final_base_.ang.p().z();
    Eigen::Vector3d euler(0.0, 0.0, yaw);
    Eigen::Matrix3d w_R_b = EulerConverter::GetRotationMatrixBaseToWorld(euler);
    Vector3d final_ee_pos_W = final_base_.lin.p() + w_R_b*model_.kinematic_model_->GetNominalStanceInBase().at(ee);
    double x = final_ee_pos_W.x();
    double y = final_ee_pos_W.y();
    double z = terrain_->GetHeight(x,y);
    nodes->SetByLinearInterpolation(initial_ee_W_.at(ee), Vector3d(x,y,z), T);

    nodes->AddStartBound(kPos, {X,Y,Z}, initial_ee_W_.at(ee));
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeForceVariables () const
{
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  double T = params_.GetTotalTime();
  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto nodes = std::make_shared<NodesVariablesEEForce>(
                                              params_.GetPhaseCount(ee),
                                              params_.ee_in_contact_at_start_.at(ee),
                                              id::EEForceNodes(ee),
                                              params_.force_polynomials_per_stance_phase_);

    // initialize with mass of robot distributed equally on all legs
    double m = model_.dynamic_model_->m();
    double g = model_.dynamic_model_->g();

    Vector3d f_stance(0.0, 0.0, m*g/params_.GetEECount());
    nodes->SetByLinearInterpolation(f_stance, f_stance, T); // stay constant
    vars.push_back(nodes);
  }

  return vars;
}

std::vector<PhaseDurations::Ptr>
NlpFormulation::MakeContactScheduleVariables () const
{
  std::vector<PhaseDurations::Ptr> vars;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto var = std::make_shared<PhaseDurations>(ee,
                                                params_.ee_phase_durations_.at(ee),
                                                params_.ee_in_contact_at_start_.at(ee),
                                                params_.bound_phase_duration_.first,
                                                params_.bound_phase_duration_.second);
    vars.push_back(var);
  }

  return vars;
}


std::vector<NodesVariables::Ptr>
NlpFormulation::MakeTensionVariablesTEST () const
{
  std::vector<NodesVariables::Ptr> vars;

  int n_nodes = params_.GetBasePolyDurations().size() + 1;
  
  auto spline_tension_lin_ = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::ee_tension_lin_nodes);
  // auto spline_tension_ang_ = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::ee_tension_ang_nodes);

  auto spline_base_lin_ = std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);
  
  double x_fin = final_base_.lin.p().x();
  double y_fin = final_base_.lin.p().y();
  double z_fin = terrain_->GetHeight(x_fin,y_fin) - model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x_fin, y_fin, z_fin);

  double m = model_.dynamic_model_->m();
  double g = model_.dynamic_model_->g();

  // double phi = -0.1;     // roll angle

  double phi = 0.445;     // roll angle                                                     pitch앵글일지도?
  double theta = -0.5;   // ptich angle  original = 0.1  //  test revise = -0.1
  // double mu = terrain_->GetFrictionCoeff();      // friction coef = 0.5
  double mu = 0.5;      // friction coef = 0.5

  // x축을 기준으로 phi만큼 회전하는 변환
  Eigen::Matrix3d Rx = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()).toRotationMatrix();

  // y축을 기준으로 theta만큼 회전하는 변환
  Eigen::Matrix3d Ry = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  // x축 회전 뒤 y축 회전을 순차적으로 적용
  Eigen::Matrix3d R = Ry * Rx;
  
  Vector3d gravity_force(0.0, 0.0, -m*g);             // mg in world frame 
  Vector3d tilt_force = R * gravity_force;            // tilted gravity force
  Vector3d f, PointA, PointB, PointBB, ascender_offset, diff_vector, ori_vector;            // 이 부분을 각도에 따라서 장력값 변경 가능하게 계산하기 
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
  ascender_offset(2) = 0.1;                      // ascender_offset z  0.2915
  Vector3d pos_offset(-0.13, -0.09, 0.58);            // Vector3d pos_offset(0.0, -0.04, 0.61);
  
  PointA = spline_base_lin_->GetValues() + ascender_offset + pos_offset;   // 
  std::cout << "nlp_form in PointA is " << PointA.transpose() << std::endl;               // 포인터 값 받는 과정에서 문제 생긴듯 231219 20:00

  // PointA = formulation_.initial_base_.lin.at(kPos) + ascender_offset;
  // std::cout << "nlp_form in PointA is " << (std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes)->GetValues()+ascender_offset+pos_offset).transpose() << std::endl;               // 포인터 값 받는 과정에서 문제 생긴듯 231219 20:00
  PointB(0) = 3.0;                                  // 로프 고정점 위치 x
  PointB(1) = 0.0;                                  // 로프 고정점 위치 y
  PointB(2) = 3.0;                                  // 로프 고정점 위치 z

  // PointBB(0) = 3.0;                                  // 로프 고정점 위치 x
  // PointBB(1) = 0.0;                                  // 로프 고정점 위치 y
  // PointBB(2) = 4.0;                                  // 로프 고정점 위치 z

  diff_vector = PointB - PointA;
  std::cout << "Init Vector : " << diff_vector.transpose() << std::endl;
  double vec_mag = sqrt(pow(diff_vector(0),2) + pow(diff_vector(1),2) + pow(diff_vector(2),2));
  double tension_mag = sqrt(pow(f_stance(0),2) + pow(f_stance(1),2) + pow(f_stance(2),2));

  Vector3d diff_vector_fin = PointB - (final_pos+ascender_offset);
  std::cout << "Final Vector : " << diff_vector_fin.transpose() << std::endl;
  double vec_mag_fin = sqrt(pow(diff_vector_fin(0),2) + pow(diff_vector_fin(1),2) + pow(diff_vector_fin(2),2));


  ori_vector = diff_vector/vec_mag;                // 로프와 로봇 베이스 사이의 방향벡터
  Vector3d ori_vector_fin = diff_vector_fin/vec_mag_fin;      // final pos에서의 방향벡터

  f_stance = ori_vector*tension_mag;               // 방향벡터 * 텐션 크기
  // f_stance = f_stance + PointA;                    // test

  Vector3d f_final = ori_vector_fin*tension_mag;
  // Vector3d A_fin(2.1, 0, 0);
  // f_final = f_final + PointA + A_fin;

  std::cout << "nlp_formulation " << f_stance.transpose() << std::endl;

  // spline_tension_lin->SetConstantValue(initial_tension_.lin.p(), f_stance, params_.GetTotalTime());                 // 텐션 일정한 값 주도록 하는것
  spline_tension_lin_->SetByLinearInterpolation(f_stance, f_final, params_.GetTotalTime());           // initial_tension_.lin.p() ????
  // spline_tension_lin_->AddStartBound(kPos, {X,Y,Z}, initial_tension_.lin.p());
  // spline_tension_lin_->AddFinalBound(kPos, params_.bounds_final_tension_lin_, initial_tension_.lin.p());

  // spline_tension_lin_->AddStartBound(kPos, {X,Y,Z}, f_stance);
  // spline_tension_lin_->AddFinalBound(kPos, params_.bounds_final_tension_lin_, f_stance);
  vars.push_back(spline_tension_lin_);

  // tension의 방향을 결정하는 부분
  
  // spline_tension_ang_->SetByLinearInterpolation(initial_tension_.ang.p(), final_tension_.ang.p(), params_.GetTotalTime());

  // 장력의 방향 범위 지정? 
  // spline_tension_ang_->AddStartBound(kPos, {X,Y,Z}, initial_tension_.ang.p());
  // spline_tension_ang_->AddFinalBound(kPos, params_.bounds_final_tension_ang_, final_tension_.ang.p());

  // vars.push_back(spline_tension_ang_);

  return vars;
}



NlpFormulation::ContraintPtrVec
NlpFormulation::GetConstraints(const SplineHolder& spline_holder) const
{
  ContraintPtrVec constraints;
  for (auto name : params_.constraints_)
    for (auto c : GetConstraint(name, spline_holder))
      constraints.push_back(c);

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::GetConstraint (Parameters::ConstraintName name,
                           const SplineHolder& s) const
{
  switch (name) {
    case Parameters::Dynamic:        return MakeDynamicConstraint(s);
    case Parameters::EndeffectorRom: return MakeRangeOfMotionBoxConstraint(s);
    case Parameters::BaseRom:        return MakeBaseRangeOfMotionConstraint(s);
    case Parameters::TotalTime:      return MakeTotalTimeConstraint();
    case Parameters::Terrain:        return MakeTerrainConstraint();
    case Parameters::Force:          return MakeForceConstraint();
    case Parameters::Swing:          return MakeSwingConstraint();
    case Parameters::BaseAcc:        return MakeBaseAccConstraint(s);

    case Parameters::BaseMotion: return MakeBaseMotionConstraint(s);   // NEW NEW 
    // case Parameters::Tension: return MakeTensionConstraint(s);         // NEW NEW 

    default: throw std::runtime_error("constraint not defined!");
  }
}

// NlpFormulation::ContraintPtrVec                                                   // NEW NEW NEW NEW NEW NEW 
// NlpFormulation::MakeTensionConstraint (const SplineHolder& s) const
// {
//   return {std::make_shared<TensionConstraint>(params_.GetTotalTime(),
//                                                  params_.dt_constraint_base_motion_,
//                                                  s,
//                                                  terrain_)};
// }


NlpFormulation::ContraintPtrVec                                                   // NEW NEW NEW NEW NEW NEW 
NlpFormulation::MakeBaseMotionConstraint (const SplineHolder& s) const
{
  return {std::make_shared<BaseMotionConstraint>(params_.GetTotalTime(),
                                                 params_.dt_constraint_base_motion_,
                                                 s,
                                                 terrain_)};
}


NlpFormulation::ContraintPtrVec
NlpFormulation::MakeBaseRangeOfMotionConstraint (const SplineHolder& s) const
{
  return {std::make_shared<BaseMotionConstraint>(params_.GetTotalTime(),
                                                 params_.dt_constraint_base_motion_,
                                                 s,
                                                 terrain_)};
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeDynamicConstraint(const SplineHolder& s) const
{
  auto constraint = std::make_shared<DynamicConstraint>(model_.dynamic_model_,
                                                        params_.GetTotalTime(),
                                                        params_.dt_constraint_dynamic_,
                                                        s);
  return {constraint};
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeRangeOfMotionBoxConstraint (const SplineHolder& s) const
{
  ContraintPtrVec c;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto rom = std::make_shared<RangeOfMotionConstraint>(model_.kinematic_model_,
                                                         params_.GetTotalTime(),
                                                         params_.dt_constraint_range_of_motion_,
                                                         ee,
                                                         s);
    c.push_back(rom);
  }

  return c;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeTotalTimeConstraint () const
{
  ContraintPtrVec c;
  double T = params_.GetTotalTime();

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto duration_constraint = std::make_shared<TotalDurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeTerrainConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto c = std::make_shared<TerrainConstraint>(terrain_, id::EEMotionNodes(ee));
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeForceConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto c = std::make_shared<ForceConstraint>(terrain_,
                                               params_.force_limit_in_normal_direction_,
                                               ee);
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeSwingConstraint () const
{
  ContraintPtrVec constraints;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    auto swing = std::make_shared<SwingConstraint>(id::EEMotionNodes(ee));
    constraints.push_back(swing);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeBaseAccConstraint (const SplineHolder& s) const
{
  ContraintPtrVec constraints;

  constraints.push_back(std::make_shared<SplineAccConstraint>
                        (s.base_linear_, id::base_lin_nodes));

  constraints.push_back(std::make_shared<SplineAccConstraint>
                        (s.base_angular_, id::base_ang_nodes));

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::GetCosts() const
{
  ContraintPtrVec costs;
  for (const auto& pair : params_.costs_)
    for (auto c : GetCost(pair.first, pair.second))
      costs.push_back(c);

  return costs;
}

NlpFormulation::CostPtrVec
NlpFormulation::GetCost(const Parameters::CostName& name, double weight) const
{
  switch (name) {
    case Parameters::ForcesCostID:   return MakeForcesCost(weight);
    case Parameters::EEMotionCostID: return MakeEEMotionCost(weight);
    case Parameters::BaseCostID: return MakeBaseCost(weight);                       // NEW   NEW   NEW   NEW   NEW   NEW   NEW   NEW   
    case Parameters::PitchCostID: return MakeBaseCost(weight);                       // NEW   NEW   NEW   NEW   NEW   NEW   NEW   NEW  
    default: throw std::runtime_error("cost not defined!");
  }
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeForcesCost(double weight) const
{
  CostPtrVec cost;

  for (int ee=0; ee<params_.GetEECount(); ee++)
    cost.push_back(std::make_shared<NodeCost>(id::EEForceNodes(ee), kPos, Z, weight));

  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeEEMotionCost(double weight) const
{
  CostPtrVec cost;

  for (int ee=0; ee<params_.GetEECount(); ee++) {
    cost.push_back(std::make_shared<NodeCost>(id::EEMotionNodes(ee), kVel, X, weight));
    cost.push_back(std::make_shared<NodeCost>(id::EEMotionNodes(ee), kVel, Y, weight));
  }

  return cost;
}

// Base Motion Cost PART   Base Motion Cost PART   Base Motion Cost PART   Base Motion Cost PART   Base Motion Cost PART   
NlpFormulation::CostPtrVec
NlpFormulation::MakeBaseCost(double weight) const
{
  CostPtrVec cost;

  // for (int ee=0; ee<params_.GetEECount(); ee++) {
    // cost.push_back(std::make_shared<NodeCost>(id::EEBaseLinNodes(ee), kPos, Z, weight));            // z-axis CoM Position cost    BaseNodeCost
    // std::cout << std::make_shared<NodeCost>(id::EEBaseLinNodes(ee), kPos, Z, weight) << std::endl;
    // cost.push_back(std::make_shared<PitchNodeCost>(id::EEBaseLinNodes(ee), kPos, Y, weight));            // CoM pitch angle cost
  // }
  cost.push_back(std::make_shared<PitchNodeCost>(id::base_lin_nodes, kPos, Z, weight));
  return cost;
}


NlpFormulation::CostPtrVec
NlpFormulation::MakePitchCost(double weight) const
{ 
  CostPtrVec cost;

  // for (int ee=0; ee<params_.GetEECount(); ee++) {
  //   cost.push_back(std::make_shared<BaseNodeCost>(id::EEBaseLinNodes(ee), kPos, Z, weight));            // z-axis CoM Position cost
  //   cost.push_back(std::make_shared<PitchNodeCost>(id::EEBaseAngNodes(), kPos, X, weight));            // CoM pitch angle cost
  //   std::cout << std::make_shared<NodeCost>(id::EEBaseAngNodes(), kPos, X, weight) << std::endl;
    
  // } 
              // CoM pitch angle cost   PitchNodeCost
  cost.push_back(std::make_shared<PitchNodeCost>(id::base_ang_nodes, kPos, Y, weight));

  return cost;
}


} /* namespace towr */
