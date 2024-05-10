#include <towr/constraints/dynamic_constraint.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>

#include <iostream>


namespace towr {

DynamicConstraint::DynamicConstraint (const DynamicModel::Ptr& m,
                                      double T, double dt,
                                      const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(T, dt, "dynamic")
{
  model_ = m;

  // link with up-to-date spline variables  
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_forces_    = spline_holder.ee_force_;
  ee_motion_    = spline_holder.ee_motion_;

  // ee_tensions_ = spline_holder.ee_tension_;                            // NEW
  ee_tensions_test_lin_ = spline_holder.ee_tension_test_lin_;                            // NEW  TEST
  // ee_tensions_test_ang_ = EulerConverter(spline_holder.ee_tension_test_ang_);                            // NEW  TEST
  SetRows(GetNumberOfNodes()*k6D);
}

int
DynamicConstraint::GetRow (int k, Dim6D dimension) const
{
  return k6D*k + dimension;
} 

void
DynamicConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const
{
  UpdateModel(t);
  g.segment(GetRow(k,AX), k6D) = model_->GetDynamicViolation();
}

void
DynamicConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const
{
  for (auto dim : AllDim6D)
    bounds.at(GetRow(k,dim)) = ifopt::BoundZero;
}

void
DynamicConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set,
                                            Jacobian& jac) const
{
  UpdateModel(t);

  int n = jac.cols();
  Jacobian jac_model(k6D,n);

  // sensitivity of dynamic constraint w.r.t base variables.
  if (var_set == id::base_lin_nodes) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobianWrtNodes(t,kPos);
    Jacobian jac_base_lin_acc = base_linear_->GetJacobianWrtNodes(t,kAcc);


    jac_model = model_->GetJacobianWrtBaseLin(jac_base_lin_pos,
                                              jac_base_lin_acc);                                                  
  }
  
  if (var_set == id::base_ang_nodes) {
    jac_model = model_->GetJacobianWrtBaseAng(base_angular_, t);
  }

  // if (var_set == id::ee_tension_ang_nodes) {                                                        // NEW   NEW   NEW  
  //   jac_model = model_->GetJacobianWrtBaseAng(ee_tensions_test_ang_, t);
  // }

  // sensitivity of dynamic constraint w.r.t. endeffector variables
  for (int ee=0; ee<model_->GetEECount(); ++ee) {
    if (var_set == id::EEForceNodes(ee)) {
      Jacobian jac_ee_force = ee_forces_.at(ee)->GetJacobianWrtNodes(t,kPos);

      // Jacobian jac_ee_tension = ee_tensions_.at(ee)->GetJacobianWrtNodes(t,kPos);                // NEW -> get tension Jacobian 
      Jacobian jac_ee_tension = ee_tensions_test_lin_->GetJacobianWrtNodes(t,kPos);                // NEW -> get tension Jacobian  TEST TEST

      jac_model = model_->GetJacobianWrtForce(jac_ee_force, ee, jac_ee_tension);                // NEW -> add tension Jacobian, to calculate wheelleg_SRBD    
    }

    if (var_set == id::EEMotionNodes(ee)) {
      Jacobian jac_ee_pos = ee_motion_.at(ee)->GetJacobianWrtNodes(t,kPos);

      jac_model = model_->GetJacobianWrtEEPos(jac_ee_pos, ee);
    }

    if (var_set == id::EESchedule(ee)) {
      Jacobian jac_f_dT = ee_forces_.at(ee)->GetJacobianOfPosWrtDurations(t);

      // Jacobian jac_tension_dT = ee_tensions_.at(ee)->GetJacobianOfPosWrtDurations(t);            // NEW -> get tension jacobian derivative
      Jacobian jac_tension_dT = ee_tensions_test_lin_->GetJacobianOfPosWrtDurations(t);            // NEW -> get tension jacobian derivative  TEST TEST

      jac_model += model_->GetJacobianWrtForce(jac_f_dT, ee, jac_tension_dT );                   // NEW -> add tension jacobian derivative    

      Jacobian jac_x_dT = ee_motion_.at(ee)->GetJacobianOfPosWrtDurations(t);

      jac_model +=  model_->GetJacobianWrtEEPos(jac_x_dT, ee);  
    }
  }

  jac.middleRows(GetRow(k,AX), k6D) = jac_model;
  // std::cout << jac << std::endl;
}

void
DynamicConstraint::UpdateModel (double t) const
{
  auto com = base_linear_->GetPoint(t);

  // auto ascender_com = ascender_linear_->GetPoint(t);                                            // NEW -> get ascender position in world frame

  Eigen::Matrix3d w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
  Eigen::Vector3d omega = base_angular_.GetAngularVelocityInWorld(t);
  Eigen::Vector3d omega_dot = base_angular_.GetAngularAccelerationInWorld(t);

  int n_ee = model_->GetEECount();
  std::vector<Eigen::Vector3d> ee_pos;
  std::vector<Eigen::Vector3d> ee_force;

  // std::vector<Eigen::Vector3d> ee_tension_vector;                                                     // NEW -> tension vector initialization
  auto ee_tension_vector = ee_tensions_test_lin_->GetPoint(t);                                   // NEW -> tension vector initialization   TEST    TEST

  for (int ee=0; ee<n_ee; ++ee) {
    ee_force.push_back(ee_forces_.at(ee)->GetPoint(t).p());
    ee_pos.push_back(ee_motion_.at(ee)->GetPoint(t).p());

    // ee_tension_vector.push_back(ee_tensions_.at(ee)->GetPoint(t).p());                                 // NEW -> get rope tension
  }

  // model_->SetCurrent(com.p(), com.a(), w_R_b, omega, omega_dot, ee_force, ee_pos, ee_tension_vector);                    // NEW  
  model_->SetCurrent(com.p(), com.a(), w_R_b, omega, omega_dot, ee_force, ee_pos, ee_tension_vector.p());                    // TEST 
}

} /* namespace towr */
