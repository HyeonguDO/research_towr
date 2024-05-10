#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/variables/cartesian_dimensions.h>

#include <iostream>

namespace towr {

// some Eigen helper functions
static Eigen::Matrix3d BuildInertiaTensor( double Ixx, double Iyy, double Izz,
                                           double Ixy, double Ixz, double Iyz)
{
  Eigen::Matrix3d I;
  I <<  Ixx, -Ixy, -Ixz,
       -Ixy,  Iyy, -Iyz,
       -Ixz, -Iyz,  Izz;
  return I;
}

// builds a cross product matrix out of "in", so in x v = X(in)*v
SingleRigidBodyDynamics::Jac
Cross(const Eigen::Vector3d& in)
{
  SingleRigidBodyDynamics::Jac out(3,3);

  out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
  out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
  out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

  return out;
}

SingleRigidBodyDynamics::SingleRigidBodyDynamics (double mass,
                                  double Ixx, double Iyy, double Izz,
                                  double Ixy, double Ixz, double Iyz,
                                  int ee_count)
   : SingleRigidBodyDynamics(mass,
                     BuildInertiaTensor(Ixx, Iyy, Izz, Ixy, Ixz, Iyz),
                     ee_count)
{
}

SingleRigidBodyDynamics::SingleRigidBodyDynamics (double mass, const Eigen::Matrix3d& inertia_b,
                                  int ee_count)
    :DynamicModel(mass, ee_count)
{
  I_b = inertia_b.sparseView();
}

SingleRigidBodyDynamics::BaseAcc
SingleRigidBodyDynamics::GetDynamicViolation () const
{
  Vector3d f_sum, tau_sum, ascender_offset(3);   // rope tension -> T ,  ascender_offset(초기화 되지 않은 크기 3인 벡터)은 CoM 과 Rope 사이의 거리 
  f_sum.setZero(); tau_sum.setZero(); 
  ascender_offset(0) = 0.156;                       // ascender_offset에 값 대입 x
  ascender_offset(1) = 0.0;                         // ascender_offset에 값 대입 Y
  ascender_offset(2) = 0.15;                      // ascender_offset에 값 대입 Z   0.2915

  // ascender_offset(0) = 0.0;                       // ascender_offset에 값 대입 x
  // ascender_offset(1) = 0.0;                         // ascender_offset에 값 대입 Y
  // ascender_offset(2) = 0.0;                      // ascender_offset에 값 대입 Z

  for (int ee=0; ee<ee_pos_.size(); ++ee) {
    Vector3d f = ee_force_.at(ee);     // end-effector의 값을 저장
    
    tau_sum += f.cross(com_pos_ - ee_pos_.at(ee));   // contact force X (CoM position - foot position) ->  cross product  뉴턴-오일러 식에서 토크에 대한 항
    tau_sum += ee_tension_test_.cross(com_pos_ - (com_pos_ + ascender_offset));           // TEST TEST NEW NEW 
    f_sum   += f;  // contact force들의 합    뉴턴-오일러 식에서 힘에 대한 항
  }
  

  // for (int ee_t=0; ee_t<ee_tension_.size(); ee_t++) {                   // NEW   -> have to TEST!!
  //   Vector3d f_t = ee_tension_.at(ee_t);
  //   tau_sum += f_t.cross(com_pos_ - (com_pos_ + ascender_offset) );     // CoM + ascender offset
  //   f_sum = f_sum + f_t; 
  // }
   
  
  // express inertia matrix in world frame based on current body orientation
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  BaseAcc acc;
  acc.segment(AX, k3D) = I_w*omega_dot_
                         + Cross(omega_)*(I_w*omega_)
                         - tau_sum;
  acc.segment(LX, k3D) = m()*com_acc_
                         - f_sum
                         - Vector3d(0.0, 0.0, -m()*g()); // gravity force
  return acc;
}

SingleRigidBodyDynamics::Jac
SingleRigidBodyDynamics::GetJacobianWrtBaseLin (const Jac& jac_pos_base_lin,
                                        const Jac& jac_acc_base_lin) const
{
  // // build the com jacobian
  // int n = jac_pos_base_lin.cols();

  // Jac jac_tau_sum(k3D, n);
  // for (const Vector3d& f : ee_force_) {
  //   Jac jac_tau = Cross(f)*jac_pos_base_lin;
  //   jac_tau_sum += jac_tau;
  // }


  // NEW NEW NEW NEW
  int n = jac_pos_base_lin.cols() + 1;     // Return the number of columns(4개다리) + 1 (장력항)
  // int n = jac_pos_base_lin.cols() ;                                                                                            // TEST

  Jac jac_tau_sum(k3D, n);    // 3 row, n개 만큼의 열인 자코비안 생성
  Jac sub_jac(k3D, 1);        // rope에 대한 자코비안 계산 대입용 
  
  for (const Vector3d& f : ee_force_) {
    Jac jac_tau = Cross(f)*jac_pos_base_lin;   // 토크 공식은 torque = r x F 인데 F x r 로 썼으므로 앞에 - 필요함
    jac_tau_sum += jac_tau;
  }
  sub_jac = Cross(ee_tension_test_)*jac_pos_base_lin;  
  jac_tau_sum += sub_jac;


  // for (const Vector3d& f_tension : ee_tension_) {                                                                 // NEW (have to TEST)
  //   sub_jac = Cross(f_tension)*jac_pos_base_lin;  
  //   jac_tau_sum += sub_jac;
  // }  


  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = -jac_tau_sum;
  jac.middleRows(LX, k3D) = m()*jac_acc_base_lin;

  return jac;
}

SingleRigidBodyDynamics::Jac
SingleRigidBodyDynamics::GetJacobianWrtBaseAng (const EulerConverter& base_euler,
                                        double t) const
{
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  // Derivative of R*I_b*R^T * wd
  // 1st term of product rule (derivative of R)
  Vector3d v11 = I_b*w_R_b_.transpose()*omega_dot_;
  Jac jac11 = base_euler.DerivOfRotVecMult(t, v11, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac12 = w_R_b_.sparseView()*I_b*base_euler.DerivOfRotVecMult(t, omega_dot_, true);

  // 3rd term of product rule (derivative of wd)
  Jac jac_ang_acc = base_euler.GetDerivOfAngAccWrtEulerNodes(t);
  Jac jac13 = I_w * jac_ang_acc;
  Jac jac1 = jac11 + jac12 + jac13;


  // Derivative of w x Iw
  // w x d_dn(R*I_b*R^T*w) -(I*w x d_dnw)
  // right derivative same as above, just with velocity instead acceleration
  Vector3d v21 = I_b*w_R_b_.transpose()*omega_;
  Jac jac21 = base_euler.DerivOfRotVecMult(t, v21, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac22 = w_R_b_.sparseView()*I_b*base_euler.DerivOfRotVecMult(t, omega_, true);

  // 3rd term of product rule (derivative of omega)
  Jac jac_ang_vel = base_euler.GetDerivOfAngVelWrtEulerNodes(t);
  Jac jac23 = I_w * jac_ang_vel;

  Jac jac2 = Cross(omega_)*(jac21+jac22+jac23) - Cross(I_w*omega_)*jac_ang_vel;


  // Combine the two to get sensitivity to I_w*w + w x (I_w*w)
  int n = jac_ang_vel.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = jac1 + jac2;

  return jac;
}

SingleRigidBodyDynamics::Jac
SingleRigidBodyDynamics::GetJacobianWrtForce (const Jac& jac_force, EE ee, const Jac& jac_tension_force) const            // NEW  
{
  Vector3d r = com_pos_ - ee_pos_.at(ee);   // 발끝 - 무게중심이 r벡터가 맞다. 
  Vector3d r_asender(3);  // Base CoM - 어센더까지의 거리
  r_asender(0) = 0.156;       // x
  r_asender(1) = 0.0;         // y
  r_asender(2) = 0.15;      // z   0.2915

  // r_asender(0) = 0.0;       // x
  // r_asender(1) = 0.0;         // y
  // r_asender(2) = 0.0;      // z 
  
  Jac jac_ascender_tau = -Cross(-r_asender)*jac_tension_force;   // rope tension에 대한 토크 값.  ->>  CoM - ( CoM + r_ascender ) = -r_ascender

  // Jac jac_ascender_tau = -Cross(com_pos_)*jac_tension_force;   // TEST

  Jac jac_tau = -Cross(r)*jac_force;        // 토크의 정의에 의해 tau = r X F 앞에 음수는 위에 문제를 바로잡기 위함.
 
  Jac temp_jac(3, 5); // temp_jac는 3x5 행렬이므로 이렇게 선언합니다.                                                             TEST
  Jac temp_force(3, 5); // temp_force는 3x5 행렬이므로 이렇게 선언합니다.                                                         TEST

  Jac non_const_jac_force = jac_force;                              // const 자료형이어서 임시로 변수 저장용으로 만듦
  Jac non_const_jac_tension_force = jac_tension_force;              // const 자료형이어서 임시로 변수 저장용으로 만듦

  // jac_tau의 내용을 temp_jac에 복사
  for (int row = 0; row < jac_tau.rows(); ++row) {
    for (int col = 0; col < jac_tau.cols(); ++col) {
      temp_jac.coeffRef(row, col) = jac_tau.coeffRef(row, col);
    }
  }

  // jac_ascender_tau의 내용을 temp_jac에 복사
  for (int row = 0; row < jac_ascender_tau.rows(); ++row) {
    temp_jac.coeffRef(row, 4) = jac_ascender_tau.coeffRef(row, 0); // 마지막 열에 복사
  }


  // jac_force 내용을 temp_force에 복사
  for (int row = 0; row < non_const_jac_force.rows(); ++row) {
    for (int col = 0; col < non_const_jac_force.cols(); ++col) {
      temp_force.coeffRef(row, col) = non_const_jac_force.coeffRef(row, col);
    }
  }

  // jac_tension_force의 내용을 temp_jac에 복사
  for (int row = 0; row < non_const_jac_tension_force.rows(); ++row) {
    temp_force.coeffRef(row, 4) = non_const_jac_tension_force.coeffRef(row, 0); // 마지막 열에 복사
  }


  int n = jac_force.cols() + jac_tension_force.cols();
  Jac jac(k6D, n);

  // jac.middleRows(AX, k3D) = -jac_tau;
  jac.middleRows(AX, k3D) = -temp_jac;

  // jac.middleRows(LX, k3D) = -jac_force;
  jac.middleRows(LX, k3D) = -temp_force;


  return jac;
}

SingleRigidBodyDynamics::Jac
SingleRigidBodyDynamics::GetJacobianWrtEEPos (const Jac& jac_ee_pos, EE ee) const
{

  Vector3d f = ee_force_.at(ee);
  // Vector3d f_t = ee_tension_.at(ee);    // rope tension이 발이 바뀔 때 마다 계속 받는게 맞는지?
  Vector3d f_t = ee_tension_test_;    // rope tension TEST TEST
  // Vector3d f_t(80.11, -21.02, 5.97); 
  Jac jac_ascender_pos(3, 1);  // Base CoM - 어센더까지의 거리
  jac_ascender_pos.coeffRef(0,0) = 0.156;     // x
  jac_ascender_pos.coeffRef(1,0) = 0.0;     // y
  jac_ascender_pos.coeffRef(2,0) = 0.15;     // z   0.2915

  // jac_ascender_pos.coeffRef(0,0) = 0.0;     // x
  // jac_ascender_pos.coeffRef(1,0) = 0.0;     // y
  // jac_ascender_pos.coeffRef(2,0) = 0.0;     // z 

  Jac jac_ascender_tau = Cross(f_t)*(-jac_ascender_pos);   // CoM - ( CoM + r_ascender ) = -r_ascender  와 같은 이유  Com 에서 ascender offset을 빼면 원래 토크 계산식과 반대이기 때문에 -가 붙고 시작함에 유의
  Jac jac_tau = Cross(f)*(-jac_ee_pos);   //토크의 정의 tau = r X F 와 다르게 외적을 반대로 했으므로 앞에 (-)가 붙는다.

  Jac temp_jac(3,5);                                                                                                                    // TEST
  
  // jac_tau의 내용을 temp_jac에 복사
  for (int row = 0; row < jac_tau.rows(); ++row) {
    for (int col = 0; col < jac_tau.cols(); ++col) {
      temp_jac.coeffRef(row, col) = jac_tau.coeffRef(row, col);
    }
  }

  // jac_ascender_tau의 내용을 temp_jac에 복사
  for (int row = 0; row < jac_ascender_tau.rows(); ++row) {
    temp_jac.coeffRef(row, 4) = jac_ascender_tau.coeffRef(row, 0); // 마지막 열에 복사
  }

  int n = jac_tau.cols()+jac_ascender_tau.cols();

  Jac jac(k6D, n);
  // jac.middleRows(AX, k3D) = -jac_tau;
  jac.middleRows(AX, k3D) = -temp_jac;

  // linear dynamics don't depend on endeffector position.
  return jac;
}

} /* namespace towr */