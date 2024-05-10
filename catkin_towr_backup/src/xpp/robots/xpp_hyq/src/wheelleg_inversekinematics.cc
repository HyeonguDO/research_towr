#include <xpp_hyq/wheelleg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>

#include <iostream>



namespace xpp
{
  
  WheellegInverseKinematics::Vector3d
  WheellegInverseKinematics::GetJointAngles(const Vector3d &ee_pos_B, LegNum bend) const
  {
    double q_hr, q_hp, q_k;                   // joint
    double k1;                                // 각 관절별로 계산할 때 필요한 변수
    double hr_alpha, hr_beta, hp_phi, hp_psi; // 각 관절별로 계산할 때 필요한 변수

    Eigen::Vector3d hr_Pos_wheel;             // wheel-center vector (from q_hr to wheel)
    Eigen::Matrix3d R;                         // rotation Matrix about q_hr

    Eigen::Vector4d n1, n2, n3, n4;                   // 다리의 방향을 결정하기 위한 벡터

    n1 << 1, -1, 1, -1;                       // sym1
    n2 << 1, 1, -1, -1;                       // sym4
    n3 << -1, -1, 1, 1;                       
    n4 << 1, -1, -1, 1;                       

    hr_Pos_wheel = ee_pos_B;
    
    // std::cout << "IK_local_pos : " << hr_Pos_wheel.transpose() << std::endl;  // 정상확인완료

    hr_beta = acos((length_d1 + length_d2) / sqrt(pow(hr_Pos_wheel[X], 2) + pow(hr_Pos_wheel[Y], 2)));
    hr_alpha = atan2( abs(hr_Pos_wheel[X]), abs(hr_Pos_wheel[Y]) );

    if (bend == Leg1)
    {   
      q_hr = -(hr_beta - hr_alpha);
    }
    if (bend == Leg2)
    {
      q_hr = hr_beta - hr_alpha;  
    }
    if (bend == Leg3)
    {
      q_hr = hr_beta - hr_alpha;  
    }
    if (bend == Leg4)
    {   
      q_hr = -(hr_beta - hr_alpha);
    }



    R << cos(q_hr), -sin(q_hr), 0, sin(q_hr), cos(q_hr), 0, 0, 0, 1;
    hr_Pos_wheel = (R * hr_Pos_wheel).eval();     // Calculate ee_pos including q_hr rotation.

    // std::cout << "IK_local_pos_rotation : " << hr_Pos_wheel.transpose() << std::endl; // good 



    k1 = sqrt( pow(hr_Pos_wheel[X],2) + pow((hr_Pos_wheel[Z]+length_l1),2) );

    // if (bend == Leg1 || bend == Leg4)  // sym4 +--+
    // {
    //   q_k = -acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
    // }
    // else if (bend == Leg2 || bend == Leg3)  // sym4 +--+ 
    // {
    //   q_k = acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
    // }

    // if (bend == Leg1 || bend == Leg4)  // sym2 ++--
    // {
    //   hp_phi = atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
    // }
    // else if (bend == Leg2 || bend == Leg3)  // sym2 ++--
    // {
    //   hp_phi = -atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
    // }

    // hp_psi = atan2( length_l3*sin(q_k), (length_l2+length_l3*cos(q_k)) );
    // // q_hp = hp_phi + hp_psi;

    // q_hp = hp_phi + hp_psi - M_PI/2;   // - M_PI/2 <- 원래 없었음
    
    if (bend == Leg1)  // 
    {
      q_k = -acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
      hp_phi = atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
      hp_psi = atan2( length_l3*sin(q_k), (length_l2+length_l3*cos(q_k)) );
      q_hp = (hp_phi + hp_psi);
    }
    if (bend == Leg2)  // 
    {
      q_k = -acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
      hp_phi = atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
      hp_psi = atan2( length_l3*sin(q_k), (length_l2+length_l3*cos(q_k)) );
      q_hp = hp_phi + hp_psi;   
    }
    if (bend == Leg3)  // 
    {
      q_k = acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
      hp_phi = atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
      hp_psi = atan2( length_l3*sin(q_k), (length_l2+length_l3*cos(q_k)) );
      q_hp = (hp_phi + hp_psi); 
    }
    if (bend == Leg4)  //
    {
      q_k = acos( (pow(k1,2)-pow(length_l2,2)-pow(length_l3,2)) / (2*length_l2*length_l3) );
      hp_phi = atan2( hr_Pos_wheel[X], (hr_Pos_wheel[Z]+length_l1) ) - M_PI;
      hp_psi = atan2( length_l3*sin(q_k), (length_l2+length_l3*cos(q_k)) );
      q_hp = hp_phi + hp_psi;
    }
    
    
    if (q_hp > M_PI)
    {
      q_hp = q_hp - M_PI*2;
    }
    else if (q_hp < -M_PI)
    {
      q_hp = q_hp + M_PI*2;
    }
 
    EnforceLimits(q_hr, hr);
    EnforceLimits(q_hp, hp2);
    EnforceLimits(q_k, k);

    if (bend == Leg1 || bend == Leg2 || bend == Leg3 || bend == Leg4)
      return Vector3d(q_hr, q_hp, q_k); 

  }

  void
  WheellegInverseKinematics::EnforceLimits(double &val, WheellegJointID joint) const
  {
    // totally exaggerated joint angle limits
    const static double q_hr_max = M_PI / 2;
    const static double q_hr_min = -M_PI / 2;

    // const static double q_hp_max = 2 / 3 * M_PI;
    // const static double q_hp_min = -2 / 3 * M_PI;

    const static double q_hp_max = 3 * M_PI;
    const static double q_hp_min = -3 * M_PI;

    const static double q_k_max = 2.4435;  // 140도
    const static double q_k_min = -2.4435; // -140도

    // reduced joint angles for optimization
    static const std::map<WheellegJointID, double> max_range{
        {hr, q_hr_max},
        {hp2, q_hp_max},
        {k, q_k_max}
    };

    // reduced joint angles for optimization
    static const std::map<WheellegJointID, double> min_range{
        {hr, q_hr_min},
        {hp2, q_hp_min},
        {k, q_k_min}
    };

    double max = max_range.at(joint);
    val = val > max ? max : val; // 조건 연산자 (조건) ? A : B -> 조건이 참이면 A 반환 거짓이면 B 반환

    double min = min_range.at(joint);
    val = val < min ? min : val;
  }

} /* namespace xpp */