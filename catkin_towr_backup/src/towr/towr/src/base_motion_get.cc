#include <ifopt/cost_term.h>
#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>
#include <towr/variables/spline_holder.h>
#include <towr/constraints/base_motion_get.h>

namespace towr {


BaseMotionGet::BaseMotionGet (double T, double dt,
                                            const SplineHolder& spline_holder, std::shared_ptr<HeightMap> terrain, SharedBaseMotionData* shared_data)
    :TimeDiscretizationConstraint(T, dt, "baseMotion"),
    terrain_(terrain), // initializing the terrain_
    shared_data_(shared_data)      // initializing
{  
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = spline_holder.base_angular_;

  // double GetHeight(double x, double y);

  double get_z_pos = base_linear_->GetPoint(T).p().z();                     
  double get_pitch_ang = base_angular_->GetPoint(T).p().y();

  double x_current = base_linear_->GetPoint(T).p().x();      
  double y_current = base_linear_->GetPoint(T).p().y();

  double z_terrain = terrain_->GetHeight(x_current, y_current);

  shared_data_->get_z_pos;
  shared_data_->z_terrain;
  shared_data_->get_pitch_ang;

}


} /* namespace towr */
