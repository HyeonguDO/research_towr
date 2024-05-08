
#ifndef TOWR_CONSTRAINTS_BASE_MOTION_GET_H_
#define TOWR_CONSTRAINTS_BASE_MOTION_GET_H_

#include <towr/variables/spline_holder.h>
#include <towr/variables/spline.h>

#include "time_discretization_constraint.h"

#include <towr/terrain/height_map.h>
#include <towr/terrain/examples/height_map_examples.h>


namespace towr {

struct SharedBaseMotionData {
  double get_z_pos;
  double z_terrain;
  double get_pitch_ang;
};

class BaseMotionGet : public TimeDiscretizationConstraint {
public:
  /**
   * @brief Links the base variables and sets hardcoded bounds on the state.
   * @param T  The total time of the optimization horizon.
   * @param dt The discretization interval of the constraints.
   * @param spline_holder  Holds pointers to the base variables.
   */
  BaseMotionGet (double T, double dt, const SplineHolder& spline_holder, std::shared_ptr<HeightMap> terrain, SharedBaseMotionData* shared_data);
  virtual ~BaseMotionGet () = default;
    
//   void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
//   void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
//   void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

private:
  NodeSpline::Ptr base_linear_;
  NodeSpline::Ptr base_angular_;

  std::shared_ptr<HeightMap> terrain_;                                  // NEW    NEW   

  SharedBaseMotionData* shared_data_;                                   // Pointer to shared data structure  NEW
};



} /* namespace towr */

#endif