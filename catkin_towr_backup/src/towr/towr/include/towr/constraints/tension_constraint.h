#ifndef TOWR_CONSTRAINTS_TENSION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TENSION_CONSTRAINT_H_

#include <towr/variables/spline_holder.h>
#include <towr/variables/spline.h>

#include "time_discretization_constraint.h"

#include <towr/terrain/height_map.h>
#include <towr/terrain/examples/height_map_examples.h>


namespace towr {

/**
 * @brief Keeps the 6D base motion in a specified range.
 *
 * In general this constraint should be avoided, since a similar affect can be
 * achieved with RangeOfMotionConstraint.
 *
 * @ingroup Constraints
 */
class TensionConstraint : public TimeDiscretizationConstraint {
public:
  /**
   * @brief Links the base variables and sets hardcoded bounds on the state.
   * @param T  The total time of the optimization horizon.
   * @param dt The discretization interval of the constraints.
   * @param spline_holder  Holds pointers to the base variables.
   */
  TensionConstraint (double T, double dt, const SplineHolder& spline_holder, std::shared_ptr<HeightMap> terrain);  // Using shared_ptr for HeightMap
  virtual ~TensionConstraint () = default;

  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

private:
  NodeSpline::Ptr ee_tension_test_lin_;
  NodeSpline::Ptr ee_tension_test_ang_;

  std::shared_ptr<HeightMap> terrain_;                                  // NEW    NEW   

  VecBound node_bounds_;     ///< same bounds for each discretized node
  int GetRow (int node, int dim) const;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_ */