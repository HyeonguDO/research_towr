#ifndef TOWR_BASE_COSTS_NODE_COST_H_
#define TOWR_BASE_COSTS_NODE_COST_H_

#include <memory>
#include <string>

#include <ifopt/cost_term.h>

#include <towr/variables/nodes_variables.h>
#include <towr/variables/spline_holder.h>

#include <towr/constraints/base_motion_get.h>


namespace towr {

/**
 * @brief  Assigns a cost to node values.
 *
 * @ingroup Costs
 */
class BaseNodeCost : public ifopt::CostTerm {
public:
  /**
   * @brief Constructs a cost term for the optimization problem.
   * @param nodes_id  The name of the node variables.
   * @param deriv     The node derivative (pos, vel) which should be penalized.
   * @param dim       The node dimension which should be penalized.
   */
  BaseNodeCost (const std::string& nodes_id, Dx deriv, int dim, double weight, SharedBaseMotionData* shared_data);
  virtual ~BaseNodeCost () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  double GetCost () const override;

private:
  std::shared_ptr<NodesVariables> nodes_;

  std::string node_id_;
  Dx deriv_;
  int dim_;

  double weight_;     // height weight

//   NodeSpline::Ptr base_linear_;
//   std::shared_ptr<HeightMap> terrain_;
//   const TimeDiscretizationConstraint& time_constraint_; // Store the time constraint

  SharedBaseMotionData* shared_data_; // Pointer to shared data

  void FillJacobianBlock(std::string var_set, Jacobian&) const override;
};

} /* namespace towr */

#endif /* TOWR_COSTS_NODE_COST_H_ */
