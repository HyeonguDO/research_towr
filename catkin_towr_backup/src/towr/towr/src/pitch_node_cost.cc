#include <towr/costs/pitch_node_cost.h>
#include <towr/constraints/base_motion_get.h>
// #include <towr/variables/spline_holder.h>

#include <cmath>

namespace towr {

PitchNodeCost::PitchNodeCost (const std::string& nodes_id, Dx deriv, int dim, double weight)
    : CostTerm(nodes_id +"-dx_"+std::to_string(deriv) +"-dim_"+std::to_string(dim))
    //   ,shared_data_(shared_data)
{
  node_id_ = nodes_id;
  deriv_ = deriv;
  dim_   = dim;
  weight_ = weight;  
}

void
PitchNodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = x->GetComponent<NodesVariables>(node_id_);
}

double
PitchNodeCost::GetCost () const
{
  double cost;
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    cost += weight_ * std::pow(val - pitch_ang, 2);
  }

  return cost;
}

void
PitchNodeCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == node_id_) {
    for (int i=0; i<nodes_->GetRows(); ++i)
      for (auto nvi : nodes_->GetNodeValuesInfo(i))
        if (nvi.deriv_==deriv_ && nvi.dim_==dim_) {
          double val = nodes_->GetNodes().at(nvi.id_).at(deriv_)(dim_);
          jac.coeffRef(0, i) += weight_*2.0*(val - pitch_ang);
        }
  }
}

} /* namespace towr */