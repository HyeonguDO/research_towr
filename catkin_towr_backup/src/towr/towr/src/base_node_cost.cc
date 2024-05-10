#include <towr/costs/base_node_cost.h>
#include <towr/constraints/base_motion_get.h>
// #include <towr/variables/spline_holder.h>

#include <cmath>

namespace towr {

BaseNodeCost::BaseNodeCost (const std::string& nodes_id, Dx deriv, int dim, double weight, SharedBaseMotionData* shared_data)
    : CostTerm(nodes_id +"-dx_"+std::to_string(deriv) +"-dim_"+std::to_string(dim)),
      shared_data_(shared_data)
{
  node_id_ = nodes_id;
  deriv_ = deriv;
  dim_   = dim;
  weight_ = weight; 

  shared_data_ = shared_data; 
}

void
BaseNodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = x->GetComponent<NodesVariables>(node_id_);
}

double
BaseNodeCost::GetCost () const
{
  double cost;
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    cost += weight_ * std::pow(val - shared_data_->z_terrain, 2);
  }

  return cost;
}

void
BaseNodeCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == node_id_) {
    for (int i=0; i<nodes_->GetRows(); ++i)
      for (auto nvi : nodes_->GetNodeValuesInfo(i))
        if (nvi.deriv_==deriv_ && nvi.dim_==dim_) {
          double val = nodes_->GetNodes().at(nvi.id_).at(deriv_)(dim_);
          jac.coeffRef(0, i) += weight_*2.0*(val - shared_data_->z_terrain);
        }
  }
}

} /* namespace towr */