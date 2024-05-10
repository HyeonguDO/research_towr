#include <towr/constraints/tension_constraint.h>                // NEW NEW NEW

#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>
#include <towr/variables/spline_holder.h>

namespace towr {


TensionConstraint::TensionConstraint (double T, double dt,
                                            const SplineHolder& spline_holder,
                                            std::shared_ptr<HeightMap> terrain)
    :TimeDiscretizationConstraint(T, dt, "ee_tension"),             // baseMotion 부분 수정하기 
     terrain_(terrain) // initializing the terrain_
{
  ee_tension_test_lin_  = spline_holder.ee_tension_test_lin_;
  ee_tension_test_ang_ = spline_holder.ee_tension_test_ang_;

  double dev_rad = 0.1;
  double low_rad = 0.1;

  node_bounds_.resize(k6D);
  node_bounds_.at(AX) = Bounds(-low_rad, dev_rad);                             // ;    ifopt::NoBound
  node_bounds_.at(AY) = Bounds(-low_rad, dev_rad);
  node_bounds_.at(AZ) = Bounds(-low_rad, dev_rad);                             //Bounds(-dev_rad, dev_rad);

  // // Original
//   double z_init = base_linear_->GetPoint(0.0).p().z();
  node_bounds_.at(LX) = ifopt::NoBound;
  node_bounds_.at(LY) = ifopt::NoBound;//Bounds(-0.05, 0.05);
  node_bounds_.at(LZ) = ifopt::NoBound; // allow to move dev_z cm up and down

//   double clearance = 0.58; // distance in meters to maintain between robot and terrain
//   double x_init = base_linear_->GetPoint(0.0).p().x();
//   double y_init = base_linear_->GetPoint(0.0).p().y();
//   double z_terrain = terrain_->GetHeight(x_init, y_init) + clearance;
//   node_bounds_.at(LX) = ifopt::NoBound;
//   node_bounds_.at(LY) = ifopt::NoBound;
//   node_bounds_.at(LZ) = Bounds(z_terrain-0.4, z_terrain+0.2); // account for terrain height


  int n_constraints_per_node = node_bounds_.size();
  SetRows(GetNumberOfNodes()*n_constraints_per_node);
}

void
TensionConstraint::UpdateConstraintAtInstance (double t, int k,
                                                  VectorXd& g) const
{
  g.middleRows(GetRow(k, LX), k3D) = ee_tension_test_lin_->GetPoint(t).p();
  g.middleRows(GetRow(k, AX), k3D) = ee_tension_test_ang_->GetPoint(t).p();
}

void
TensionConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  for (int dim=0; dim<node_bounds_.size(); ++dim)
    bounds.at(GetRow(k,dim)) = node_bounds_.at(dim);
}

void
TensionConstraint::UpdateJacobianAtInstance (double t, int k,
                                                std::string var_set,
                                                Jacobian& jac) const
{
  if (var_set == id::ee_tension_ang_nodes)
    jac.middleRows(GetRow(k,AX), k3D) = ee_tension_test_ang_->GetJacobianWrtNodes(t, kPos);

  if (var_set == id::ee_tension_lin_nodes)
    jac.middleRows(GetRow(k,LX), k3D) = ee_tension_test_lin_->GetJacobianWrtNodes(t, kPos);
}

int
TensionConstraint::GetRow (int node, int dim) const
{
  return node*node_bounds_.size() + dim;
}

} /* namespace towr */