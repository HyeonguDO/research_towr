#include <towr/variables/spline_holder.h>
#include <towr/variables/phase_spline.h>

#include <iostream>

namespace towr{

SplineHolder::SplineHolder (NodesVariables::Ptr base_lin_nodes,
                            NodesVariables::Ptr base_ang_nodes,
                            const std::vector<double>& base_poly_durations,
                            std::vector<NodesVariablesPhaseBased::Ptr> ee_motion_nodes,
                            std::vector<NodesVariablesPhaseBased::Ptr> ee_force_nodes,
                            std::vector<PhaseDurations::Ptr> phase_durations,

                            // std::vector<NodesVariablesPhaseBased::Ptr> ee_tension_nodes,    // NEW
                            NodesVariables::Ptr ee_tension_test_lin,                           // NEW     TEST
                            // NodesVariables::Ptr ee_tension_test_ang,                           // NEW     TEST
                            bool durations_change)
{
  
  base_linear_  = std::make_shared<NodeSpline>(base_lin_nodes.get(), base_poly_durations);
  base_angular_ = std::make_shared<NodeSpline>(base_ang_nodes.get(), base_poly_durations);
  ee_tension_test_lin_ = std::make_shared<NodeSpline>(ee_tension_test_lin.get(), base_poly_durations);              // TEST  TEST  linear
  // ee_tension_test_ang_ = std::make_shared<NodeSpline>(ee_tension_test_ang.get(), base_poly_durations);              // TEST  TEST  angular

  phase_durations_ = phase_durations;

  for (uint ee=0; ee<ee_motion_nodes.size(); ++ee) {
    if (durations_change) {

      ee_motion_.push_back(std::make_shared<PhaseSpline>(ee_motion_nodes.at(ee), phase_durations.at(ee).get()));
      ee_force_.push_back(std::make_shared<PhaseSpline>(ee_force_nodes.at(ee), phase_durations.at(ee).get()));
      
      // ee_tension_.push_back(std::make_shared<PhaseSpline>(ee_tension_nodes.at(ee), phase_durations.at(ee).get()));            // NEW
      } 
    else {
      // spline without changing the polynomial durations
      auto ee_motion_poly_durations = ee_motion_nodes.at(ee)->ConvertPhaseToPolyDurations(phase_durations.at(ee)->GetPhaseDurations());
      auto ee_force_poly_durations = ee_force_nodes.at(ee)->ConvertPhaseToPolyDurations(phase_durations.at(ee)->GetPhaseDurations());

      // auto ee_tension_poly_durations = ee_tension_nodes.at(ee)->ConvertPhaseToPolyDurations(phase_durations.at(ee)->GetPhaseDurations());       // NEW


      ee_motion_.push_back(std::make_shared<NodeSpline>(ee_motion_nodes.at(ee).get(), ee_motion_poly_durations));
      ee_force_.push_back (std::make_shared<NodeSpline>(ee_force_nodes.at(ee).get(), ee_force_poly_durations));

      // ee_tension_.push_back(std::make_shared<NodeSpline>(ee_tension_nodes.at(ee).get(), ee_tension_poly_durations));            // NEW
      }
    }
}


} /* namespace towr */