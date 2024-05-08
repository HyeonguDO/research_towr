#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_

#include "phase_durations.h"
#include "node_spline.h"
#include "nodes_variables.h"
#include "nodes_variables_phase_based.h"

namespace towr {

/**
 * @brief Builds splines from node values (pos/vel) and durations.
 *
 * These splines are linked to the optimization variables, so change as the
 * nodes or durations change. This is a convenience class to not have
 * to construct the splines from the variables new every time.
 */
struct SplineHolder {
  /**
   * @brief Fully construct all splines.
   * @param base_lin  The nodes describing the base linear motion.
   * @param base_ang  The nodes describing the base angular motion.
   * @param base_poly_durations The durations of each base polynomial.
   * @param ee_motion The nodes describing the endeffector motions.
   * @param ee_force  The nodes describing the endeffector forces.
   * @param phase_durations  The phase durations of each endeffector.
   * @param ee_durations_change  True if the ee durations are optimized over.
   */
  SplineHolder (NodesVariables::Ptr base_lin,
                NodesVariables::Ptr base_ang,
                const std::vector<double>& base_poly_durations,
                std::vector<NodesVariablesPhaseBased::Ptr> ee_motion,
                std::vector<NodesVariablesPhaseBased::Ptr> ee_force,
                std::vector<PhaseDurations::Ptr> phase_durations,
                // std::vector<NodesVariablesPhaseBased::Ptr> ee_tension,                          // NEW
                NodesVariables::Ptr ee_tension_test_lin,                                           // NEW     TEST
                // NodesVariables::Ptr ee_tension_test_ang,                                           // NEW     TEST
                bool ee_durations_change);

  /**
   * @brief Attention, nothing initialized.
   */
  SplineHolder () = default;

  NodeSpline::Ptr base_linear_;
  NodeSpline::Ptr base_angular_;
  NodeSpline::Ptr ee_tension_test_lin_;
  // NodeSpline::Ptr ee_tension_test_ang_;  

  std::vector<NodeSpline::Ptr> ee_motion_;
  std::vector<NodeSpline::Ptr> ee_force_;
  std::vector<PhaseDurations::Ptr> phase_durations_;
  // std::vector<NodeSpline::Ptr> ee_tension_;                                                     // NEW
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_H_ */