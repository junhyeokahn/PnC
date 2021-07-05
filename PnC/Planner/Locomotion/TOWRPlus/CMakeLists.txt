add_library(towr_plus SHARED
  # sample formulation usage
  src/nlp_formulation.cc
  src/parameters.cc
  # variables
  src/nodes_variables.cc
  src/nodes_variables_all.cc
  src/nodes_variables_phase_based.cc
  src/phase_durations.cc
  # models
  src/robot_model.cc
  src/dynamic_model.cc
  src/composite_rigid_body_dynamics.cc
  src/composite_rigid_body_inertia.cc
  src/crbi_helper.cc
  src/atlas_crbi_helper.c
  src/atlas_composite_rigid_body_inertia.cc
  src/valkyrie_crbi_helper.c
  src/valkyrie_composite_rigid_body_inertia.cc
  src/nao_crbi_helper.c
  src/nao_composite_rigid_body_inertia.cc
  # constraints
  src/time_discretization_constraint.cc
  src/base_motion_constraint.cc
  src/terrain_constraint.cc
  src/swing_constraint.cc
  src/force_constraint.cc
  src/total_duration_constraint.cc
  src/dynamic_constraint.cc
  src/range_of_motion_circle_constraint.cc
  src/range_of_motion_box_constraint.cc
  src/spline_acc_constraint.cc
  src/linear_constraint.cc
  # costs
  src/node_cost.cc
  src/node_difference_cost.cc
  src/final_node_cost.cc
  src/intermediate_node_cost.cc
  src/soft_constraint.cc
  # initialization
  src/footstep.cpp
  src/dcm_planner.cpp
  # terrain
  src/height_map_examples.cc
  src/height_map.cc
  # helpers
  src/state.cc
  src/polynomial.cc
  src/spline.cc
  src/phase_spline.cc
  src/node_spline.cc
  src/nodes_observer.cc
  src/spline_holder.cc
  src/euler_converter.cc
  src/phase_durations_observer.cc
  # high level interface
  src/locomotion_task.cc
  src/locomotion_solution.cc
)

target_link_libraries(towr_plus PUBLIC ifopt::ifopt_core pnc-util dubins
    my_goldfarb)

