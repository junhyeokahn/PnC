/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <ifopt/cost_term.h>

#include <towr_plus/variables/nodes_variables.h>

namespace towr_plus {

/**
 * @brief  Assigns a cost to a final node value.
 *
 * @ingroup Costs
 */
class FinalNodeCost : public ifopt::CostTerm {
public:
  /**
   * @brief Constructs a cost term for the optimization problem.
   * @param nodes_id  The name of the node variables.
   * @param deriv     The node derivative (pos, vel) which should be penalized.
   * @param dim       The node dimension which should be penalized.
   */
  FinalNodeCost(const std::string &nodes_id, Dx deriv, int dim, double weight,
                double des_val = 0.);
  virtual ~FinalNodeCost() = default;

  void InitVariableDependedQuantities(const VariablesPtr &x) override;

  double GetCost() const override;

private:
  std::shared_ptr<NodesVariables> nodes_;

  std::string node_id_;
  Dx deriv_;
  int dim_;
  double weight_;
  double des_val_;

  void FillJacobianBlock(std::string var_set, Jacobian &) const override;
};

} /* namespace towr_plus */
