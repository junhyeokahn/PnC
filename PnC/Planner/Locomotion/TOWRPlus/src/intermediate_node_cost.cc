/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr_plus/costs/intermediate_node_cost.h>

namespace towr_plus {

IntermediateNodeCost::IntermediateNodeCost(const std::string &nodes_id,
                                           Dx deriv, int dim, double weight,
                                           double des_val)
    : CostTerm(nodes_id + "-intermediate-dx_" + std::to_string(deriv) +
               "-dim_" + std::to_string(dim)) {
  node_id_ = nodes_id;
  deriv_ = deriv;
  dim_ = dim;
  weight_ = weight;
  des_val_ = des_val;
}

void IntermediateNodeCost::InitVariableDependedQuantities(
    const VariablesPtr &x) {
  nodes_ = x->GetComponent<NodesVariables>(node_id_);
}

double IntermediateNodeCost::GetCost() const {
  double cost(0.);
  // Iterate except for the final node
  for (int i = 0; i < nodes_->GetNodes().size() - 1; ++i) {
    auto n = nodes_->GetNodes().at(i);
    double val = n.at(deriv_)(dim_);
    cost += weight_ * std::pow(des_val_ - val, 2);
  }
  return cost;
}

void IntermediateNodeCost::FillJacobianBlock(std::string var_set,
                                             Jacobian &jac) const {
  if (var_set == node_id_) {
    for (int i = 0; i < nodes_->GetRows(); ++i) {
      for (auto nvi : nodes_->GetNodeValuesInfo(i)) {
        if ((nvi.id_ < (nodes_->GetNodes().size() - 1)) &&
            nvi.deriv_ == deriv_ && nvi.dim_ == dim_) {
          double val = nodes_->GetNodes().at(nvi.id_).at(deriv_)(dim_);
          jac.coeffRef(0, i) += weight_ * 2.0 * (val - des_val_);
        }
      }
    }
  }
}

} /* namespace towr_plus */
