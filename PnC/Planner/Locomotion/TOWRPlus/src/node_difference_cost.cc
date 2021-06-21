/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#include <towr_plus/costs/node_difference_cost.h>

#include <cmath>

namespace towr_plus {

NodeDifferenceCost::NodeDifferenceCost(const std::string &nodes_id, Dx deriv,
                                       int dim, double weight)
    : CostTerm(nodes_id + "difference-dx_" + std::to_string(deriv) + "-dim_" +
               std::to_string(dim)) {
  node_id_ = nodes_id;
  deriv_ = deriv;
  dim_ = dim;
  weight_ = weight;
}

void NodeDifferenceCost::InitVariableDependedQuantities(const VariablesPtr &x) {
  nodes_ = x->GetComponent<NodesVariables>(node_id_);
}

double NodeDifferenceCost::GetCost() const {
  double cost(0.);
  int n_nodes = nodes_->GetNodes().size();
  for (int i = 0; i < n_nodes - 1; ++i) {
    auto n0 = nodes_->GetNodes().at(i);
    auto n1 = nodes_->GetNodes().at(i + 1);
    double val_diff = n1.at(deriv_)(dim_) - n0.at(deriv_)(dim_);
    cost += weight_ * std::pow(val_diff, 2);
  }
  return cost;
}

void NodeDifferenceCost::FillJacobianBlock(std::string var_set,
                                           Jacobian &jac) const {
  int n_nodes = nodes_->GetNodes().size();
  if (var_set == node_id_) {
    for (int i = 0; i < nodes_->GetRows(); ++i) {
      for (auto nvi : nodes_->GetNodeValuesInfo(i)) {
        if (nvi.deriv_ == deriv_ && nvi.dim_ == dim_) {
          if (nvi.id_ == 0) {
            // first node
            auto n0 = nodes_->GetNodes().at(nvi.id_);
            auto n1 = nodes_->GetNodes().at(nvi.id_ + 1);
            jac.coeffRef(0, i) +=
                weight_ * 2.0 * (n0.at(deriv_)(dim_) - n1.at(deriv_)(dim_));

          } else if (nvi.id_ == n_nodes - 1) {
            // final node
            auto nf = nodes_->GetNodes().at(nvi.id_);
            auto nf_prev = nodes_->GetNodes().at(nvi.id_ - 1);
            jac.coeffRef(0, i) +=
                weight_ * 2.0 *
                (nf.at(deriv_)(dim_) - nf_prev.at(deriv_)(dim_));
          } else {
            // intermediate nodes
            auto np = nodes_->GetNodes().at(nvi.id_ - 1);
            auto nc = nodes_->GetNodes().at(nvi.id_);
            auto nn = nodes_->GetNodes().at(nvi.id_ + 1);
            jac.coeffRef(0, i) += weight_ * 2.0 *
                                  (2 * nc.at(deriv_)(dim_) -
                                   np.at(deriv_)(dim_) - nn.at(deriv_)(dim_));
          }
        }
      }
    }
  }
}

} /* namespace towr_plus */
