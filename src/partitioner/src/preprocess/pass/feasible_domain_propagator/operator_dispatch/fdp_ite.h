#pragma once

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpIteOperator : public FdpOperator {
  public:
    FdpIteOperator(OperatorStatistics &stats,
                   DomainVector &children,
                   FeasibleDomain *self);
    Result apply() override;
    bool implied_by(std::vector<uint32_t>& child_ids);

  private:
    OperatorStatistics &d_stats;
};

}  // namespace bzla::preprocess::pass::fdp
