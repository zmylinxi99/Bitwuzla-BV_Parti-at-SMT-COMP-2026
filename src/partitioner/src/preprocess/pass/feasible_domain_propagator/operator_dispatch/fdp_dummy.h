#pragma once

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpDummyOperator : public FdpOperator {
  public:
    FdpDummyOperator(OperatorStatistics& stats,
                     DomainVector& children,
                     FeasibleDomain* self);

    Result apply() override;

  private:
    OperatorStatistics& d_stats;
};

}  // namespace bzla::preprocess::pass::fdp
