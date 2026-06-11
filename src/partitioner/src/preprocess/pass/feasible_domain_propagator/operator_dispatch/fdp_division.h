#pragma once

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpDivOperator : public FdpOperator {
  public:
    FdpDivOperator(OperatorStatistics& stats,
                   DomainVector& children,
                   FeasibleDomain* self);
    Result apply() override;

  private:
    OperatorStatistics& d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

class FdpRemOperator : public FdpOperator {
  public:
    FdpRemOperator(OperatorStatistics& stats,
                   DomainVector& children,
                   FeasibleDomain* self);
    Result apply() override;

  private:
    OperatorStatistics& d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

}  // namespace bzla::preprocess::pass::fdp
