#pragma once

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpExtractOperator : public FdpOperator {
  private:
    OperatorStatistics &d_stats;
    uint64_t d_low;
    uint64_t d_high;

  public:
    FdpExtractOperator(OperatorStatistics &stats,
                       DomainVector &children,
                       FeasibleDomain *self,
                       uint64_t low,
                       uint64_t high);
    Result apply() override;
};

class FdpConcatOperator : public FdpOperator {
  public:
    FdpConcatOperator(OperatorStatistics &stats,
                      DomainVector &children,
                      FeasibleDomain *self);
    Result apply() override;

  private:
    OperatorStatistics &d_stats;
};

}  // namespace bzla::preprocess::pass::fdp
