#pragma once

#include <vector>

#include "../feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpNotOperator : public FdpOperator {
  public:
    FdpNotOperator(OperatorStatistics& stats,
                   DomainVector& children,
                   FeasibleDomain* self)
        : FdpOperator(node::Kind::NOT, "fdp_not", children, self),
          d_stats(stats) {}

    Result apply() override;
    bool implied_by(std::vector<uint32_t>& child_ids);

  private:
    OperatorStatistics& d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

class FdpXorOperator : public FdpOperator {
  public:
    FdpXorOperator(OperatorStatistics& stats,
                   DomainVector& children,
                   FeasibleDomain* self)
        : FdpOperator(node::Kind::XOR, "fdp_xor", children, self),
          d_stats(stats) {}

    Result apply() override;
    bool implied_by(std::vector<uint32_t>& child_ids);

  private:
    OperatorStatistics& d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

class FdpAndOperator : public FdpOperator {
  public:
    FdpAndOperator(OperatorStatistics& stats,
                   DomainVector& children,
                   FeasibleDomain* self)
        : FdpOperator(node::Kind::AND, "fdp_and", children, self),
          d_stats(stats) {}

    Result apply() override;
    bool implied_by(std::vector<uint32_t>& child_ids);

  private:
    OperatorStatistics& d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

}  // namespace bzla::preprocess::pass::fdp
