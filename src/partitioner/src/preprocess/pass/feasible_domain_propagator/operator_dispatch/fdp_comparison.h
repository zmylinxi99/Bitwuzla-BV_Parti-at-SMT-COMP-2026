#pragma once

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

// BV_ULT
class FdpLessThanOperator : public FdpOperator {
  public:
    FdpLessThanOperator(OperatorStatistics &stats,
                        DomainVector &children,
                        FeasibleDomain *self);
    ~FdpLessThanOperator() = default;

    Result apply() override;

  private:
    Result tighten_less_than_by_fixed_bits(FeasibleDomain &c0,
                                           FeasibleDomain &c1);

    Result tighten_less_than(FeasibleDomain &c0,
                             FeasibleDomain &c1);

    Result tighten_less_than_equals_by_fixed_bits(FeasibleDomain &c0,
                                                  FeasibleDomain &c1);
    Result tighten_less_than_equals(FeasibleDomain &c0,
                                    FeasibleDomain &c1);

    Result tighten_output_by_inputs(FeasibleDomain &c0,
                                    FeasibleDomain &c1,
                                    FeasibleDomain &output);

  private:
    // d_self and d_children provided by base class.
    OperatorStatistics &d_stats;
};

// BV_SLT
class FdpSignedLessThanOperator : public FdpOperator {
  public:
    FdpSignedLessThanOperator(OperatorStatistics &stats,
                              DomainVector &children,
                              FeasibleDomain *self);
    ~FdpSignedLessThanOperator() = default;

    Result apply() override;

  private:
    inline bool using_signed_interval(const FeasibleDomain &domain);
    inline BitVector signed_interval_min(const FeasibleDomain &domain);
    inline BitVector signed_interval_max(const FeasibleDomain &domain);

    Result tighten_output_by_inputs(FeasibleDomain &c0,
                                    FeasibleDomain &c1,
                                    FeasibleDomain &output);

    Result tighten_signed_less_than_by_fixed_bits(FeasibleDomain &c0,
                                                  FeasibleDomain &c1);
    Result tighten_signed_less_than(FeasibleDomain &c0,
                                    FeasibleDomain &c1);

    Result tighten_signed_less_than_equals_by_fixed_bits(FeasibleDomain &c0,
                                                         FeasibleDomain &c1);
    Result tighten_signed_less_than_equals(FeasibleDomain &c0,
                                           FeasibleDomain &c1);

    // d_self and d_children provided by base class.

    OperatorStatistics &d_stats;
};

// EQ
class FdpEqOperator : public FdpOperator {
  public:
    FdpEqOperator(OperatorStatistics &stats,
                  DomainVector &children,
                  FeasibleDomain *self);

    Result apply() override;

  private:
    OperatorStatistics &d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};
}  // namespace bzla::preprocess::pass::fdp
