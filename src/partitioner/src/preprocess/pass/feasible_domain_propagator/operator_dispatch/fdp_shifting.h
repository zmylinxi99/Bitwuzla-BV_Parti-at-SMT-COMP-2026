#pragma once

#include <memory>

#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics;

class FdpLeftShiftOperator : public FdpOperator {
  public:
    FdpLeftShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self);
    Result apply() override;
    Result fixed_bits_both_way();

  private:
    OperatorStatistics &d_stats;

    Result interval_both_way();

    void get_possible_shifts(const FeasibleDomain &shift,
                             std::unique_ptr<bool[]> &possible_shifts,
                             uint32_t max_possible_shift_num);

    Result fix_bit_in_operand();
    Result fix_bit_in_shift();
    Result fix_bit_in_output();
};

class FdpRightShiftOperator : public FdpOperator {
  public:
    FdpRightShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self);
    Result fixed_bits_both_way();
    Result apply() override;

  private:
    OperatorStatistics &d_stats;

    Result interval_both_way();
};

class FdpArithRightShiftOperator : public FdpOperator {
  public:
    FdpArithRightShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self);
    Result apply() override;

  private:
    OperatorStatistics &d_stats;

    Result fixed_bits_both_way();
    Result interval_both_way();
};

}  // namespace bzla::preprocess::pass::fdp
