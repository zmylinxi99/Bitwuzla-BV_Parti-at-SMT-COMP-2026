#pragma once

#include <string>

#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

struct OperatorStatistics {
    OperatorStatistics(util::Statistics& stats, const std::string& prefix)
        : d_num_apply(stats.new_stat<uint64_t>(prefix + "num_apply")),
          // FdpLeftShiftOperator
          d_time_fdp_leftshift(stats.new_stat<util::TimerStatistic>(prefix + "FdpLeftShiftOperator::" + "time_apply")),
          d_num_fdp_leftshift(stats.new_stat<uint64_t>(prefix + "FdpLeftShiftOperator::" + "num_apply")),
          // FdpRightShiftOperator
          d_time_fdp_rightshift(stats.new_stat<util::TimerStatistic>(prefix + "FdpRightShiftOperator::" + "time_apply")),
          d_num_fdp_rightshift(stats.new_stat<uint64_t>(prefix + "FdpRightShiftOperator::" + "num_apply")),
          // FdpArithRightShiftOperator
          d_time_fdp_arith_rightshift(stats.new_stat<util::TimerStatistic>(prefix + "FdpArithRightShiftOperator::" + "time_apply")),
          d_num_fdp_arith_rightshift(stats.new_stat<uint64_t>(prefix + "FdpArithRightShiftOperator::" + "num_apply")),
          // FdpAddOperator
          d_time_fdp_add(stats.new_stat<util::TimerStatistic>(prefix + "FdpAddOperator::" + "time_apply")),
          d_num_fdp_add(stats.new_stat<uint64_t>(prefix + "FdpAddOperator::" + "num_apply")),
          // FdpMulOperator
          d_time_fdp_mul(stats.new_stat<util::TimerStatistic>(prefix + "FdpMulOperator::" + "time_apply")),
          d_num_fdp_mul(stats.new_stat<uint64_t>(prefix + "FdpMulOperator::" + "num_apply")),
          // FdpDivOperator
          d_time_fdp_div(stats.new_stat<util::TimerStatistic>(prefix + "FdpDivOperator::" + "time_apply")),
          d_num_fdp_div(stats.new_stat<uint64_t>(prefix + "FdpDivOperator::" + "num_apply")),
          // FdpRemOperator
          d_time_fdp_rem(stats.new_stat<util::TimerStatistic>(prefix + "FdpRemOperator::" + "time_apply")),
          d_num_fdp_rem(stats.new_stat<uint64_t>(prefix + "FdpRemOperator::" + "num_apply")),
          // FdpNotOperator
          d_time_fdp_not(stats.new_stat<util::TimerStatistic>(prefix + "FdpNotOperator::" + "time_apply")),
          d_num_fdp_not(stats.new_stat<uint64_t>(prefix + "FdpNotOperator::" + "num_apply")),
          // FdpXorOperator
          d_time_fdp_xor(stats.new_stat<util::TimerStatistic>(prefix + "FdpXorOperator::" + "time_apply")),
          d_num_fdp_xor(stats.new_stat<uint64_t>(prefix + "FdpXorOperator::" + "num_apply")),
          // FdpAndOperator
          d_time_fdp_and(stats.new_stat<util::TimerStatistic>(prefix + "FdpAndOperator::" + "time_apply")),
          d_num_fdp_and(stats.new_stat<uint64_t>(prefix + "FdpAndOperator::" + "num_apply")),
          // FdpLessThanOperator
          d_time_fdp_less_than(stats.new_stat<util::TimerStatistic>(prefix + "FdpLessThanOperator::" + "time_apply")),
          d_num_fdp_less_than(stats.new_stat<uint64_t>(prefix + "FdpLessThanOperator::" + "num_apply")),
          // FdpSignedLessThanOperator
          d_time_fdp_signed_less_than(stats.new_stat<util::TimerStatistic>(prefix + "FdpSignedLessThanOperator::" + "time_apply")),
          d_num_fdp_signed_less_than(stats.new_stat<uint64_t>(prefix + "FdpSignedLessThanOperator::" + "num_apply")),
          // FdpEqOperator
          d_time_fdp_eq(stats.new_stat<util::TimerStatistic>(prefix + "FdpEqOperator::" + "time_apply")),
          d_num_fdp_eq(stats.new_stat<uint64_t>(prefix + "FdpEqOperator::" + "num_apply")),
          // FdpIteOperator
          d_time_fdp_ite(stats.new_stat<util::TimerStatistic>(prefix + "FdpIteOperator::" + "time_apply")),
          d_num_fdp_ite(stats.new_stat<uint64_t>(prefix + "FdpIteOperator::" + "num_apply")),
          // FdpExtractOperator
          d_time_fdp_extract(stats.new_stat<util::TimerStatistic>(prefix + "FdpExtractOperator::" + "time_apply")),
          d_num_fdp_extract(stats.new_stat<uint64_t>(prefix + "FdpExtractOperator::" + "num_apply")),
          // FdpConcatOperator
          d_time_fdp_concat(stats.new_stat<util::TimerStatistic>(prefix + "FdpConcatOperator::" + "time_apply")),
          d_num_fdp_concat(stats.new_stat<uint64_t>(prefix + "FdpConcatOperator::" + "num_apply")),
          // FdpDummyOperator
          d_time_fdp_dummy(stats.new_stat<util::TimerStatistic>(prefix + "FdpDummyOperator::" + "time_apply")),
          d_num_fdp_dummy(stats.new_stat<uint64_t>(prefix + "FdpDummyOperator::" + "num_apply")) {}

    uint64_t& d_num_apply;
    // FdpLeftShiftOperator
    util::TimerStatistic& d_time_fdp_leftshift;
    uint64_t& d_num_fdp_leftshift;
    // FdpRightShiftOperator
    util::TimerStatistic& d_time_fdp_rightshift;
    uint64_t& d_num_fdp_rightshift;
    // FdpArithRightShiftOperator
    util::TimerStatistic& d_time_fdp_arith_rightshift;
    uint64_t& d_num_fdp_arith_rightshift;
    // FdpAddOperator
    util::TimerStatistic& d_time_fdp_add;
    uint64_t& d_num_fdp_add;
    // FdpMulOperator
    util::TimerStatistic& d_time_fdp_mul;
    uint64_t& d_num_fdp_mul;
    // FdpDivOperator
    util::TimerStatistic& d_time_fdp_div;
    uint64_t& d_num_fdp_div;
    // FdpRemOperator
    util::TimerStatistic& d_time_fdp_rem;
    uint64_t& d_num_fdp_rem;
    // FdpNotOperator
    util::TimerStatistic& d_time_fdp_not;
    uint64_t& d_num_fdp_not;
    // FdpXorOperator
    util::TimerStatistic& d_time_fdp_xor;
    uint64_t& d_num_fdp_xor;
    // FdpAndOperator
    util::TimerStatistic& d_time_fdp_and;
    uint64_t& d_num_fdp_and;
    // FdpLessThanOperator
    util::TimerStatistic& d_time_fdp_less_than;
    uint64_t& d_num_fdp_less_than;
    // FdpSignedLessThanOperator
    util::TimerStatistic& d_time_fdp_signed_less_than;
    uint64_t& d_num_fdp_signed_less_than;
    // FdpEqOperator
    util::TimerStatistic& d_time_fdp_eq;
    uint64_t& d_num_fdp_eq;
    // FdpIteOperator
    util::TimerStatistic& d_time_fdp_ite;
    uint64_t& d_num_fdp_ite;
    // FdpExtractOperator
    util::TimerStatistic& d_time_fdp_extract;
    uint64_t& d_num_fdp_extract;
    // FdpConcatOperator
    util::TimerStatistic& d_time_fdp_concat;
    uint64_t& d_num_fdp_concat;
    // FdpDummyOperator
    util::TimerStatistic& d_time_fdp_dummy;
    uint64_t& d_num_fdp_dummy;
};

}  // namespace bzla::preprocess::pass::fdp
