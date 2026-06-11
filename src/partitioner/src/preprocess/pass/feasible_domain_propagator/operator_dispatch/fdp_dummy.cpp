#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_dummy.h"

#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

FdpDummyOperator::FdpDummyOperator(OperatorStatistics& stats,
                                   DomainVector& children,
                                   FeasibleDomain* self)
    : FdpOperator(node::Kind::NULL_NODE, "fdp_dummy", children, self),
      d_stats(stats) {
}

Result
FdpDummyOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_dummy);
    ++d_stats.d_num_fdp_dummy;

    return Result::UNCHANGED;
}

}  // namespace bzla::preprocess::pass::fdp
