#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_ite.h"

#include <cassert>
#include <vector>

#include "fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

FdpIteOperator::FdpIteOperator(OperatorStatistics &stats,
                               DomainVector &children,
                               FeasibleDomain *self)
    : FdpOperator(node::Kind::ITE, "fdp_ite", children, self),
      d_stats(stats) {}

Result
FdpIteOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_ite);
    ++d_stats.d_num_fdp_ite;

    Result result = Result::UNCHANGED;

    FeasibleDomain &guard = *d_children[0];
    FeasibleDomain &c1 = *d_children[1];
    FeasibleDomain &c2 = *d_children[2];
    FeasibleDomain &output = *d_self;

    uint32_t width = output.width();

    // Assertions
    assert(d_children.size() == 3);
    assert(guard.width() == 1);
    assert(c1.width() == c2.width());
    assert(output.width() == c1.width());

    // Input => Output
    if (guard.is_fixed_one(0)) {
        FeasibleDomain boolean(1);
        boolean.set_fixed(0, true);
        std::vector<FeasibleDomain *> to_equal = {&c1, &output};

        result |= FdpEqOperator(d_stats, to_equal, &boolean).apply();

        return result;
    }
    else if (guard.is_fixed_zero(0)) {
        FeasibleDomain boolean(1);
        boolean.set_fixed(0, true);
        std::vector<FeasibleDomain *> to_equal = {&c2, &output};

        result |= FdpEqOperator(d_stats, to_equal, &boolean).apply();
        return result;
    }

    // guard is not fixed
    for (uint32_t i = 0; i < width; ++i) {
        if (c1.is_fixed(i) && c2.is_fixed(i) &&
            c1.get_value(i) == c2.get_value(i)) {
            result |= output.set_fixed(i, c1.get_value(i));
        }
        if (result == Result::CONFLICT)
            return Result::CONFLICT;
    }

    if (output.judge_valid() == Result::CONFLICT)
        return Result::CONFLICT;

    {  // judge c1 and output
        FeasibleDomain boolean(1);
        std::vector<FeasibleDomain *> to_equal = {&c1, &output};
        [[maybe_unused]] Result res = FdpEqOperator(d_stats, to_equal, &boolean).apply();
        assert(res != Result::CONFLICT);
        if (boolean.is_fixed_zero(0)) {
            // c1 and output are not equal => guard must be 0
            result |= guard.set_fixed(0, false);
        }
    }

    {  // judge c2 and output
        FeasibleDomain boolean(1);
        std::vector<FeasibleDomain *> to_equal = {&c2, &output};
        [[maybe_unused]] Result res = FdpEqOperator(d_stats, to_equal, &boolean).apply();
        assert(res != Result::CONFLICT);
        if (boolean.is_fixed_zero(0)) {
            // c2 and output are not equal => guard must be 1
            result |= guard.set_fixed(0, true);
        }
    }

    return result;
}
bool FdpIteOperator::implied_by(std::vector<uint32_t>& child_ids) {
    if (d_children.at(0)->is_totally_fixed()) {
        unsigned branch = d_children.at(0)->get_value(0) ? 1u : 2u;
        unsigned other_branch = 3u - branch;
        assert(d_self->get_value(0) == d_children.at(branch)->get_value(0));
        if (d_children.at(other_branch)->is_totally_fixed()
          && d_self->get_value(0) != d_children.at(other_branch)->get_value(0)) {
            // X 1 X -X -> {c, t} | <- X ? ? -X
            child_ids = {0, branch};
            return true;
        }
        else {
            // X 1 X  X -> {t} | <- X 1 ? X
            // X 1 X  ? -> {t} | <- X 1 ? ?
            child_ids = {branch};
            return true;
        }
    }
    else {
        if (d_children.at(1)->is_totally_fixed() && d_children.at(2)->is_totally_fixed()
          && d_children.at(1)->get_value(0) == d_children.at(2)->get_value(0)) {
            // X ? X X -> false
            return false;
        }
        else {
            // X ? -X -X -> impossible conf

            // X ? ? -X -> impossible up
            // X ? -X ? -> impossible up
            // X ? -X X -> impossible up
            // X ? X -X -> impossible up

            // X ? X ? -> {}
            // X ? ? X -> {}
            // X ? ? ? -> {}
            child_ids = {};
            return true;
        }
    }
}
}  // namespace bzla::preprocess::pass::fdp
