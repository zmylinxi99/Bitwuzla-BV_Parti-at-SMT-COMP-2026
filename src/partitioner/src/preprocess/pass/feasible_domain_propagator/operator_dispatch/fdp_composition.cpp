#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_composition.h"

#include <cassert>

#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

FdpExtractOperator::FdpExtractOperator(OperatorStatistics &stats,
                                       DomainVector &children,
                                       FeasibleDomain *self,
                                       uint64_t low,
                                       uint64_t high)
    : FdpOperator(node::Kind::BV_EXTRACT, "fdp_bv_extract", children, self),
      d_stats(stats),
      d_low(low),
      d_high(high) {}

Result
FdpExtractOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_extract);
    ++d_stats.d_num_fdp_extract;

    Result result = Result::UNCHANGED;

    assert(d_children.size() == 1);

    uint64_t low = d_low;
    uint64_t high = d_high;

    FeasibleDomain &input = *d_children[0];
    FeasibleDomain &output = *d_self;

    const uint32_t extract_width = high - low + 1;

    assert(output.width() == extract_width);
    assert(input.width() > high);

    for (uint32_t output_bit = 0; output_bit < extract_width; ++output_bit) {
        uint32_t input_bit = low + output_bit;

        if (input.is_fixed(input_bit) && output.is_fixed(output_bit) &&
            input.get_value(input_bit) != output.get_value(output_bit)) {
            // fixed values are different => conflict
            return Result::CONFLICT;
        }
        else if (!input.is_fixed(input_bit) && output.is_fixed(output_bit)) {
            // output fixed => set input
            result |= input.set_fixed(input_bit, output.get_value(output_bit));
        }
        else if (input.is_fixed(input_bit) && !output.is_fixed(output_bit)) {
            // input fixed => set output
            result |= output.set_fixed(output_bit, input.get_value(input_bit));
        }

        if (result == Result::CONFLICT)
            return Result::CONFLICT;
    }

    // TODO: update intervals

    return result;
}

FdpConcatOperator::FdpConcatOperator(OperatorStatistics &stats,
                                     DomainVector &children,
                                     FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_CONCAT, "fdp_bv_concat", children, self),
      d_stats(stats) {}

Result
FdpConcatOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_concat);
    ++d_stats.d_num_fdp_concat;

    Result result = Result::UNCHANGED;

    const uint32_t children_size = d_children.size();
    FeasibleDomain &output = *d_self;
    uint32_t offset = 0;

    for (int32_t i = children_size - 1; i >= 0; --i) {
        FeasibleDomain &child = *d_children[i];
        const uint32_t child_width = child.width();

        for (uint32_t j = 0; j < child_width; ++j, ++offset) {
            if (child.is_fixed(j) && output.is_fixed(offset) &&
                child.get_value(j) != output.get_value(offset)) {
                // fixed values are different => conflict
                return Result::CONFLICT;
            }
            else if (!child.is_fixed(j) && output.is_fixed(offset)) {
                // output fixed => set child
                result |= child.set_fixed(j, output.get_value(offset));
            }
            else if (child.is_fixed(j) && !output.is_fixed(offset)) {
                // child fixed => set output
                result |= output.set_fixed(offset, child.get_value(j));
            }

            if (result == Result::CONFLICT)
                return Result::CONFLICT;
        }
    }

    // TODO: update intervals

    assert(offset == output.width());

    return result;
}

}  // namespace bzla::preprocess::pass::fdp
