#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_arithmetic.h"

#include <algorithm>

#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

FdpAddOperator::FdpAddOperator(OperatorStatistics& stats,
                               DomainVector& children,
                               FeasibleDomain* self)
    : FdpOperator(node::Kind::BV_ADD, "fdp_bv_add", children, self),
      d_stats(stats) {}

// Result
// FdpAddOperator::fixed_bits_both_way() {
//     Result result = Result::UNCHANGED;
//     FeasibleDomain& c0 = *d_children[0];
//     FeasibleDomain& c1 = *d_children[1];
//     FeasibleDomain& output = *d_self;
//     uint32_t width = c0.width();

//     std::unique_ptr<bool[]> fixed = std::make_unique<bool[]>(width + 1);
//     std::fill_n(fixed.get(), width + 1, false);
//     std::unique_ptr<bool[]> carry = std::make_unique<bool[]>(width + 1);
//     std::fill_n(carry.get(), width + 1, false);

//     const int32_t width_signed = static_cast<int32_t>(width);
//     for (int32_t i = 0; i < width_signed; ++i) {
//         const uint32_t idx = static_cast<uint32_t>(i);
//         int cur_lb = (c0.is_fixed_one(idx) ? 1 : 0) +
//                      (c1.is_fixed_one(idx) ? 1 : 0) +
//                      (fixed[i] ? carry[i] : 0);
//         int cur_ub = (c0.is_fixed_zero(idx) ? 0 : 1) +
//                      (c1.is_fixed_zero(idx) ? 0 : 1) +
//                      (fixed[i] ? carry[i] : 1);

//         if (cur_lb == cur_ub) {
//             // everything is fixed
//             bool out_bit = (cur_lb % 2 == 1);
//             if (!output.is_fixed(i)) {
//                 result |= output.set_fixed(i, out_bit);
//                 continue;
//             }
//             else if (output.get_value(i) != out_bit) {
//                 return Result::CONFLICT;
//             }
//             // carry bit is fixed, donot need to back [i - 1]
//         }

//         const int init_lb = cur_lb;
//         const int init_ub = cur_ub;

//         // use carry bit on [i + 1] tighten current lb/ub
//         if (fixed[i + 1] && carry[i + 1])
//             cur_lb = std::max(cur_lb, 2);
//         if (fixed[i + 1] && !carry[i + 1])
//             cur_ub = std::min(cur_ub, 1);

//         // use fixed output bit to tighten current lb/ub
//         if (output.is_fixed_one(idx)) {
//             if (cur_lb % 2 == 0) cur_lb += 1;
//             if (cur_ub % 2 == 0) cur_ub -= 1;
//         }
//         if (output.is_fixed_zero(idx)) {
//             if (cur_lb % 2 == 1) cur_lb += 1;
//             if (cur_ub % 2 == 1) cur_ub -= 1;
//         }

//         if (cur_lb > cur_ub) {
//             return Result::CONFLICT;
//         }

//         // fix carry bit for [i + 1]
//         if (cur_lb >= 2) {
//             carry[i + 1] = true;
//             fixed[i + 1] = true;
//         }
//         else if (cur_ub <= 1) {
//             carry[i + 1] = false;
//             fixed[i + 1] = true;
//         }

//         if (cur_lb != cur_ub) continue;

//         // cur_lb == cur_ub
//         // is fix output bit for [i]
//         bool out_bit = (cur_lb % 2 == 1);
//         if (!output.is_fixed(idx)) {
//             result |= output.set_fixed(idx, out_bit);
//         }
//         else if (output.get_value(idx) != out_bit) {
//             return Result::CONFLICT;
//         }

//         bool backtrack = false;
//         if (init_lb == cur_lb) {  // set false as we can
//             if (!c0.is_fixed(idx)) {
//                 result |= c0.set_fixed(idx, false);
//             }
//             if (!c1.is_fixed(idx)) {
//                 result |= c1.set_fixed(idx, false);
//             }
//             if (!fixed[i]) {
//                 carry[i] = false;
//                 fixed[i] = true;
//                 backtrack = true;
//             }
//         }

//         else if (init_ub == cur_lb) {  // set true as we can
//             if (!c0.is_fixed(idx)) {
//                 result |= c0.set_fixed(idx, true);
//             }
//             if (!c1.is_fixed(idx)) {
//                 result |= c1.set_fixed(idx, true);
//             }
//             if (!fixed[i]) {
//                 carry[i] = true;
//                 fixed[i] = true;
//                 backtrack = true;
//             }
//         }

//         if (backtrack) {
//             // Step back one column; loop increment will move us to the prior index.
//             i = std::max(-1, i - 2);
//         }
//     }

//     return result;
// }
Result
FdpAddOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;
    FeasibleDomain& c0 = *d_children[0];
    FeasibleDomain& c1 = *d_children[1];
    FeasibleDomain& output = *d_self;
    uint32_t width = c0.width();

    std::unique_ptr<bool[]> fixed = std::make_unique<bool[]>(width + 1);
    std::fill_n(fixed.get(), width + 1, false);
    std::unique_ptr<bool[]> carry = std::make_unique<bool[]>(width + 1);
    std::fill_n(carry.get(), width + 1, false);

    fixed[0] = true;
    carry[0] = false;

    const int32_t width_signed = static_cast<int32_t>(width);
    for (int32_t i = 0; i < width_signed; ++i) {
        const uint32_t idx = static_cast<uint32_t>(i);
        int cur_lb = (c0.is_fixed_one(idx) ? 1 : 0) +
                     (c1.is_fixed_one(idx) ? 1 : 0) +
                     (fixed[i] ? carry[i] : 0);
        int cur_ub = (c0.is_fixed_zero(idx) ? 0 : 1) +
                     (c1.is_fixed_zero(idx) ? 0 : 1) +
                     (fixed[i] ? carry[i] : 1);

        const int init_lb = cur_lb;
        const int init_ub = cur_ub;

        if (fixed[i + 1] && carry[i + 1])
            cur_lb = std::max(cur_lb, 2);
        if (fixed[i + 1] && !carry[i + 1])
            cur_ub = std::min(cur_ub, 1);

        if (output.is_fixed_one(idx)) {
            if (cur_lb % 2 == 0) cur_lb += 1;
            if (cur_ub % 2 == 0) cur_ub -= 1;
        }
        if (output.is_fixed_zero(idx)) {
            if (cur_lb % 2 == 1) cur_lb += 1;
            if (cur_ub % 2 == 1) cur_ub -= 1;
        }

        if (cur_lb > cur_ub) {
            return Result::CONFLICT;
        }

        if (cur_lb >= 2) {
            carry[i + 1] = true;
            fixed[i + 1] = true;
        }
        else if (cur_ub <= 1) {
            carry[i + 1] = false;
            fixed[i + 1] = true;
        }

        if (cur_lb != cur_ub) continue;

        bool out_bit = (cur_lb % 2 == 1);
        if (!output.is_fixed(idx)) {
            result |= output.set_fixed(idx, out_bit);
        }
        else if (output.get_value(idx) != out_bit) {
            return Result::CONFLICT;
        }

        bool backtrack = false;
        if (init_lb == cur_lb) {
            if (!c0.is_fixed(idx)) {
                result |= c0.set_fixed(idx, false);
            }
            if (!c1.is_fixed(idx)) {
                result |= c1.set_fixed(idx, false);
            }
            if (!fixed[i]) {
                carry[i] = false;
                fixed[i] = true;
                backtrack = true;
            }
        }
        else if (init_ub == cur_lb) {
            if (!c0.is_fixed(idx)) {
                result |= c0.set_fixed(idx, true);
            }
            if (!c1.is_fixed(idx)) {
                result |= c1.set_fixed(idx, true);
            }
            if (!fixed[i]) {
                carry[i] = true;
                fixed[i] = true;
                backtrack = true;
            }
        }

        if (backtrack) {
            i = std::max(-1, i - 2);
        }
    }

    return result;
}

Result
FdpAddOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    FeasibleDomain& c0 = *d_children[0];
    FeasibleDomain& c1 = *d_children[1];
    FeasibleDomain& output = *d_self;

    // input => output
    result |= update_interval_by_plus(c0, c1, output);

    // output => input
    result |= update_interval_by_minus(output, c0, c1);
    result |= update_interval_by_minus(output, c1, c0);

    // we harmonize fixed bits and interval after interval propagation
    // result |= tighten_interval_by_fixed_bits(c0); // c1, output
    // result |= tighten_fixed_bits_by_interval(c0); // c1, output

    return result;
}

Result FdpAddOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_add);
    ++d_stats.d_num_fdp_add;

    // Assertions
    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());
    assert(d_children[0]->width() == d_self->width());

    Result result = Result::UNCHANGED;

    result |= fixed_bits_both_way();
    result |= interval_both_way();

    return result;
}

}  // namespace bzla::preprocess::pass::fdp
