#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_division.h"

#include <algorithm>
#include <cassert>
#include <utility>

#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

namespace {

struct Bounds {
    BitVector d_min;
    BitVector d_max;
};

struct DivState {
    Bounds d_top;     // dividend
    Bounds d_bottom;  // divisor
    Bounds d_quot;    // quotient
    Bounds d_rem;     // remainder
};

Result tighten_min(Bounds& bounds, const BitVector& candidate) {
    if (candidate.compare(bounds.d_max) > 0) {
        return Result::CONFLICT;
    }
    if (bounds.d_min.compare(candidate) < 0) {
        bounds.d_min = candidate;
        return Result::CHANGED;
    }
    return Result::UNCHANGED;
}

Result tighten_max(Bounds& bounds, const BitVector& candidate) {
    if (candidate.compare(bounds.d_min) < 0) {
        return Result::CONFLICT;
    }
    if (bounds.d_max.compare(candidate) > 0) {
        bounds.d_max = candidate;
        return Result::CHANGED;
    }
    return Result::UNCHANGED;
}

bool safe_add(const BitVector& lhs, const BitVector& rhs, BitVector& out) {
    if (lhs.is_uadd_overflow(rhs)) {
        return false;
    }
    out = lhs.bvadd(rhs);
    return true;
}

bool safe_sub(const BitVector& lhs, const BitVector& rhs, BitVector& out) {
    if (lhs.is_usub_overflow(rhs)) {
        return false;
    }
    out = lhs.bvsub(rhs);
    return true;
}

bool safe_mul(const BitVector& lhs, const BitVector& rhs, BitVector& out) {
    if (lhs.is_umul_overflow(rhs)) {
        return false;
    }
    out = lhs.bvmul(rhs);
    return true;
}

bool safe_div(const BitVector& numerator,
              const BitVector& denominator,
              BitVector& quotient,
              BitVector& remainder) {
    if (denominator.is_zero()) {
        return false;
    }
    numerator.bvudivurem(denominator, &quotient, &remainder);
    return true;
}

bool safe_inc(const BitVector& value, BitVector& out) {
    if (value.is_ones()) {
        return false;
    }
    out = value.bvinc();
    return true;
}

bool safe_dec(const BitVector& value, BitVector& out) {
    if (value.is_zero()) {
        return false;
    }
    out = value.bvdec();
    return true;
}

Bounds make_bounds(const FeasibleDomain& domain) {
    if (!domain.is_interval_complementary()) {
        Bounds bounds{domain.interval_min(), domain.interval_max()};
        return bounds;
    }
    else {
        Bounds bounds{make_unsigned_min(domain), make_unsigned_max(domain)};
        return bounds;
    }
}

DivState make_state(FeasibleDomain& dividend,
                    FeasibleDomain& divisor,
                    FeasibleDomain& output,
                    bool quotient_output) {
    const uint32_t width = dividend.width();
    const BitVector zero = BitVector::mk_zero(width);
    const BitVector ones = BitVector::mk_ones(width);

    DivState state;
    state.d_top = make_bounds(dividend);
    state.d_bottom = make_bounds(divisor);
    if (quotient_output) {
        state.d_quot = make_bounds(output);
        state.d_rem = Bounds{zero, ones};
    }
    else {
        state.d_quot = Bounds{zero, ones};
        state.d_rem = make_bounds(output);
    }
    return state;
}

Result apply_bounds(FeasibleDomain& domain, const Bounds& bounds) {
    // If we make sure we use non-complementary intervals here
    // we need to ensure min <= max
    assert(bounds.d_min.compare(bounds.d_max) <= 0);

    Result result = domain.apply_interval_constraint(
        bounds.d_min, bounds.d_max, false);
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }
    result |= domain.tighten_fixed_bits_by_interval();

    return result;
}

// bool needs_retry(const DivState& before, const DivState& after) {
//     return before.d_top.d_min != after.d_top.d_min ||
//            before.d_top.d_max != after.d_top.d_max ||
//            before.d_bottom.d_min != after.d_bottom.d_min ||
//            before.d_bottom.d_max != after.d_bottom.d_max ||
//            before.d_quot.d_min != after.d_quot.d_min ||
//            before.d_quot.d_max != after.d_quot.d_max ||
//            before.d_rem.d_min != after.d_rem.d_min ||
//            before.d_rem.d_max != after.d_rem.d_max;
// }

Result enforce_equality(FeasibleDomain& a, FeasibleDomain& b) {
    Result result = Result::UNCHANGED;
    Result interval_result = align_interval(a, b);
    if (is_conflict(interval_result)) {
        return Result::CONFLICT;
    }
    result |= interval_result;

    for (uint32_t i = 0, width = a.width(); i < width; ++i) {
        if (a.is_fixed(i) && b.is_fixed(i)) {
            if (a.get_value(i) != b.get_value(i)) {
                return Result::CONFLICT;
            }
            continue;
        }
        if (a.is_fixed(i)) {
            Result r = b.set_fixed(i, a.get_value(i));
            if (is_conflict(r)) {
                return Result::CONFLICT;
            }
            result |= r;
        }
        else if (b.is_fixed(i)) {
            Result r = a.set_fixed(i, b.get_value(i));
            if (is_conflict(r)) {
                return Result::CONFLICT;
            }
            result |= r;
        }
    }
    return result;
}

bool domains_can_be_equal(const FeasibleDomain& a, const FeasibleDomain& b) {
    if (a.width() != b.width()) {
        return false;
    }
    FeasibleDomain lhs(a);
    FeasibleDomain rhs(b);
    Result eq = enforce_equality(lhs, rhs);
    return !is_conflict(eq);
}

Result propagate_unsigned_division(bool quotient_output,
                                   FeasibleDomain& dividend,
                                   FeasibleDomain& divisor,
                                   FeasibleDomain& output) {
    // This logic is ported/translated from the STP project.
    // This function will also update intervals in the feasible domains. (see apply_bounds)

    const uint32_t width = dividend.width();
    assert(divisor.width() == width);
    assert(output.width() == width);

    const BitVector one = BitVector::mk_one(width);
    const BitVector ones = BitVector::mk_ones(width);

    Result result = Result::UNCHANGED;
    bool zero_possible = feasible_domain_holds_unsigned(divisor, 0);

    bool zero_forced = false;
    if (!divisor.is_interval_complementary()) {
        zero_forced =
            zero_possible && divisor.interval_min().is_zero() && divisor.interval_max().is_zero();
    }

    if (zero_forced) {
        if (quotient_output) {
            result |= output.set_constant(ones);
            return result;
        }
        result |= enforce_equality(dividend, output);
        return result;
    }

    if (zero_possible) {
        bool zero_branch_feasible = true;
        if (quotient_output) {
            zero_branch_feasible = feasible_domain_holds_value(output, ones);
        }
        else {
            zero_branch_feasible = domains_can_be_equal(dividend, output);
        }
        if (!zero_branch_feasible) {
            result |= divisor.apply_interval_constraint(one, ones, false);
            if (is_conflict(result)) {
                return Result::CONFLICT;
            }
            
            result |= divisor.tighten_fixed_bits_by_interval();
            if (is_conflict(result)) {
                return Result::CONFLICT;
            }
            zero_possible = feasible_domain_holds_unsigned(divisor, 0);
        }
    }

    DivState state = make_state(dividend, divisor, output, quotient_output);

    if (zero_possible) {
        return result;
    }

    if (state.d_bottom.d_min.is_zero()) {
        state.d_bottom.d_min = one;
    }

    for (bool changed = true; changed;) {
        changed = false;

        BitVector tmp, q, r;

        // quot = (top - rem) / bottom
        // (1) quot >= floor((top.min - rem.min) / bottom.max).
        if (safe_sub(state.d_top.d_min, state.d_rem.d_min, tmp) &&
            safe_div(tmp, state.d_bottom.d_max, q, r)) {
            Result adj = tighten_min(state.d_quot, q);
            if (is_conflict(adj)) {
                return Result::CONFLICT;
            }
            changed |= is_changed(adj);
        }

        // quot = (top - rem) / bottom
        // (2) rem >= 0 => quot <= floor(top.max / bottom.min).
        if (safe_div(state.d_top.d_max, state.d_bottom.d_min, q, r)) {
            Result adj = tighten_max(state.d_quot, q);
            if (is_conflict(adj)) {
                return Result::CONFLICT;
            }
            changed |= is_changed(adj);
        }

        BitVector product;
        // top = quot * bottom + rem
        // (3) rem < bottom => top <= quot.max * bottom.max + (bottom.max - 1).
        if (safe_mul(state.d_quot.d_max, state.d_bottom.d_max, product)) {
            BitVector bottom_minus_one;
            if (safe_dec(state.d_bottom.d_max, bottom_minus_one)) {
                BitVector candidate;
                if (safe_add(product, bottom_minus_one, candidate)) {
                    Result adj = tighten_max(state.d_top, candidate);
                    if (is_conflict(adj)) {
                        return Result::CONFLICT;
                    }
                    changed |= is_changed(adj);
                }
            }
        }

        // top = quot * bottom + rem
        // (4) rem >= 0 => top >= quot.min * bottom.min.
        if (safe_mul(state.d_quot.d_min, state.d_bottom.d_min, product)) {
            Result adj = tighten_min(state.d_top, product);
            if (is_conflict(adj)) {
                return Result::CONFLICT;
            }
            changed |= is_changed(adj);
        }

        // bottom = (top - rem) / quot
        // (5) quot > 0 and rem >= 0 => bottom <= floor(top.max / quot.min).
        if (state.d_quot.d_min.compare(one) >= 0 &&
            safe_div(state.d_top.d_max, state.d_quot.d_min, q, r)) {
            Result adj = tighten_max(state.d_bottom, q);
            if (is_conflict(adj)) {
                return Result::CONFLICT;
            }
            changed |= is_changed(adj);
        }

        BitVector top_plus_one;
        BitVector quot_plus_one;
        // top = quot * bottom + rem
        // rem < bottom => top <= quot * bottom + bottom - 1 => bottom >= (top + 1) / (quot + 1)
        // (6) bottom >= ceil((top.min + 1) / (quot.max + 1)).
        if (safe_add(state.d_top.d_min, one, top_plus_one) &&
            safe_add(state.d_quot.d_max, one, quot_plus_one) &&
            safe_div(top_plus_one, quot_plus_one, q, r)) {
            BitVector ceil_q = q;
            if (!r.is_zero()) {
                // Add one when division leaves a remainder to realize the ceil.
                if (!safe_inc(q, ceil_q)) {
                    return Result::CONFLICT;
                }
            }
            if (ceil_q.compare(one) >= 0) {
                Result adj = tighten_min(state.d_bottom, ceil_q);
                if (is_conflict(adj)) {
                    return Result::CONFLICT;
                }
                changed |= is_changed(adj);
            }
        }

        // #TODO: This part is different from STP
        //        stp logic: a / b = q, rem (r)
        //        stp calls LessThanBothWays(r < b) to fix rem and bottom.
        //        stp calls MultiplyBothWays(times = b * q) to fix bottom and quotient.
        //        stp calls AddBothWays(a = times + r) to fix top and rem.

        // Additional deductions from top = quot * bottom + rem with 0 <= rem < bottom.
        if (!quotient_output) {
            BitVector rem_max_candidate;
            // Enforce rem <= bottom.max - 1.
            if (safe_dec(state.d_bottom.d_max, rem_max_candidate)) {
                Result adj = tighten_max(state.d_rem, rem_max_candidate);
                if (is_conflict(adj)) {
                    return Result::CONFLICT;
                }
                changed |= is_changed(adj);
            }
            // Enforce rem <= top.max.
            {
                Result adj = tighten_max(state.d_rem, state.d_top.d_max);
                if (is_conflict(adj)) {
                    return Result::CONFLICT;
                }
                changed |= is_changed(adj);
            }

            BitVector qmax_bmax;
            // top = quot * bottom + rem
            // rem >= top.min - quot.max * bottom.max.
            if (safe_mul(state.d_quot.d_max, state.d_bottom.d_max, qmax_bmax) &&
                safe_sub(state.d_top.d_min, qmax_bmax, tmp)) {
                Result adj = tighten_min(state.d_rem, tmp);
                if (is_conflict(adj)) {
                    return Result::CONFLICT;
                }
                changed |= is_changed(adj);
            }

            // Keep top.min >= rem.min.
            Result align_top = tighten_min(state.d_top, state.d_rem.d_min);
            if (is_conflict(align_top)) {
                return Result::CONFLICT;
            }
            if (is_changed(align_top)) {
                changed = true;
            }
        }

        // If conflict happens, it should be return conflict during tighten_* calls.
        assert(state.d_quot.d_min.compare(state.d_quot.d_max) <= 0 &&
               state.d_bottom.d_min.compare(state.d_bottom.d_max) <= 0 &&
               state.d_top.d_min.compare(state.d_top.d_max) <= 0 &&
               state.d_rem.d_min.compare(state.d_rem.d_max) <= 0);
    }

    result |= apply_bounds(dividend, state.d_top);
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }

    result |= apply_bounds(divisor, state.d_bottom);
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }

    if (quotient_output) {
        result |= apply_bounds(output, state.d_quot);
        if (is_conflict(result)) {
            return Result::CONFLICT;
        }
    }
    else {
        result |= apply_bounds(output, state.d_rem);
        if (is_conflict(result)) {
            return Result::CONFLICT;
        }
    }

    // It should be redundant to check needs_retry here, if 'needs_retry' returns true,
    // some bits should be fixed in the propagation framework, which will lead call this function again.

    // DivState updated = make_state(dividend, divisor, output, quotient_output);
    // if (needs_retry(state, updated)) {
    //     // Some bounds have been tightened, return CHANGED to request another round.
    //     result |= Result::CHANGED;
    // }

    return result;
}

}  // namespace

FdpDivOperator::FdpDivOperator(OperatorStatistics& stats,
                               DomainVector& children,
                               FeasibleDomain* self)
    : FdpOperator(node::Kind::BV_UDIV, "fdp_bv_udiv", children, self),
      d_stats(stats) {}

Result
FdpDivOperator::fixed_bits_both_way() {
    return propagate_unsigned_division(true, *d_children[0], *d_children[1], *d_self);
}

Result
FdpDivOperator::interval_both_way() {
    // #TODO: implement interval propagation for division.
    //  Result ret = Result::UNCHANGED;
    //  FeasibleDomain& c0 = *d_children[0];
    //  FeasibleDomain& c1 = *d_children[1];
    //  FeasibleDomain& output = *d_self;
    //  // input => output
    //  // output = c0 / c1
    //  ret |= update_interval_by_division(c0, c1, output);

    // // output => input
    // // c0 = output * c1
    // ret |= update_interval_by_multiply(output, c1, c0);
    // // c1 = c0 / output
    // ret |= update_interval_by_division(c0, output, c1);
    // return ret;
    return Result::UNCHANGED;
}

Result
FdpDivOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_div);
    ++d_stats.d_num_fdp_div;

    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());
    assert(d_children[0]->width() == d_self->width());

    Result result = fixed_bits_both_way();
    // if (is_conflict(result)) {
    //     return Result::CONFLICT;
    // }
    // result |= interval_both_way();
    return result;
}

FdpRemOperator::FdpRemOperator(OperatorStatistics& stats,
                               DomainVector& children,
                               FeasibleDomain* self)
    : FdpOperator(node::Kind::BV_UREM, "fdp_bv_urem", children, self),
      d_stats(stats) {}

Result
FdpRemOperator::fixed_bits_both_way() {
    assert(d_children.size() == 2);
    return propagate_unsigned_division(false, *d_children[0], *d_children[1], *d_self);
}

Result
FdpRemOperator::interval_both_way() {
    // Interval refinement for remainder is already handled inside
    // propagate_unsigned_division(false, ...) invoked from
    // fixed_bits_both_way(). The previous implementation mis-modeled
    // the relationship by treating the remainder as a quotient and
    // multiplying it with the divisor, which could introduce
    // unsound tightening. Keep this hook as a no-op to avoid
    // reintroducing the incorrect propagation.
    return Result::UNCHANGED;
}

Result
FdpRemOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_rem);
    ++d_stats.d_num_fdp_rem;

    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());
    assert(d_children[0]->width() == d_self->width());

    Result result = fixed_bits_both_way();

    // if (is_conflict(result)) {
    //     return Result::CONFLICT;
    // }
    // result |= interval_both_way();
    return result;
}

}  // namespace bzla::preprocess::pass::fdp
