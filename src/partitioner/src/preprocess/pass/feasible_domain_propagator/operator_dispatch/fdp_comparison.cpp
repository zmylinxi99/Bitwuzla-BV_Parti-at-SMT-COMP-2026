#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"

#include <sys/types.h>

#include <cassert>
#include <cstdint>
#include <utility>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

/* =========================================================== */

FdpLessThanOperator::FdpLessThanOperator(OperatorStatistics &stats,
                                         DomainVector &children,
                                         FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_ULT, "fdp_bv_ult", children, self),
      d_stats(stats) {
}

Result FdpLessThanOperator::tighten_output_by_inputs(FeasibleDomain &c0,
                                                     FeasibleDomain &c1,
                                                     FeasibleDomain &output) {
    assert(output.width() == 1 && output.is_fixed(0) == false);

    Result result = Result::UNCHANGED;
    BitVector c0_min = c0.is_interval_complementary() ? make_unsigned_min(c0) : c0.interval_min();
    BitVector c0_max = c0.is_interval_complementary() ? make_unsigned_max(c0) : c0.interval_max();
    BitVector c1_min = c1.is_interval_complementary() ? make_unsigned_min(c1) : c1.interval_min();
    BitVector c1_max = c1.is_interval_complementary() ? make_unsigned_max(c1) : c1.interval_max();

    // c0_min >= c1_max  =>  c0 < c1  must be false
    if (c0_min.compare(c1_max) >= 0) {
        result |= output.set_fixed(0, false);
    }

    // c0_max < c1_min  =>  c0 < c1  must be true
    if (c0_max.compare(c1_min) < 0) {
        result |= output.set_fixed(0, true);
    }

    return result;
}

Result FdpLessThanOperator::tighten_less_than_by_fixed_bits(FeasibleDomain &c0,
                                                            FeasibleDomain &c1) {
    assert(c0.width() == c1.width());
    assert(c0.is_interval_complementary() ||
           c1.is_interval_complementary());
    // at least one complementary interval
    // it includes 0000 and 1111, so we use fixed bits to tighten

    // BitVector c0_min = c0.is_interval_complementary() ? c0.interval_min() : make_unsigned_min(c0);
    // BitVector c1_max = c1.is_interval_complementary() ? c1.interval_max() : make_unsigned_max(c1);
    BitVector c0_min = make_unsigned_min(c0);
    BitVector c1_max = make_unsigned_max(c1);

    uint32_t width = c0.width();
    Result result = Result::UNCHANGED;

    if (c0_min.compare(c1_max) >= 0) {
        // conflict
        return Result::CONFLICT;
    }

    // tighten c0
    // stp version, we can more efficiently
    for (int i = width - 1; i >= 0; --i) {
        if (c0.is_fixed(i)) {
            assert(c0.get_value(i) == c0_min.bit(i));
            continue;
        }

        c0_min.set_bit(i, true);
        if (c0_min.compare(c1_max) >= 0) {
            c0_min.set_bit(i, false);
            result |= c0.set_fixed(i, false);
        }
        else {
            c0_min.set_bit(i, false);
            break;
        }
    }

    // tighten c1
    for (int i = width - 1; i >= 0; --i) {
        if (c1.is_fixed(i)) {
            assert(c1.get_value(i) == c1_max.bit(i));
            continue;
        }

        c1_max.set_bit(i, false);
        if (c1_max.compare(c0_min) <= 0) {
            c1_max.set_bit(i, true);
            result |= c1.set_fixed(i, true);
        }
        else {
            c1_max.set_bit(i, true);
            break;
        }
    }

    return result;
}

// We suppose output is true
Result FdpLessThanOperator::tighten_less_than(FeasibleDomain &c0,
                                              FeasibleDomain &c1) {
    Result result = Result::UNCHANGED;

    if (c0.is_interval_complementary() || c1.is_interval_complementary()) {
        // complementary interval has 0000 and 1111, so we use fixed bits to tighten
        return tighten_less_than_by_fixed_bits(c0, c1);
    }

    // both are non-complementary intervals, min/max is more precise

    if (c1.interval_max().is_zero()) {
        // c0 < 0  => conflict
        return Result::CONFLICT;
    }

    if (c0.interval_min().is_ones()) {
        // 1111 < c1 => conflict
        return Result::CONFLICT;
    }

    if (c0.interval_min().compare(c1.interval_max()) >= 0) {
        // c0 >= c1 => conflict
        return Result::CONFLICT;
    }

    // won't cause overflow, both are non-complementary intervals
    result |= c0.apply_interval_constraint(
        c0.interval_min(),
        c1.interval_max().bvdec(),
        false);
    result |= c1.apply_interval_constraint(
        c0.interval_min().bvinc(),
        c1.interval_max(),
        false);

    return result;
}

Result FdpLessThanOperator::tighten_less_than_equals_by_fixed_bits(FeasibleDomain &c0,
                                                                   FeasibleDomain &c1) {
    // BitVector c0_min = c0.is_interval_complementary() ? c0.interval_min() : make_unsigned_min(c0);
    // BitVector c1_max = c1.is_interval_complementary() ? c1.interval_max() : make_unsigned_max(c1);
    BitVector c0_min = make_unsigned_min(c0);
    BitVector c1_max = make_unsigned_max(c1);

    uint32_t width = c0.width();
    Result result = Result::UNCHANGED;

    if (c0_min.compare(c1_max) > 0) {
        // conflict
        return Result::CONFLICT;
    }

    // tighten c0
    for (int i = width - 1; i >= 0; --i) {
        if (c0.is_fixed(i)) {
            assert(c0.get_value(i) == c0_min.bit(i));
            continue;
        }

        c0_min.set_bit(i, true);
        if (c0_min.compare(c1_max) > 0) {
            c0_min.set_bit(i, false);
            result |= c0.set_fixed(i, false);
        }
        else {
            c0_min.set_bit(i, false);
            break;
        }
    }

    // tighten c1
    for (int i = width - 1; i >= 0; --i) {
        if (c1.is_fixed(i)) {
            assert(c1.get_value(i) == c1_max.bit(i));
            continue;
        }

        c1_max.set_bit(i, false);
        if (c1_max.compare(c0_min) < 0) {
            c1_max.set_bit(i, true);
            result |= c1.set_fixed(i, true);
        }
        else {
            c1_max.set_bit(i, true);
            break;
        }
    }

    return result;
}

Result FdpLessThanOperator::tighten_less_than_equals(FeasibleDomain &c0,
                                                     FeasibleDomain &c1) {
    Result result = Result::UNCHANGED;

    if (c0.is_interval_complementary() || c1.is_interval_complementary()) {
        // complementary interval has 0000 and 1111, so we use fixed bits to tighten
        return tighten_less_than_equals_by_fixed_bits(c0, c1);
    }

    // both are non-complementary intervals, min/max is more precise
    if (c0.interval_min().compare(c1.interval_max()) > 0) {
        // c0 > c1 => conflict
        return Result::CONFLICT;
    }

    result |= c0.apply_interval_constraint(
        c0.interval_min(),
        c1.interval_max(),
        false);
    result |= c1.apply_interval_constraint(
        c0.interval_min(),
        c1.interval_max(),
        false);
    return result;
}

Result
FdpLessThanOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_less_than);
    ++d_stats.d_num_fdp_less_than;

    // Assertions
    assert(d_self->width() == 1);
    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());

    Result result = Result::UNCHANGED;

    FeasibleDomain &c0 = *d_children[0];
    FeasibleDomain &c1 = *d_children[1];

    // Input => Output
    if (!d_self->is_fixed(0))
        result |= tighten_output_by_inputs(c0, c1, *d_self);

    // Output => Input
    if (d_self->is_fixed_one(0))
        result |= tighten_less_than(c0, c1);
    if (d_self->is_fixed_zero(0))
        result |= tighten_less_than_equals(c1, c0);

    return result;
}

// BV_SLT
bool FdpSignedLessThanOperator::using_signed_interval(const FeasibleDomain &domain) {
    uint32_t sign_bit = domain.width() - 1;
    if (domain.is_interval_complementary()) {
        if (domain.interval_max().bit(sign_bit) == 1 &&
            domain.interval_min().bit(sign_bit) == 0) {
            // [1xxx, 1111] U [0000, 0xxx]
            return true;
        }

        return false;
    }

    // [0xxx, 0xxx] or [1xxx, 1xxx]
    if (domain.interval_max().bit(sign_bit) ==
        domain.interval_min().bit(sign_bit)) {
        return true;
    }

    return false;
}

BitVector
FdpSignedLessThanOperator::signed_interval_min(const FeasibleDomain &domain) {
    assert(using_signed_interval(domain));
    if (domain.is_interval_complementary()) {
        return domain.interval_max();  //
    }
    // non-complementary interval
    return domain.interval_min();
}

BitVector
FdpSignedLessThanOperator::signed_interval_max(const FeasibleDomain &domain) {
    assert(using_signed_interval(domain));
    if (domain.is_interval_complementary()) {
        return domain.interval_min();
    }
    // non-complementary interval
    return domain.interval_max();
}

Result
FdpSignedLessThanOperator::tighten_output_by_inputs(FeasibleDomain &c0,
                                                    FeasibleDomain &c1,
                                                    FeasibleDomain &output) {
    Result result = Result::UNCHANGED;

    BitVector c0_min = using_signed_interval(c0) ? signed_interval_min(c0) : make_signed_min(c0);
    BitVector c0_max = using_signed_interval(c0) ? signed_interval_max(c0) : make_signed_max(c0);
    BitVector c1_min = using_signed_interval(c1) ? signed_interval_min(c1) : make_signed_min(c1);
    BitVector c1_max = using_signed_interval(c1) ? signed_interval_max(c1) : make_signed_max(c1);

    // c0_max < c1_min  =>  c0 < c1  must be true
    if (c0_max.signed_compare(c1_min) < 0) {
        result |= output.set_fixed(0, true);
    }

    // c0_min >= c1_max  =>  c0 < c1  must be false
    if (c0_min.signed_compare(c1_max) >= 0) {
        result |= output.set_fixed(0, false);
    }

    return result;
}

// We suppose output is true

Result
FdpSignedLessThanOperator::tighten_signed_less_than_by_fixed_bits(FeasibleDomain &c0,
                                                                  FeasibleDomain &c1) {
    BitVector c0_min = make_signed_min(c0);
    BitVector c1_max = make_signed_max(c1);

    // assert(c0_min.signed_compare(c1_max) <= 0);
    if (c0_min.signed_compare(c1_max) > 0) {
        // conflict
        return Result::CONFLICT;
    }

    uint32_t width = c0.width();
    uint32_t sign_bit = width - 1;

    Result result = Result::UNCHANGED;

    if (!c0.is_fixed(sign_bit)) {  // try to fix sign bit
        assert(c0_min.bit(sign_bit) == true);
        c0_min.set_bit(sign_bit, false);
        if (c0_min.signed_compare(c1_max) >= 0) {
            c0_min.set_bit(sign_bit, true);
            result |= c0.set_fixed(sign_bit, true);
        }
        else {
            c0_min.set_bit(sign_bit, true);
        }
    }

    if (!c1.is_fixed(sign_bit)) {  // try to fix sign bit
        assert(c1_max.bit(sign_bit) == false);
        c1_max.set_bit(sign_bit, true);
        if (c1_max.signed_compare(c0_min) <= 0) {
            c1_max.set_bit(sign_bit, false);
            result |= c1.set_fixed(sign_bit, false);
        }
        else {
            c1_max.set_bit(sign_bit, false);
        }
    }

    // tighten c0
    for (int i = width - 2; i >= 0; --i) {
        if (c0.is_fixed(i)) {
            assert(c0.get_value(i) == c0_min.bit(i));
            continue;
        }
        assert(c0_min.bit(i) == false);
        c0_min.set_bit(i, true);
        if (c0_min.signed_compare(c1_max) >= 0) {
            c0_min.set_bit(i, false);
            result |= c0.set_fixed(i, false);
        }
        else {
            c0_min.set_bit(i, false);
            break;
        }
    }

    // tighten c1
    for (int i = width - 2; i >= 0; --i) {
        if (c1.is_fixed(i)) {
            assert(c1.get_value(i) == c1_max.bit(i));
            continue;
        }
        assert(c1_max.bit(i) == true);
        c1_max.set_bit(i, false);
        if (c1_max.signed_compare(c0_min) <= 0) {
            c1_max.set_bit(i, true);
            result |= c1.set_fixed(i, true);
        }
        else {
            c1_max.set_bit(i, true);
            break;
        }
    }

    return result;
}

Result
FdpSignedLessThanOperator::tighten_signed_less_than(FeasibleDomain &c0,
                                                    FeasibleDomain &c1) {
    Result result = Result::UNCHANGED;

    if (!using_signed_interval(c0) || !using_signed_interval(c1)) {
        // complementary interval has [0111] and [1000], so we use fixed bits to tighten
        return tighten_signed_less_than_by_fixed_bits(c0, c1);
    }

    // min/max is more precise

    BitVector c0_min = signed_interval_min(c0);
    BitVector c0_max = signed_interval_max(c0);
    BitVector c1_min = signed_interval_min(c1);
    BitVector c1_max = signed_interval_max(c1);
    [[maybe_unused]] uint32_t sign_bit = c0.width() - 1;

    assert(c0_min.signed_compare(c0_max) <= 0);
    assert(c1_min.signed_compare(c1_max) <= 0);

    // c0_min < c1_max
    if (c0_min.signed_compare(c1_max) >= 0) {
        return Result::CONFLICT;
    }

    // We can only tighten c0_max < c1_max and c1_min > c0_min
    // [c0_min, c1_max - 1] for c0 and [c0_min + 1, c1_max] for c1
    // we use signed comparison and don't worry about overflow
    bool update_c0 = (c0_max.signed_compare(c1_max.bvdec()) > 0);
    bool update_c1 = (c1_min.signed_compare(c0_min.bvinc()) < 0);

    if (!update_c0 && !update_c1) {
        return Result::UNCHANGED;
    }

    if (update_c0) {
        BitVector tmp_min = c0_min;
        BitVector tmp_max = c1_max.bvdec();
        bool complementary = false;

        assert(tmp_min.signed_compare(tmp_max) <= 0);
        if (tmp_max.compare(tmp_min) < 0) {
            // need complementary interval
            assert(tmp_min.bit(sign_bit) == 1 && tmp_max.bit(sign_bit) == 0);
            complementary = true;
            std::swap(tmp_min, tmp_max);  // min <= max
        }
        // we make sure tmp_min <= tmp_max by swap
        assert(tmp_min.compare(tmp_max) <= 0);
        result |= c0.apply_interval_constraint(tmp_min, tmp_max, complementary);
    }

    if (update_c1) {
        BitVector tmp_min = c0_min.bvinc();
        BitVector tmp_max = c1_max;
        bool complementary = false;

        assert(tmp_min.signed_compare(tmp_max) <= 0);
        if (tmp_max.compare(tmp_min) < 0) {
            // need complementary interval
            assert(tmp_min.bit(sign_bit) == 1 && tmp_max.bit(sign_bit) == 0);
            complementary = true;
            std::swap(tmp_min, tmp_max);  // min <= max
        }
        // we make sure tmp_min <= tmp_max by swap
        assert(tmp_min.compare(tmp_max) <= 0);
        result |= c1.apply_interval_constraint(tmp_min, tmp_max, complementary);
    }

    return result;
}

Result
FdpSignedLessThanOperator::tighten_signed_less_than_equals_by_fixed_bits(FeasibleDomain &c0,
                                                                         FeasibleDomain &c1) {
    BitVector c0_min = make_signed_min(c0);
    BitVector c1_max = make_signed_max(c1);

    // assert(c0_min.signed_compare(c1_max) <= 0);
    if (c0_min.signed_compare(c1_max) > 0) {
        return Result::CONFLICT;
    }

    uint32_t width = c0.width();
    uint32_t sign_bit = width - 1;

    Result result = Result::UNCHANGED;

    if (!c0.is_fixed(sign_bit)) {  // try to fix sign bit
        assert(c0_min.bit(sign_bit) == true);
        c0_min.set_bit(sign_bit, false);
        if (c0_min.signed_compare(c1_max) > 0) {
            c0_min.set_bit(sign_bit, true);
            result |= c0.set_fixed(sign_bit, true);
        }
        else {
            c0_min.set_bit(sign_bit, true);
        }
    }

    if (!c1.is_fixed(sign_bit)) {  // try to fix sign bit
        assert(c1_max.bit(sign_bit) == false);
        c1_max.set_bit(sign_bit, true);
        if (c1_max.signed_compare(c0_min) < 0) {
            c1_max.set_bit(sign_bit, false);
            result |= c1.set_fixed(sign_bit, false);
        }
        else {
            c1_max.set_bit(sign_bit, false);
        }
    }

    // tighten c0
    for (int i = width - 2; i >= 0; --i) {
        if (c0.is_fixed(i)) {
            assert(c0.get_value(i) == c0_min.bit(i));
            continue;
        }
        assert(c0_min.bit(i) == false);
        c0_min.set_bit(i, true);
        if (c0_min.signed_compare(c1_max) > 0) {
            c0_min.set_bit(i, false);
            result |= c0.set_fixed(i, false);
        }
        else {
            c0_min.set_bit(i, false);
            break;
        }
    }

    // tighten c1
    for (int i = width - 2; i >= 0; --i) {
        if (c1.is_fixed(i)) {
            assert(c1.get_value(i) == c1_max.bit(i));
            continue;
        }

        assert(c1_max.bit(i) == true);
        c1_max.set_bit(i, false);
        if (c1_max.signed_compare(c0_min) < 0) {
            c1_max.set_bit(i, true);
            result |= c1.set_fixed(i, true);
        }
        else {
            c1_max.set_bit(i, true);
            break;
        }
    }
    return result;
}

Result
FdpSignedLessThanOperator::tighten_signed_less_than_equals(FeasibleDomain &c0,
                                                           FeasibleDomain &c1) {
    Result result = Result::UNCHANGED;

    if (!using_signed_interval(c0) || !using_signed_interval(c1)) {
        // complementary interval has [0111] and [1000], so we use fixed bits to tighten
        return tighten_signed_less_than_equals_by_fixed_bits(c0, c1);
    }

    // min/max is more precise

    BitVector c0_min = signed_interval_min(c0);
    BitVector c0_max = signed_interval_max(c0);
    BitVector c1_min = signed_interval_min(c1);
    BitVector c1_max = signed_interval_max(c1);
    [[maybe_unused]] uint32_t sign_bit = c0.width() - 1;

    assert(c0_min.signed_compare(c0_max) <= 0);
    assert(c1_min.signed_compare(c1_max) <= 0);

    // c0_min <= c1_max
    if (c0_min.signed_compare(c1_max) > 0) {
        return Result::CONFLICT;
    }

    // We can only tighten c0_max <= c1_max and c1_min >= c0_min
    // That means [c0_min, c1_max] for both c0 and c1
    bool update_c0 = (c0_max.signed_compare(c1_max) > 0);
    bool update_c1 = (c1_min.signed_compare(c0_min) < 0);

    if (!update_c0 && !update_c1) {
        return Result::UNCHANGED;
    }

    BitVector tmp_min = c0_min;
    BitVector tmp_max = c1_max;
    bool complementary = false;

    assert(tmp_min.signed_compare(tmp_max) <= 0);
    if (tmp_max.compare(tmp_min) < 0) {
        // need complementary interval
        assert(tmp_min.bit(sign_bit) == 1 && tmp_max.bit(sign_bit) == 0);
        complementary = true;
        std::swap(tmp_min, tmp_max);  // min <= max
    }
    // we make sure tmp_min <= tmp_max by swap
    assert(tmp_min.compare(tmp_max) <= 0);

    if (update_c0) {
        result |= c0.apply_interval_constraint(tmp_min, tmp_max, complementary);
    }

    if (update_c1) {
        result |= c1.apply_interval_constraint(tmp_min, tmp_max, complementary);
    }
    return result;
}

FdpSignedLessThanOperator::FdpSignedLessThanOperator(OperatorStatistics &stats,
                                                     DomainVector &children,
                                                     FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_SLT, "fdp_bv_slt", children, self),
      d_stats(stats) {}

Result
FdpSignedLessThanOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_signed_less_than);
    ++d_stats.d_num_fdp_signed_less_than;

    // Assertions
    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());
    assert(d_self->width() == 1);

    Result result = Result::UNCHANGED;
    FeasibleDomain &c0 = *d_children[0];
    FeasibleDomain &c1 = *d_children[1];

    // Input => Output
    if (!d_self->is_fixed(0))
        result |= tighten_output_by_inputs(c0, c1, *d_self);

    // Output => Input
    if (d_self->is_fixed_one(0))
        result |= tighten_signed_less_than(c0, c1);

    if (d_self->is_fixed_zero(0))
        result |= tighten_signed_less_than_equals(c1, c0);

    return result;
}

FdpEqOperator::FdpEqOperator(OperatorStatistics &stats,
                             DomainVector &children,
                             FeasibleDomain *self)
    : FdpOperator(node::Kind::EQUAL, "fdp_eq", children, self),
      d_stats(stats) {}

Result
FdpEqOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;
    if (d_self->is_totally_fixed()) {  // UP -> DOWN
        if (d_self->get_value(0)) {
            for (size_t i = 0, isz = d_children.at(0)->width(); i < isz; ++i) {
                if (d_children.at(0)->is_fixed(i))
                    result |= d_children.at(1)->set_fixed(i, d_children.at(0)->get_value(i));
                else if (d_children.at(1)->is_fixed(i))
                    result |= d_children.at(0)->set_fixed(i, d_children.at(1)->get_value(i));
            }
        }
        else {
            bool left_fixed = d_children.at(0)->is_totally_fixed();
            bool right_fixed = d_children.at(1)->is_totally_fixed();
            if (!left_fixed && !right_fixed)
                return result;
            else if (left_fixed && right_fixed) {
                bool conflict = true;
                for (size_t i = 0, isz = d_children.at(0)->width(); i < isz; ++i) {
                    if (d_children.at(0)->get_value(i) != d_children.at(1)->get_value(i)) {
                        conflict = false;
                        break;
                    }
                }
                if (conflict)
                    return Result::CONFLICT;
                else
                    return result;
            }
            else {
                bool suitable = false;
                size_t index = 0;
                for (size_t i = 0, isz = d_children.at(0)->width(); i < isz; ++i) {
                    if (!d_children.at(0)->is_fixed(i) || !d_children.at(1)->is_fixed(i)) {
                        if (suitable)
                            return result;
                        suitable = true;
                        index = i;
                    }
                    else if (d_children.at(0)->get_value(i) != d_children.at(1)->get_value(i)) {
                        return result;
                    }
                }

                if (suitable) {
                    // single unknown
                    if (d_children.at(0)->is_fixed(index))
                        result |= d_children.at(1)->set_fixed(index, !d_children.at(0)->get_value(index));
                    else
                        result |= d_children.at(0)->set_fixed(index, !d_children.at(1)->get_value(index));
                }
            }
        }
    }
    else {  // DOWN -> UP
        bool is_equal = true;
        for (size_t i = 0, isz = d_children.at(0)->width(); i < isz; ++i) {
            if (d_children.at(0)->is_fixed(i) && d_children.at(1)->is_fixed(i)) {
                if ((d_children.at(0)->get_value(i) == d_children.at(1)->get_value(i)))
                    continue;
                else {
                    result |= d_self->set_fixed(0, false);
                    return result;
                }
            }
            else {
                is_equal = false;
                break;
            }
        }

        if (is_equal)
            result |= d_self->set_fixed(0, true);
    }
    return result;
}

Result
FdpEqOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    if (d_self->is_totally_fixed()) {
        if (d_self->get_value(0)) {
            // FeasibleDomain holds min <= max
            assert(d_children.at(0)->interval_min().compare(d_children.at(0)->interval_max()) <= 0);
            assert(d_children.at(1)->interval_min().compare(d_children.at(1)->interval_max()) <= 0);

            result |= d_children.at(0)->apply_interval_constraint(d_children.at(1)->interval_min(),
                                                                  d_children.at(1)->interval_max(),
                                                                  d_children.at(1)->is_interval_complementary());

            result |= d_children.at(1)->apply_interval_constraint(d_children.at(0)->interval_min(),
                                                                  d_children.at(0)->interval_max(),
                                                                  d_children.at(0)->is_interval_complementary());
        }
    }
    else {
        if (intersects(d_children.at(0)->interval_min(),
                       d_children.at(0)->interval_max(),
                       d_children.at(0)->is_interval_complementary(),
                       d_children.at(1)->interval_min(),
                       d_children.at(1)->interval_max(),
                       d_children.at(1)->is_interval_complementary()) == false) {
            // no intersection
            result |= d_self->set_fixed(0, false);
        }
    }
    return result;
}

Result
FdpEqOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_eq);
    ++d_stats.d_num_fdp_eq;

    // if (d_self->is_totally_fixed())
    //     return Result::UNCHANGED;
    Result result = Result::UNCHANGED;
    result |= fixed_bits_both_way();
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }
    result |= interval_both_way();
    return result;
}
}  // namespace bzla::preprocess::pass::fdp
