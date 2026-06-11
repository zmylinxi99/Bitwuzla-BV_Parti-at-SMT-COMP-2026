#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <ostream>
#include <sstream>
#include <utility>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

namespace {

// constexpr uint32_t kDomainEncodeThres = 3; // n means at least (1 - 1 / (1 << n)) reduction

const char *
state_to_cstr(Result state) {
    switch (state) {
        case Result::UNCHANGED:
            return "unchanged";
        case Result::CHANGED:
            return "changed";
        case Result::UPDATED:
            return "updated";
        case Result::CONFLICT:
            return "conflict";
    }
    return "?";
}

/**
 * This funtion is same as FeasibleDomain::get_interval_width()
 * and return the size of the interval - 1.
 */
BitVector
compute_interval_size(const BitVector &min,
                      const BitVector &max,
                      bool complementary) {
    assert(min.size() == max.size());
    assert(max.compare(min) >= 0);  // we ensure max >= min
    if (!complementary) {
        // [3, 5], 3 bit: 5 - 3 = 2
        return max.bvsub(min);
    }
    else {
        // [0, 3] \cup [5, 7], 3 bit: 3 - 5 = -2 = 6
        return min.bvsub(max);
    }
}

}  // namespace

const char *
mark_to_cstr(EncodingMark mark) {
    switch (mark) {
        case EncodingMark::NONE:
            return "none";
        case EncodingMark::BITS:
            return "bits";
        case EncodingMark::INTERVAL:
            return "interval";
        case EncodingMark::BOTH:
            return "both";
        case EncodingMark::REMOVED:
            return "removed";
    }
    return "?";
}

BitVector
FeasibleDomain::build_value_from_fixed_bits() const {
    assert(is_totally_fixed());
    BitVector value = BitVector::mk_zero(d_width);
    for (uint32_t i = 0; i < d_width; ++i) {
        if (d_fixed[i] && d_value[i]) {
            value.set_bit(i, true);
        }
    }
    return value;
}

void FeasibleDomain::sync_interval_if_totally_fixed() {
    if (!is_totally_fixed()) {
        return;
    }
    BitVector value = build_value_from_fixed_bits();
    d_interval_min = value;
    d_interval_max = value;
    d_complementary = false;
    d_last_interval_size = BitVector::mk_zero(d_width);
}

BitVector FeasibleDomain::get_fixed_value() const {
    assert(is_totally_fixed());
    return build_value_from_fixed_bits();
}

std::ostream &operator<<(std::ostream &out,
                         const FeasibleDomain &domain) {
    out << " fb: ";
    for (int32_t i = domain.width() - 1; i >= 0; --i) {
        if (domain.is_fixed(i)) {
            out << (domain.is_fixed_one(i) ? '1' : '0');
        }
        else {
            out << '*';
        }
    }
    out << "   compl: " << (domain.is_interval_complementary() ? "true" : "false");
    out << "\n";
    out << "min: " << domain.interval_min() << "\n";
    out << "max: " << domain.interval_max() << "\n";

    return out;
}

FeasibleDomain::FeasibleDomain(uint32_t width)
    : d_pending_state(Result::UNCHANGED),
      d_width(width),
      d_fixed_count(0),
      d_complementary(false),
      d_interval_max(BitVector::mk_ones(width)),
      d_interval_min(BitVector::mk_zero(width)),
      d_last_interval_size(BitVector::mk_ones(width)) {
    assert(width > 0);
    d_fixed = std::make_unique<bool[]>(width);
    d_value = std::make_unique<bool[]>(width);
    std::fill_n(d_fixed.get(), width, false);
    std::fill_n(d_value.get(), width, false);
}

FeasibleDomain::~FeasibleDomain() = default;

FeasibleDomain::FeasibleDomain(const FeasibleDomain &other)
    : d_pending_state(Result::UNCHANGED),
      d_width(other.d_width),
      d_fixed_count(other.d_fixed_count),
      d_complementary(other.d_complementary),
      d_interval_max(other.d_interval_max),
      d_interval_min(other.d_interval_min),
      d_last_interval_size(other.d_last_interval_size) {
    d_fixed = std::make_unique<bool[]>(d_width);
    d_value = std::make_unique<bool[]>(d_width);
    std::copy_n(other.d_fixed.get(), d_width, d_fixed.get());
    std::copy_n(other.d_value.get(), d_width, d_value.get());
}

void FeasibleDomain::copy_from(const FeasibleDomain &domain) {
    assert(d_width == domain.d_width);
    d_fixed_count = domain.d_fixed_count;
    std::copy_n(domain.d_fixed.get(), d_width, d_fixed.get());
    std::copy_n(domain.d_value.get(), d_width, d_value.get());
    d_interval_max = domain.d_interval_max;
    d_interval_min = domain.d_interval_min;
    d_complementary = domain.d_complementary;
    d_last_interval_size = domain.d_last_interval_size;
    // Keep the representation consistent: when all bits are fixed, the interval
    // must match the fixed value. This can be violated by callers that restore
    // only bit information (e.g., via overwrite(BITS)) before any tightening is
    // applied.
    sync_interval_if_totally_fixed();
    d_pending_state = Result::UNCHANGED;
}

void FeasibleDomain::reset() {
    d_fixed_count = 0;
    std::fill_n(d_fixed.get(), d_width, false);
    std::fill_n(d_value.get(), d_width, false);
    d_interval_max = BitVector::mk_ones(d_width);
    d_interval_min = BitVector::mk_zero(d_width);
    d_complementary = false;
    d_last_interval_size = BitVector::mk_ones(d_width);
    d_pending_state = Result::UNCHANGED;
}

void FeasibleDomain::reset(const FeasibleDomain &domain, EncodingMark mark) {
    if (mark == EncodingMark::BITS || mark == EncodingMark::BOTH) {
        d_fixed_count = domain.d_fixed_count;
        std::copy_n(domain.d_fixed.get(), d_width, d_fixed.get());
        std::copy_n(domain.d_value.get(), d_width, d_value.get());
    }
    else {
        d_fixed_count = 0;
        std::fill_n(d_fixed.get(), d_width, false);
        std::fill_n(d_value.get(), d_width, false);
    }

    if (mark == EncodingMark::INTERVAL || mark == EncodingMark::BOTH) {
        d_interval_max = domain.d_interval_max;
        d_interval_min = domain.d_interval_min;
        d_complementary = domain.d_complementary;
        d_last_interval_size = domain.d_last_interval_size;
    }
    else {
        d_interval_max = BitVector::mk_ones(d_width);
        d_interval_min = BitVector::mk_zero(d_width);
        d_complementary = false;
        d_last_interval_size = BitVector::mk_ones(d_width);
    }
    sync_interval_if_totally_fixed();
    d_pending_state = Result::UNCHANGED;
}

// void FeasibleDomain::overwrite(const FeasibleDomain &domain, EncodingMark mark) {
//     assert(d_width == domain.d_width);
//     // Always reset the pending state when we replace the stored domain.
//     // Otherwise we can end up carrying stale UPDATED/CHANGED flags (or no
//     // update at all) across history restores, which breaks later propagation
//     // decisions that rely on consume_state().
//     d_pending_state = Result::UNCHANGED;

//     if (mark == EncodingMark::BITS || mark == EncodingMark::BOTH) {
//         d_fixed_count = domain.d_fixed_count;
//         std::copy_n(domain.d_fixed.get(), d_width, d_fixed.get());
//         std::copy_n(domain.d_value.get(), d_width, d_value.get());
//     }

//     if (mark == EncodingMark::INTERVAL || mark == EncodingMark::BOTH) {
//         d_interval_max = domain.d_interval_max;
//         d_interval_min = domain.d_interval_min;
//         d_complementary = domain.d_complementary;
//         d_last_interval_size = domain.d_last_interval_size;
//     }
//     if (mark != EncodingMark::NONE) {
//         if (mark == EncodingMark::BITS || mark == EncodingMark::BOTH) {
//             d_pending_state |= tighten_interval_by_fixed_bits();
//         }
//         if (mark == EncodingMark::INTERVAL || mark == EncodingMark::BOTH) {
//             d_pending_state |= tighten_fixed_bits_by_interval();
//         }

//         sync_interval_if_totally_fixed();

//         // Even if the tighten helpers reported UNCHANGED, overwriting the
//         // domain should be treated as an update so that parents get
//         // re-enqueued when consume_state() is called.
//         if (is_unchanged(d_pending_state)) {
//             d_pending_state = Result::UPDATED;
//         }
//     }
// }

Result FeasibleDomain::overwrite(const FeasibleDomain &domain, EncodingMark mark) {
    assert(d_width == domain.d_width);
    assert(mark != EncodingMark::NONE);
    // Always reset the pending state when we replace the stored domain.
    // Otherwise we can end up carrying stale UPDATED/CHANGED flags (or no
    // update at all) across history restores, which breaks later propagation
    // decisions that rely on consume_state().
    d_pending_state = Result::UPDATED;
    Result ret = Result::UNCHANGED;
    if (mark == EncodingMark::BOTH) {
        d_fixed_count = domain.d_fixed_count;
        std::copy_n(domain.d_fixed.get(), d_width, d_fixed.get());
        std::copy_n(domain.d_value.get(), d_width, d_value.get());
        d_interval_max = domain.d_interval_max;
        d_interval_min = domain.d_interval_min;
        d_complementary = domain.d_complementary;
        d_last_interval_size = domain.d_last_interval_size;
        sync_interval_if_totally_fixed();
    }
    else if (mark == EncodingMark::BITS) {
        d_fixed_count = domain.d_fixed_count;
        std::copy_n(domain.d_fixed.get(), d_width, d_fixed.get());
        std::copy_n(domain.d_value.get(), d_width, d_value.get());
        // If all bits are fixed, the fixed-bit encoding uniquely defines the
        // value. Syncing the interval first avoids temporarily inconsistent
        // states (notably for 1-bit domains) that can trigger spurious
        // conflicts during interval tightening.
        sync_interval_if_totally_fixed();
        if (!is_totally_fixed()) {
            ret |= tighten_interval_by_fixed_bits();
        }
    }
    else if (mark == EncodingMark::INTERVAL) {
        d_interval_max = domain.d_interval_max;
        d_interval_min = domain.d_interval_min;
        d_complementary = domain.d_complementary;
        d_last_interval_size = domain.d_last_interval_size;
        ret |= tighten_fixed_bits_by_interval();
    }
    if (is_conflict(ret)) {
        d_pending_state = Result::CONFLICT;
        return Result::CONFLICT;
    }
    d_pending_state |= ret;
    return d_pending_state;
}

Result FeasibleDomain::merge(const FeasibleDomain &domain) {
    assert(d_width == domain.d_width);

    Result ret = Result::UNCHANGED;
    for (uint32_t i = 0; i < d_width; ++i) {
        if (!domain.is_fixed(i)) {
            continue;
        }
        ret |= set_fixed(i, domain.get_value(i));
        if (is_conflict(ret)) {
            return Result::CONFLICT;
        }
    }

    ret |= apply_interval_constraint(
        domain.interval_min(), domain.interval_max(), domain.is_interval_complementary());
    if (is_conflict(ret)) {
        return Result::CONFLICT;
    }
    return ret;
}

Result
FeasibleDomain::consume_state() {
    Result state = d_pending_state;
    d_pending_state = Result::UNCHANGED;
    return state;
}

void FeasibleDomain::mark_pending_state(Result state) {
    if (!is_changed(state)) {
        return;
    }
    if (d_pending_state == Result::UPDATED || state == Result::UPDATED) {
        d_pending_state = Result::UPDATED;
    }
    else {
        d_pending_state = Result::CHANGED;
    }
}

Result FeasibleDomain::set_constant(bool value) {
    assert(is_one_bit());
    assert(!d_complementary);

    Result res = Result::UNCHANGED;

    res |= set_fixed(0, value);
    if (is_conflict(res)) {
        return res;
    }

    BitVector bv_value = value ? BitVector::mk_one(d_width)
                               : BitVector::mk_zero(d_width);

    res |= apply_interval_constraint(bv_value, bv_value, false);
    return res;
}

Result FeasibleDomain::set_constant(const BitVector &value) {
    assert(value.size() == d_width);
    Result res = Result::UNCHANGED;
    for (uint32_t i = 0; i < d_width; ++i) {
        const bool bit = value.bit(i);
        res |= set_fixed(i, bit);
        if (is_conflict(res)) {
            return Result::CONFLICT;
        }
    }

    res |= apply_interval_constraint(value, value, false);
    return res;
}

Result
FeasibleDomain::set_fixed(uint32_t index, bool value) {
    assert(index < d_width);

    if (d_fixed[index]) {
        if (static_cast<bool>(d_value[index]) == value) {
            return Result::UNCHANGED;
        }
        return Result::CONFLICT;
    }

    d_fixed[index] = true;
    d_value[index] = value;
    ++d_fixed_count;
    sync_interval_if_totally_fixed();
    mark_pending_state(Result::UPDATED);
    return Result::UPDATED;
}

Result
FeasibleDomain::judge_valid() const {
    if (!is_totally_fixed()) return Result::UNCHANGED;

    if (!in_interval(*this, build_value_from_fixed_bits())) {
        return Result::CONFLICT;
    }
    return Result::UNCHANGED;
}

bool FeasibleDomain::is_interval_constant() const {
    if (d_complementary) {
        return false;
    }
    return d_interval_min.compare(d_interval_max) == 0;
}

bool FeasibleDomain::is_interval_complete() const {
    if (is_one_bit()) {
        return !is_totally_fixed();
    }

    if (!d_complementary) {
        return d_interval_min.is_zero() && d_interval_max.is_ones();
    }
    // We want to avoid considering intervals like [l, l+1] as complete
    assert(d_interval_min.bvinc().compare(d_interval_max) != 0);
    return false;
}

// Returns the size of the feasible interval.
BitVector FeasibleDomain::calc_interval_size() const {
    return compute_interval_size(d_interval_min, d_interval_max, d_complementary);
}

// Returns the size of the feasible domain by dynamic programming.
BitVector FeasibleDomain::calc_domain_size() const {
    if (is_full())
        return BitVector::mk_ones(d_width);
    if (is_totally_fixed())
        return BitVector::mk_zero(d_width);
    if (d_fixed_count == 0) {
        return calc_interval_size();
    }

    auto calc_prefix = [this](const BitVector &limit) -> BitVector {
        std::array<BitVector, 2> dp = {BitVector::mk_zero(d_width),
                                       BitVector::mk_one(d_width)};

        for (int32_t pos = static_cast<int32_t>(d_width) - 1; pos >= 0; --pos) {
            std::array<BitVector, 2> next = {BitVector::mk_zero(d_width),
                                             BitVector::mk_zero(d_width)};
            const bool is_fixed_bit = d_fixed[pos];
            const bool fixed_value = is_fixed_bit ? d_value[pos] : false;
            const bool limit_bit = limit.bit(pos);
            for (unsigned tight = 0; tight < 2; ++tight) {
                const BitVector &ways = dp[tight];
                if (ways.is_zero()) {
                    continue;
                }

                auto try_bit = [&](bool bit_value) {
                    if (is_fixed_bit && bit_value != fixed_value) {
                        return;
                    }
                    if (tight && bit_value > limit_bit) {
                        return;
                    }
                    const unsigned next_tight =
                        tight && (bit_value == limit_bit) ? 1 : 0;
                    BitVector &dest = next[next_tight];
                    dest = dest.bvadd(ways);
                };

                if (is_fixed_bit) {
                    try_bit(fixed_value);
                }
                else {
                    try_bit(false);
                    try_bit(true);
                }
            }
            dp = std::move(next);
        }
        return dp[0].bvadd(dp[1]);
    };

    auto calc_range = [&](const BitVector &lower,
                          const BitVector &upper) -> BitVector {
        BitVector ret = calc_prefix(upper);
        if (lower.is_zero())
            return ret;
        return ret.bvsub(calc_prefix(lower.bvdec()));
    };

    if (!d_complementary)
        return calc_range(d_interval_min, d_interval_max);

    return calc_prefix(d_interval_min).bvadd(calc_range(d_interval_max, BitVector::mk_ones(d_width)));
}

bool FeasibleDomain::try_set_update_by_interval() {
    if (is_one_bit())
        return false;
    BitVector current_size = calc_interval_size();
    if (current_size.compare(d_last_interval_size) >= 0) {
        return false;
    }

    BitVector shrink = d_last_interval_size.bvsub(current_size);
    if (shrink.compare(d_last_interval_size.bvshr(d_interval_update_thres)) > 0) {
        stage_last_interval_size(std::move(current_size));
        // mark_pending_state(Result::UPDATED);
        return true;
    }
    return false;
}

void FeasibleDomain::stage_last_interval_size() {
    d_last_interval_size = calc_interval_size();
}

void FeasibleDomain::stage_last_interval_size(BitVector &&size) {
    d_last_interval_size = std::move(size);
}

Result
FeasibleDomain::intersect_interval_constraint(const BitVector &min,
                                              const BitVector &max,
                                              bool complementary) {
    // Assertions
    assert(min.size() == d_width);
    assert(max.size() == d_width);

#if 0
    std::cout << "In intersect_interval_constraint: " << std::endl;
    std::cout << "  New interval:     "
              << (complementary ? "~" : " ")
              << "[" << min << ", " << max << "]"
              << std::endl;
    std::cout << "  Current interval: "
              << (d_complementary ? "~" : " ")
              << "[" << d_interval_min << ", " << d_interval_max << "]"
              << std::endl;
    std::cout << *this << std::endl;
#endif

    assert(min.compare(max) <= 0);

    // if (is_interval_constant() || is_full_interval(min, max, complementary))
    //     return Result::UNCHANGED;

    if (is_full_interval(min, max, complementary))
        return Result::UNCHANGED;

    if (is_interval_constant()) {
        const BitVector &value = d_interval_min;
        if (!complementary) {
            if (value.compare(min) < 0 || value.compare(max) > 0) {
                return Result::CONFLICT;
            }
        }
        else {
            if (value.compare(min) > 0 && value.compare(max) < 0) {
                return Result::CONFLICT;
            }
        }
        return Result::UNCHANGED;
    }

    if (is_one_bit()) {
        assert(min.compare(max) == 0);
        assert(min.size() == 1);
        if (min.bit(0) == 0) {
            return set_fixed(0, 0);
        }
        return set_fixed(0, 1);
    }

    if (is_interval_complete()) {
        d_interval_min = min;
        d_interval_max = max;
        d_complementary = complementary;
        return Result::CHANGED;
    }

    int count = intersects_count(min, max, complementary, *this);
    if (count == 0) {  // no intersection
        return Result::CONFLICT;
    }

    if (count == 2) {
        // // it split into two parts, we keep current domain for consistency
        return Result::UNCHANGED;

        // // it split into two parts, we keep current domain for consistency
        // // return Result::UNCHANGED;

        // if (d_complementary && !complementary) {
        //     // d_complementary == true
        //     // complementary == false
        //     d_interval_min = min;
        //     d_interval_max = max;
        //     d_complementary = complementary;
        //     d_last_interval_size = BitVector::mk_ones(d_width);
        //     return Result::CHANGED;
        // }
        // return Result::UNCHANGED;

        BitVector size_a = feasible_domain_size(min, max, complementary);
        BitVector size_b = feasible_domain_size(d_interval_min, d_interval_max, d_complementary);

        // int32_t cmp = size_a.compare(size_b);
        // if (cmp > 0 || (cmp == 0 && !d_complementary))
        //     return Result::UNCHANGED;

        // d_interval_min = min;
        // d_interval_max = max;
        // d_complementary = complementary;
        // return Result::CHANGED;
    }

    // now we only have one intersection part
    Result result = Result::UNCHANGED;

    // both normal interval
    if (!complementary && !d_complementary) {
        if (min.compare(d_interval_min) > 0) {
            d_interval_min = min;
            result |= Result::CHANGED;
        }
        if (max.compare(d_interval_max) < 0) {
            d_interval_max = max;
            result |= Result::CHANGED;
        }
        return result;
    }

    // both complementary interval
    if (complementary && d_complementary) {
        if (min.compare(d_interval_min) < 0) {
            d_interval_min = min;
            result |= Result::CHANGED;
        }
        if (max.compare(d_interval_max) > 0) {
            d_interval_max = max;
            result |= Result::CHANGED;
        }
        return result;
    }

    // [d_min, d_max] and [max, 1111] U [0000, min]
    if (complementary && !d_complementary) {
        if (d_interval_min.compare(max) >= 0 ||
            d_interval_max.compare(min) <= 0) {
            // d_interval is more restrictive
            return Result::UNCHANGED;
        }
        else if (d_interval_max.compare(max) >= 0) {
            // This make sure only one part is intersected
            assert(d_interval_min.compare(min) > 0);
            assert(d_interval_min.compare(max) < 0);

            // [max, d_interval_max]
            d_interval_min = max;
            result |= Result::CHANGED;
        }
        else if (d_interval_min.compare(min) <= 0) {
            // This make sure only one part is intersected
            assert(d_interval_max.compare(min) > 0);
            assert(d_interval_max.compare(max) < 0);
            // [d_interval_min, min]
            d_interval_max = min;
            result |= Result::CHANGED;
        }
        else
            throw std::runtime_error("unreachable code reached");

        return result;
    }

    if (!complementary && d_complementary) {
        // [min, max] and [d_max, 1111] U [0000, d_min]
        if (max.compare(d_interval_min) <= 0 ||
            min.compare(d_interval_max) >= 0) {
            // [min, max] is more restrictive
            d_interval_min = min;
            d_interval_max = max;
            d_complementary = false;
            result |= Result::CHANGED;
        }
        else if (max.compare(d_interval_max) >= 0) {
            // This make sure only one part is intersected
            assert(min.compare(d_interval_max) < 0);
            assert(min.compare(d_interval_min) > 0);
            // [d_interval_max, max]
            d_interval_min = d_interval_max;
            d_interval_max = max;
            d_complementary = false;
            result |= Result::CHANGED;
        }
        else if (min.compare(d_interval_min) <= 0) {
            // This make sure only one part is intersected
            assert(max.compare(d_interval_min) > 0);
            assert(max.compare(d_interval_max) < 0);
            // [min, d_interval_min]
            d_interval_max = d_interval_min;
            d_interval_min = min;
            d_complementary = false;
            result |= Result::CHANGED;
        }
        else
            throw std::runtime_error("unreachable code reached");

        return result;
    }

    throw std::runtime_error("unreachable code reached");
    return Result::UNCHANGED;
}

Result
FeasibleDomain::apply_interval_constraint(const BitVector &min,
                                          const BitVector &max,
                                          bool complementary) {
    assert(min.compare(max) <= 0);
    Result ret = intersect_interval_constraint(min, max, complementary);

    if (is_conflict(ret) || is_unchanged(ret))
        return ret;
    assert(is_changed(ret));
    if (try_set_update_by_interval()) {
        mark_pending_state(Result::UPDATED);
        ret = Result::UPDATED;
    }
    else {
        mark_pending_state(Result::CHANGED);
    }
    return ret;
}

Result
FeasibleDomain::tighten_fixed_bits_by_interval() {
    // complementary interval has 0000 and 1111,
    // no fixed bits information can be used to tighten
    if (is_totally_fixed() || is_interval_complementary() || is_one_bit()) {
        return Result::UNCHANGED;
    }

    Result result = Result::UNCHANGED;
    if (!d_complementary && is_interval_constant()) {
        for (uint32_t i = 0; i < d_width; ++i) {
            const bool bit = d_interval_min.bit(i);
            result |= set_fixed(i, bit);
            if (result == Result::CONFLICT) {
                return Result::CONFLICT;
            }
        }
        return result;
    }

    // use fixed bits to get lower bound
    BitVector tmp_min = make_unsigned_min(*this);

    for (int i = d_width - 1; i >= 0; --i) {
        if (d_fixed[i]) {
            assert(d_value[i] == tmp_min.bit(i));
            continue;
        }

        // try fix to 1
        assert(!tmp_min.bit(i));
        tmp_min.set_bit(i, true);
        if (tmp_min.compare(d_interval_max) > 0) {
            // must be 0
            result |= set_fixed(i, false);
            tmp_min.set_bit(i, false);
        }
        else {
            // can be 0 or 1
            tmp_min.set_bit(i, false);
            break;
        }
    }

    // use fixed bits to get upper bound
    BitVector tmp_max = make_unsigned_max(*this);

    for (int i = d_width - 1; i >= 0; --i) {
        if (d_fixed[i]) {
            assert(d_value[i] == tmp_max.bit(i));
            continue;
        }

        // try fix to 0
        assert(tmp_max.bit(i));
        tmp_max.set_bit(i, false);
        if (tmp_max.compare(d_interval_min) < 0) {
            // must be 1
            result |= set_fixed(i, true);
            tmp_max.set_bit(i, true);
        }
        else {
            // can be 0 or 1
            tmp_max.set_bit(i, true);
            break;
        }
    }

    return result;
}

Result
FeasibleDomain::tighten_interval_by_fixed_bits() {
    if (is_totally_fixed()) {
        BitVector value = get_fixed_value();
        if (!in_interval(*this, value)) {
            return Result::CONFLICT;
        }

        if (d_interval_min != d_interval_max) {
            return apply_interval_constraint(value, value, false);
        }
        else {
            assert(d_interval_min == get_fixed_value());
            return Result::UNCHANGED;
        }
    }

    if (is_one_bit()) {
        return Result::UNCHANGED;
    }

    // BitVector unsigned_min = make_unsigned_min(*this);
    // BitVector unsigned_max = make_unsigned_max(*this);

    // if (intersects(unsigned_min, unsigned_max, false, *this) == 0) {
    //     return Result::CONFLICT;
    // }
    // if (d_complementary) {
    //     // if it will be only one part after tightening
    //     bool max_overflow = d_interval_max.compare(unsigned_max) > 0;
    //     bool min_overflow = d_interval_min.compare(unsigned_min) < 0;

    //     assert(!(min_overflow && max_overflow));

    //     if (min_overflow && !max_overflow) {
    //         // make normal interval [d_interval_max, unsigned_max]
    //         assert(d_interval_min.compare(unsigned_max) <= 0);
    //         result |= apply_interval_constraint(BitVector(d_interval_max), unsigned_max, false);
    //     }
    //     else if (!min_overflow && max_overflow) {
    //         // make normal interval [unsigned_min, d_interval_min]
    //         assert(unsigned_min.compare(d_interval_min) <= 0);
    //         result |= apply_interval_constraint(unsigned_min, BitVector(d_interval_min), false);
    //     }
    // }
    // assert(!is_conflict(result));
    Result result = Result::UNCHANGED;
    // std::cout << "min: " << d_interval_min << " max: " << d_interval_max << " complementary: " << d_complementary << std::endl;
    bool min_overflow = false;
    bool max_overflow = false;
    if (!d_complementary) {
        // Normal Interval: [d_interval_min, d_interval_max]
        // Find smallest value >= d_interval_min
        BitVector new_min = find_min_consistent(d_interval_min, min_overflow);
        BitVector new_max = find_max_consistent(d_interval_max, max_overflow);
        // std::cout << "new_min: " << new_min << " new_max: " << new_max << std::endl;
        // std::cout << "min_overflow: " << min_overflow << " max_overflow: " << max_overflow << std::endl;
        if (!min_overflow) {
            if (!max_overflow) {
                // !min_overflow and !max_overflow
                if (new_min.compare(new_max) > 0) {
                    // fixed bits -1-
                    // min 100 -> 110
                    // max 101 -> 011
                    return Result::CONFLICT;
                }
                result |= apply_interval_constraint(new_min, new_max, false);
            }
            else {
                // !min_overflow and max_overflow
                // fixed bits 1--
                // min 000 -> 100
                // max 001 -> 111
                return Result::CONFLICT;
            }
        }
        else {
            if (!max_overflow) {
                // min_overflow and !max_overflow
                // fixed bits 0--
                // min 110 -> 000
                // max 111 -> 011
                return Result::CONFLICT;
            }
            else {
                // min_overflow and max_overflow
                // not exist
                throw std::runtime_error("unreachable code reached");
                // return Result::CONFLICT;
            }
        }
    }
    else {
        BitVector new_min = find_max_consistent(d_interval_min, min_overflow);
        BitVector new_max = find_min_consistent(d_interval_max, max_overflow);
        // std::cout << "new_min: " << new_min << " new_max: " << new_max << std::endl;
        // std::cout << "min_overflow: " << min_overflow << " max_overflow: " << max_overflow << std::endl;
        if (!min_overflow) {
            if (!max_overflow) {
                // !min_overflow and !max_overflow
                if (new_min.compare(new_max) > 0) {
                    // not exist
                    throw std::runtime_error("unreachable code reached");
                }
                // fixed bits -1-
                // min 100 -> 011
                // max 110 -> 110    
                result |= apply_interval_constraint(new_min, new_max, true);
            }
            else {
                // !min_overflow and max_overflow
                // fixed bits 0--
                // min 101 -> 011
                // max 111 -> 011
                if (new_min.compare(new_max) < 0) {
                    // not exist
                    throw std::runtime_error("unreachable code reached");
                }
                result |= apply_interval_constraint(new_max, new_min, false);
            }
        }
        else {
            if (!max_overflow) {
                // min_overflow and !max_overflow
                // fixed bits 1--
                // min 001 -> 111
                // max 011 -> 110
                if (new_min.compare(new_max) < 0) {
                    // not exist
                    throw std::runtime_error("unreachable code reached");
                }
                result |= apply_interval_constraint(new_max, new_min, false);
            }
            else {
                // min_overflow and max_overflow
                // fixed bits 10-
                // min 001 -> 101
                // max 110 -> 100
                return Result::CONFLICT;
            }
        }
    }
    return result;
}

// Use the pattern fix_bits [1*00**1] to find the minimum consistent value >= limit
BitVector
FeasibleDomain::find_min_consistent(const BitVector &limit, bool &overflow) const {
    overflow = false;
    BitVector flex = BitVector::mk_zero(d_width);
    bool tight = true;
    bool all_ones = true;  // can we increment?
    uint32_t flex_cnt = 0;

    for (int i = d_width - 1; i >= 0; --i) {
        bool min_bit = limit.bit(i);
        if (d_fixed[i]) {
            bool val = d_value[i];
            if (tight && val != min_bit) {
                tight = false;
                // fixed 1 > min_bit 0, is not tight anymore
                if (val)
                    continue;

                // fixed 0 < min_bit 1
                if (all_ones) {
                    overflow = true;
                    // throw std::runtime_error("unreachable code reached");
                }
                flex.ibvinc();
            }
        }
        else {
            flex.ibvshl(1);
            ++flex_cnt;
            if (!tight || !min_bit) {
                // flex.set_bit(0, false);
                all_ones = false;
            }
            else {
                flex.set_bit(0, true);
            }
        }
    }

    // Reconstruct
    BitVector ret = BitVector::mk_zero(d_width);
    uint32_t idx = flex_cnt;
    for (int i = d_width - 1; i >= 0; --i) {
        if (d_fixed[i])
            ret.set_bit(i, d_value[i]);
        else
            ret.set_bit(i, flex.bit(--idx));
    }
    assert(idx == 0);
    return ret;
}

BitVector
FeasibleDomain::find_max_consistent(const BitVector &limit, bool &overflow) const {
    overflow = false;
    BitVector flex = BitVector::mk_zero(d_width);
    bool tight = true;
    bool all_zeros = true;  // can we decrement?
    uint32_t flex_cnt = 0;

    for (int i = d_width - 1; i >= 0; --i) {
        bool max_bit = limit.bit(i);
        if (d_fixed[i]) {
            bool val = d_value[i];
            if (tight && val != max_bit) {
                tight = false;
                if (!val)  // fixed 0 < max_bit 1, is not tight anymore
                    continue;

                if (all_zeros) {
                    overflow = true;
                    // throw std::runtime_error("unreachable code reached");
                }
                flex.ibvdec();
            }
        }
        else {
            flex.ibvshl(1);
            ++flex_cnt;
            if (!tight || max_bit) {
                flex.set_bit(0, true);
                all_zeros = false;
            }
            else {  // tight && max_bit is 0
                // flex.set_bit(0, false);
            }
        }
    }

    // Reconstruct
    BitVector ret = BitVector::mk_zero(d_width);
    uint32_t idx = flex_cnt;
    for (int i = d_width - 1; i >= 0; --i) {
        if (d_fixed[i])
            ret.set_bit(i, d_value[i]);
        else
            ret.set_bit(i, flex.bit(--idx));
    }
    assert(idx == 0);
    return ret;
}

std::string
FeasibleDomain::to_string(bool verbose) const {
    std::ostringstream ss;
    ss << "width=" << d_width << " fixed=" << d_fixed_count << "/" << d_width;
    ss << " bits=[";
    for (int32_t i = static_cast<int32_t>(d_width) - 1; i >= 0; --i) {
        if (d_fixed[i]) {
            ss << (d_value[i] ? '1' : '0');
        }
        else {
            ss << '?';
        }
    }
    ss << "]";
    ss << " interval=" << (d_complementary ? "~" : "") << "["
       << d_interval_min.str(2) << "," << d_interval_max.str(2) << "]";
    if (verbose) {
        ss << " last_interval=" << d_last_interval_size.str(2);
        ss << " pending=" << state_to_cstr(d_pending_state);
        ss << " full=" << (is_full() ? "1" : "0");
    }
    return ss.str();
}

bool FeasibleDomain::check_invariants(std::ostream *out) const {
    bool ok = true;
    auto fail = [&](const std::string &msg) {
        ok = false;
        if (out) {
            (*out) << msg << '\n';
        }
    };

    if (d_width == 0) {
        fail("width must be > 0");
    }
    if (d_fixed_count > d_width) {
        fail("fixed_count exceeds width");
    }

    uint32_t counted = 0;
    for (uint32_t i = 0; i < d_width; ++i) {
        if (d_fixed[i]) {
            ++counted;
        }
    }
    if (counted != d_fixed_count) {
        std::ostringstream ss;
        ss << "fixed_count mismatch stored=" << d_fixed_count
           << " actual=" << counted;
        fail(ss.str());
    }

    const uint64_t interval_width = d_interval_min.size();
    if (interval_width != d_interval_max.size() || interval_width != d_width) {
        std::ostringstream ss;
        ss << "interval width mismatch min=" << d_interval_min.size()
           << " max=" << d_interval_max.size()
           << " domain=" << d_width;
        fail(ss.str());
    }
    if (d_interval_min.compare(d_interval_max) > 0) {
        fail("interval_min > interval_max");
    }

    if (d_last_interval_size.size() != d_width) {
        std::ostringstream ss;
        ss << "last_interval_size width mismatch size="
           << d_last_interval_size.size() << " domain=" << d_width;
        fail(ss.str());
    }

    if (is_totally_fixed()) {
        BitVector value = build_value_from_fixed_bits();
        if (d_complementary || d_interval_min.compare(value) != 0 ||
            d_interval_max.compare(value) != 0) {
            fail("totally fixed domain interval mismatch");
        }
    }

    return ok;
}

}  // namespace bzla::preprocess::pass::fdp
