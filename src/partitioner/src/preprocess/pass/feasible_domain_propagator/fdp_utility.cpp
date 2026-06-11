#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"

namespace bzla::preprocess::pass::fdp {

namespace {

// constexpr uint32_t kDomainEncodeThres = 3; // n means domain at least reduced to 1 / n of full size

constexpr uint32_t SYMBOL_BITS_ENCODE_THRES = 1;  // n means at least fix n bits
constexpr const char *SYMBOL_INTERVAL_ENCODE_THRES = "3";  // n means interval at least reduced to 1 / n
constexpr uint32_t INTERIOR_BITS_ENCODE_THRES = 2;
constexpr const char *INTERIOR_INTERVAL_ENCODE_THRES = "6";

BitVector make_threshold_bitvector(uint32_t width, const char *threshold) {
    if (threshold == nullptr || *threshold == '\0') {
        return BitVector::mk_ones(width);
    }

    errno = 0;
    char *endptr = nullptr;
    unsigned long long parsed = std::strtoull(threshold, &endptr, 10);
    if (errno == ERANGE || endptr == nullptr || endptr == threshold || *endptr != '\0') {
        // If parsing fails or overflows, clamp to max representable value.
        return BitVector::mk_ones(width);
    }

    if (width < 64) {
        const unsigned long long max_value =
            width == 0 ? 0ull : 1ull << width;
        if (parsed >= max_value) {
            return BitVector::mk_ones(width);
        }
    }
    // For width >= 64, any parsed 64-bit value fits.
    return BitVector::from_ui(width, static_cast<uint64_t>(parsed));
}

// Normalize a possibly complementary interval into a continuous interval on an
// extended bit-width. Complementary intervals [0, left] U [right, max] are
// mapped to [right, max + left + 1].
void normalize_interval_to_extended(const FeasibleDomain &domain,
                                    uint32_t extended_width,
                                    BitVector &out_min,
                                    BitVector &out_max) {
    const uint32_t width = domain.width();
    assert(extended_width > width);
    const uint32_t padding = extended_width - width;

    if (domain.is_interval_complementary()) {
        out_min = domain.interval_max().bvzext(padding);
        BitVector upper = BitVector::mk_one(extended_width).bvshl(width);
        upper.ibvadd(domain.interval_min().bvzext(padding));
        out_max = upper;
    }
    else {
        out_min = domain.interval_min().bvzext(padding);
        out_max = domain.interval_max().bvzext(padding);
    }
}

// Apply an interval computed on an extended bit-width back onto the original
// width, taking the result modulo 2^width. When the interval crosses the
// modulus boundary, we encode it as a complementary interval.
Result apply_interval(const BitVector &ext_min,
                      const BitVector &ext_max,
                      uint32_t width,
                      FeasibleDomain &output) {
    assert(ext_min.size() == ext_max.size());
    assert(ext_min.size() >= width);

    if (ext_max.bvsub(ext_min).compare(BitVector::mk_ones(width).bvzext(ext_min.size() - width)) >= 0) {
        // The interval covers the entire range, so we can skip tightening.
        return Result::UNCHANGED;
    }

    BitVector min_mod = ext_min.bvextract(width - 1, 0);
    BitVector max_mod = ext_max.bvextract(width - 1, 0);

    if (max_mod.compare(min_mod) < 0) {
        // The interval crosses the modulus boundary, so we encode it as a complementary interval.
        return output.apply_interval_constraint(max_mod, min_mod, true);
    }
    return output.apply_interval_constraint(min_mod, max_mod, false);
}

}  // namespace

bool is_interval_complete(const BitVector &min,
                          const BitVector &max,
                          bool complementary) {
    if (complementary) {
        if (min.bvinc().compare(max) == 0) {
            return true;
        }
        return false;
    }
    return min.is_zero() && max.is_ones();
}

IntervalTerm::IntervalTerm(const BitVector &min,
                           const BitVector &max,
                           bool complementary)
    : d_min(min), d_max(max), d_complementary(complementary) {
}

IntervalTerm::IntervalTerm(const FeasibleDomain &domain)
    : d_min(domain.interval_min()),
      d_max(domain.interval_max()),
      d_complementary(domain.is_interval_complementary()) {
}

bool intersects(const FeasibleDomain &a, const FeasibleDomain &b) {
    return intersects(a.interval_min(),
                      a.interval_max(),
                      a.is_interval_complementary(),
                      b.interval_min(),
                      b.interval_max(),
                      b.is_interval_complementary());
}

bool intersects(const BitVector &min_a,
                const BitVector &max_a,
                bool comp_a,
                const FeasibleDomain &b) {
    return intersects(min_a,
                      max_a,
                      comp_a,
                      b.interval_min(),
                      b.interval_max(),
                      b.is_interval_complementary());
}

bool intersects(const BitVector &min_a,
                const BitVector &max_a,
                bool comp_a,
                const BitVector &min_b,
                const BitVector &max_b,
                bool comp_b) {
    if (comp_a && comp_b) {
        // both have 1111 and 0000
        return true;
    }
    else if (comp_a) {
        // [0, min_a] U [max_a, 1111]
        return min_b.compare(min_a) <= 0 ||
               max_a.compare(max_b) <= 0;
    }
    else if (comp_b) {
        return min_a.compare(min_b) <= 0 ||
               max_b.compare(max_a) <= 0;
    }
    return min_a.compare(max_b) <= 0 && min_b.compare(max_a) <= 0;
}

uint32_t intersects_count(const FeasibleDomain &a, const FeasibleDomain &b) {
    return intersects_count(a.interval_min(),
                            a.interval_max(),
                            a.is_interval_complementary(),
                            b.interval_min(),
                            b.interval_max(),
                            b.is_interval_complementary());
}
uint32_t intersects_count(const BitVector &min_a,
                          const BitVector &max_a,
                          bool comp_a,
                          const FeasibleDomain &b) {
    return intersects_count(min_a,
                            max_a,
                            comp_a,
                            b.interval_min(),
                            b.interval_max(),
                            b.is_interval_complementary());
}
uint32_t intersects_count(const BitVector &min_a,
                          const BitVector &max_a,
                          bool comp_a,
                          const BitVector &min_b,
                          const BitVector &max_b,
                          bool comp_b) {
    if (comp_a && !comp_b)
        return intersects_count(min_b, max_b, comp_b, min_a, max_a, comp_a);
    // now we only need to consider one case: comp_a <= comp_b

    if (!intersects(min_a, max_a, comp_a, min_b, max_b, comp_b)) {
        return 0;
    }

    if (!comp_a && !comp_b)
        return 1;

    if (comp_a && comp_b) {
        const BitVector &lower =
            (min_a.compare(min_b) > 0) ? min_a : min_b;  // max(min_a, min_b)
        const BitVector &upper =
            (max_a.compare(max_b) < 0) ? max_a : max_b;  // min(max_a, max_b)
        return lower.compare(upper) >= 0 ? 2 : 1;
    }

    if (!comp_a && comp_b) {
        return (min_a.compare(min_b) <= 0 && max_b.compare(max_a) <= 0) ? 2 : 1;
    }

    throw std::runtime_error("Function intersects_count reached unreachable code.");
    return 0;
}

bool is_full_interval(const FeasibleDomain &domain) {
    return domain.is_interval_complete();
}

bool is_full_interval(const BitVector &min, const BitVector &max, bool comp) {
    if (comp) {
        return min.bvinc().compare(max) == 0;
    }
    return min.is_zero() && max.is_ones();
}

Result
align_interval(FeasibleDomain &a, FeasibleDomain &b) {
    Result result = Result::UNCHANGED;

    assert(a.width() == b.width());
    assert(a.interval_min().compare(a.interval_max()) <= 0);
    assert(b.interval_min().compare(b.interval_max()) <= 0);

    if (intersects_count(a, b) == 0) {
        return Result::CONFLICT;
    }

    result |= a.apply_interval_constraint(b.interval_min(),
                                          b.interval_max(),
                                          b.is_interval_complementary());

    if (is_conflict(result))
        return Result::CONFLICT;

    result |= b.apply_interval_constraint(a.interval_min(),
                                          a.interval_max(),
                                          a.is_interval_complementary());

    assert(a.interval_min().compare(b.interval_min()) == 0 || result == Result::CONFLICT);
    assert(a.interval_max().compare(b.interval_max()) == 0 || result == Result::CONFLICT);

    return result;
}

bool in_interval(const FeasibleDomain &domain, const BitVector &value) {
    if (domain.is_interval_complementary()) {
        return value.compare(domain.interval_min()) <= 0 ||
               value.compare(domain.interval_max()) >= 0;
    }

    return value.compare(domain.interval_min()) >= 0 && value.compare(domain.interval_max()) <= 0;
}

// This function return the size of the interval - 1
BitVector
feasible_domain_size(const BitVector &min,
                     const BitVector &max,
                     bool complementary) {
    assert(min.size() == max.size());
    assert(min.compare(max) <= 0);

    if (complementary)
        return min.bvsub(max);
    else
        return max.bvsub(min);
}

int compare_feasible_domain_size(const BitVector &min_a,
                                 const BitVector &max_a,
                                 bool comp_a,
                                 const FeasibleDomain &b) {
    return compare_feasible_domain_size(min_a,
                                        max_a,
                                        comp_a,
                                        b.interval_min(),
                                        b.interval_max(),
                                        b.is_interval_complementary());
}

int compare_feasible_domain_size(const BitVector &min_a,
                                 const BitVector &max_a,
                                 bool comp_a,
                                 const BitVector &min_b,
                                 const BitVector &max_b,
                                 bool comp_b) {
    BitVector size_a = feasible_domain_size(min_a, max_a, comp_a);
    BitVector size_b = feasible_domain_size(min_b, max_b, comp_b);
    return size_a.compare(size_b);
}

Result
update_interval_by_plus(const IntervalTerm &a,
                        const IntervalTerm &b,
                        FeasibleDomain &output) {
    assert(a.d_min.size() == b.d_min.size());

    assert(a.d_min.compare(a.d_max) <= 0);
    assert(b.d_min.compare(b.d_max) <= 0);

    BitVector sum_min = BitVector::mk_zero(a.d_min.size());
    BitVector sum_max = BitVector::mk_zero(a.d_max.size());

    uint32_t min_carry_cnt = 0, max_carry_cnt = 0;

    if (a.d_complementary) {  // [max, min + word_size]
        sum_min.ibvadd(a.d_max);
        sum_max.ibvadd(a.d_min);
        max_carry_cnt++;
    }
    else {  // [min, max]
        sum_min.ibvadd(a.d_min);
        sum_max.ibvadd(a.d_max);
    }

    if (b.d_complementary) {  // [max, min + word_size]
        if (sum_min.is_uadd_overflow(b.d_max))
            min_carry_cnt++;
        if (sum_max.is_uadd_overflow(b.d_min))
            max_carry_cnt++;

        sum_min.ibvadd(b.d_max);
        sum_max.ibvadd(b.d_min);
        max_carry_cnt++;
    }
    else {  // [min, max]
        if (sum_min.is_uadd_overflow(b.d_min))
            min_carry_cnt++;
        if (sum_max.is_uadd_overflow(b.d_max))
            max_carry_cnt++;

        sum_min.ibvadd(b.d_min);
        sum_max.ibvadd(b.d_max);
    }

    if (sum_min.compare(sum_max) > 0 && max_carry_cnt == min_carry_cnt + 1) {  // complementary interval
        // this ensures sum_max < sum_min for complementary interval
        return output.apply_interval_constraint(sum_max, sum_min, true);
    }
    else if (sum_min.compare(sum_max) <= 0 && min_carry_cnt == max_carry_cnt) {  // normal interval
        // this ensures sum_min <= sum_max for normal interval
        return output.apply_interval_constraint(sum_min, sum_max, false);
    }

    return Result::UNCHANGED;
}

Result
update_interval_by_plus(const FeasibleDomain &a,
                        const FeasibleDomain &b,
                        FeasibleDomain &output) {
    return update_interval_by_plus(IntervalTerm(a),
                                   IntervalTerm(b),
                                   output);
}

Result update_interval_by_minus(const FeasibleDomain &a,
                                const FeasibleDomain &b,
                                FeasibleDomain &output) {
    // a - b => a + (-b)
    BitVector neg_b_min = b.interval_min().bvneg();
    BitVector neg_b_max = b.interval_max().bvneg();
    bool neg_b_comp = b.is_interval_complementary();

    if (neg_b_min.compare(neg_b_max) < 0) {
        // Overflow happened during negation
        assert(neg_b_min.is_zero());
        neg_b_comp = !neg_b_comp;
        std::swap(neg_b_min, neg_b_max);
        // now neg_b_min >= neg_b_max
    }

    return update_interval_by_plus(IntervalTerm(a),
                                   IntervalTerm(neg_b_max, neg_b_min, neg_b_comp),
                                   output);
}

Result update_interval_by_multiply(const FeasibleDomain &a,
                                   const FeasibleDomain &b,
                                   FeasibleDomain &output) {
    const uint32_t width = output.width();
    const uint32_t extended_width = width * 2 + 2;

    BitVector a_min, a_max, b_min, b_max;
    normalize_interval_to_extended(a, extended_width, a_min, a_max);
    normalize_interval_to_extended(b, extended_width, b_min, b_max);

    BitVector product_min = a_min.bvmul(b_min), product_max = a_max.bvmul(b_max);

    return apply_interval(product_min, product_max, width, output);
}

Result update_interval_by_division(const FeasibleDomain &dividend,
                                   const FeasibleDomain &divisor,
                                   FeasibleDomain &output) {
    if (feasible_domain_holds_unsigned(divisor, 0)) {
        // Division by zero is still possible; fall back to bit-level reasoning.
        return Result::UNCHANGED;
    }
    if (divisor.is_interval_complementary()) {
        return Result::UNCHANGED;
    }

    const uint32_t width = output.width();
    if (dividend.is_interval_complementary()) {
        BitVector interval0_min = BitVector::mk_zero(width), interval0_max = dividend.interval_min();
        BitVector interval1_min = dividend.interval_max(), interval1_max = BitVector::mk_ones(width);
        BitVector divisor_min = divisor.interval_min(), divisor_max = divisor.interval_max();

        BitVector left_min = interval0_min, left_max = interval0_max.bvudiv(divisor_min);
        BitVector right_min = interval1_min.bvudiv(divisor_max), right_max = interval1_max.bvudiv(divisor_min);

        bool best_comp = false;
        BitVector best_min = left_min, best_max = (left_max.compare(right_max) > 0) ? left_max : right_max;

        // I'm not sure if the following assert is always true, but if it is wrong, we need !best_comp case too.
        assert(best_min.compare(best_max) <= 0);
        // if (best_min.compare(best_max) > 0) {
        //     std::swap(best_min, best_max);
        //     best_comp = !best_comp;
        // }

        if (left_max.compare(right_min) < 0) {
            // Intervals are disjoint; consider complementary coverage between them.
            BitVector comp_min = left_max, comp_max = right_min;
            // left_max < right_min => comp_min < comp_max, comp = true
            BitVector comp_size = feasible_domain_size(comp_min, comp_max, true),
                      best_size = feasible_domain_size(best_min, best_max, false);

            if (comp_size.compare(best_size) < 0) {
                best_min = comp_min, best_max = comp_max;
                best_comp = true;
            }
        }

        return output.apply_interval_constraint(best_min, best_max, best_comp);
    }
    else {
        BitVector quotient_min = dividend.interval_min().bvudiv(divisor.interval_max());
        BitVector quotient_max = dividend.interval_max().bvudiv(divisor.interval_min());

        // smaller / bigger  < bigger / smaller
        assert(quotient_min.compare(quotient_max) <= 0);

        return output.apply_interval_constraint(quotient_min, quotient_max, false);
    }
}

Result
tighten_each_interval(std::vector<FeasibleDomain *> &children,
                      FeasibleDomain *output) {
    Result ret = Result::UNCHANGED;
    for (FeasibleDomain *child : children) {
        ret |= child->tighten_interval_by_fixed_bits();
        if (is_conflict(ret)) {
            return Result::CONFLICT;
        }
    }
    ret |= output->tighten_interval_by_fixed_bits();
    return ret;
}

Result
tighten_each_fixed_bits(std::vector<FeasibleDomain *> &children,
                        FeasibleDomain *output) {
    Result ret = Result::UNCHANGED;
    for (FeasibleDomain *child : children) {
        ret |= child->tighten_fixed_bits_by_interval();
        if (is_conflict(ret)) {
            return Result::CONFLICT;
        }
    }
    ret |= output->tighten_fixed_bits_by_interval();
    return ret;
}

bool try_set_update_by_intervals(std::vector<FeasibleDomain *> &children,
                                 FeasibleDomain *output) {
    bool ret = false;
    auto try_interval = [&](FeasibleDomain *domain) -> bool {
        if (!is_changed(domain->pending_state()))
            return false;
        return domain->try_set_update_by_interval();
    };
    for (FeasibleDomain *child : children)
        ret |= try_interval(child);
    ret |= try_interval(output);
    return ret;
}

bool feasible_domain_holds_value(const FeasibleDomain &domain,
                                 const BitVector &value) {
    assert(value.size() == domain.width());

    const bool in_interval = domain.is_interval_complementary()
                                 ? (value.compare(domain.interval_min()) <= 0 ||
                                    value.compare(domain.interval_max()) >= 0)
                                 : (value.compare(domain.interval_min()) >= 0 &&
                                    value.compare(domain.interval_max()) <= 0);

    if (!in_interval) {
        return false;
    }

    for (uint32_t i = 0, width = domain.width(); i < width; ++i) {
        if (!domain.is_fixed(i)) {
            continue;
        }
        if (domain.get_value(i) != value.bit(i)) {
            return false;
        }
    }

    return true;
}

bool feasible_domain_holds_unsigned(const FeasibleDomain &domain,
                                    uint32_t value) {
    BitVector bv_value = BitVector::from_ui(domain.width(), value);
    return feasible_domain_holds_value(domain, bv_value);
}

BitVector
make_unsigned_max(const FeasibleDomain &domain) {
    uint32_t width = domain.width();
    BitVector max = BitVector::mk_zero(width);

    for (uint32_t i = 0; i < width; ++i) {
        if (!domain.is_fixed_zero(i)) {
            max.set_bit(i, true);
        }
    }
    return max;
}

BitVector
make_unsigned_min(const FeasibleDomain &domain) {
    uint32_t width = domain.width();
    BitVector min = BitVector::mk_zero(width);
    for (uint32_t i = 0; i < width; ++i) {
        if (domain.is_fixed_one(i)) {
            min.set_bit(i, true);
        }
    }
    return min;
}

BitVector
make_signed_max(const FeasibleDomain &domain) {
    uint32_t width = domain.width();
    BitVector max = BitVector::mk_zero(width);

    for (uint32_t i = 0; i < width - 1; ++i) {
        if (!domain.is_fixed_zero(i)) {
            max.set_bit(i, true);
        }
    }

    // sign bit
    if (domain.is_fixed_one(width - 1)) {
        max.set_bit(width - 1, true);
    }

    return max;
}

BitVector
make_signed_min(const FeasibleDomain &domain) {
    uint32_t width = domain.width();
    BitVector min = BitVector::mk_zero(width);

    for (uint32_t i = 0; i < width - 1; ++i) {
        if (domain.is_fixed_one(i)) {
            min.set_bit(i, true);
        }
    }

    // sign bit
    if (!domain.is_fixed_zero(width - 1)) {
        min.set_bit(width - 1, true);
    }

    return min;
}

// totally fixed encoded in bits
EncodingMark calc_encoding_mark(uint32_t width, uint32_t fixed_count, const BitVector &interval_size, const BitVector &domain_size, bool is_symbol) {
    if (width == fixed_count)
        return EncodingMark::BITS;
    const uint32_t bits_encode_thres = is_symbol ? SYMBOL_BITS_ENCODE_THRES : INTERIOR_BITS_ENCODE_THRES;
    const char *interval_encode_thres =
        is_symbol ? SYMBOL_INTERVAL_ENCODE_THRES : INTERIOR_INTERVAL_ENCODE_THRES;
    BitVector reduce_ratio_thres = make_threshold_bitvector(width, interval_encode_thres);
    // BitVector reduce_ratio_thres = BitVector(width, interval_encode_thres, 10);
    if (fixed_count >= bits_encode_thres) {
        BitVector bits_size = BitVector::mk_zero(width);  // feasible bits size
        bits_size.set_bit(width - fixed_count, true);     // 1 << (width - fixed_count)
        if (bits_size.bvudiv(domain_size).compare(reduce_ratio_thres) >= 0) {
            // std::cout << "bits_size: " << bits_size << std::endl;
            // std::cout << "domain_size: " << domain_size << std::endl;
            // std::cout << "bits_size.bvudiv(domain_size): " << bits_size.bvudiv(domain_size) << std::endl;
            // std::cout << "reduce_ratio_thres: " << reduce_ratio_thres << std::endl;
            return EncodingMark::BOTH;
        }
        return EncodingMark::BITS;
    }
    else {
        BitVector all_size = BitVector::mk_ones(width);
        if (all_size.bvudiv(interval_size).compare(reduce_ratio_thres) >= 0) {
            // std::cout << "all_size: " << all_size << std::endl;
            // std::cout << "interval_size: " << interval_size << std::endl;
            // std::cout << "all_size.bvudiv(interval_size): " << all_size.bvudiv(interval_size) << std::endl;
            // std::cout << "reduce_ratio_thres: " << reduce_ratio_thres << std::endl;
            return EncodingMark::INTERVAL;
        }
        return EncodingMark::NONE;
    }
}


/*
    curr domain is looser than prev domain, so:
    curr_fixed_count <= prev_fixed_count
    curr_interval_size >= prev_interval_size
    curr_domain_size >= prev_domain_size
*/
// totally fixed encoded in bits
EncodingMark calc_encoding_mark_diff(uint32_t width,
                                     uint32_t curr_fixed_count,
                                     const BitVector &curr_interval_size,
                                     const BitVector &curr_domain_size,
                                     uint32_t prev_fixed_count,
                                     const BitVector &prev_interval_size,
                                     const BitVector &prev_domain_size) {
    if (width == curr_fixed_count)
        return EncodingMark::IMPLIED;
    if (width == prev_fixed_count)
        return EncodingMark::BITS;
    const uint32_t bits_encode_thres = INTERIOR_BITS_ENCODE_THRES;
    const char *interval_encode_thres = INTERIOR_INTERVAL_ENCODE_THRES;
    BitVector reduce_ratio_thres = make_threshold_bitvector(width, interval_encode_thres);
    // // happens because interval slowly converges
    // if (curr_fixed_count > prev_fixed_count)
    //     return EncodingMark::NONE;


    if (prev_fixed_count > curr_fixed_count + bits_encode_thres) {
        // happens because interval slowly converges
        if (curr_domain_size.compare(prev_domain_size) <= 0)
            return EncodingMark::BITS;

        BitVector bits_size = BitVector::mk_zero(width);                         // feasible bits size
        bits_size.set_bit(width - prev_fixed_count, true);  // 1 << (width - fixed_count)

        BitVector domain_size_delta = curr_domain_size.bvsub(prev_domain_size);
        if (bits_size.bvudiv(domain_size_delta).compare(reduce_ratio_thres) >= 0) {
            // // display bits_size.bvudiv(domain_size_delta)
            // std::cout << "bits_size: " << bits_size << std::endl;
            // std::cout << "domain_size_delta: " << domain_size_delta << std::endl;
            // std::cout << "bits_size.bvudiv(domain_size_delta)" << bits_size.bvudiv(domain_size_delta) << std::endl;
            // std::cout << "reduce_ratio_thres: " << reduce_ratio_thres << std::endl;
            return EncodingMark::BOTH;
        }
        return EncodingMark::BITS;
    }
    else {
        // happens because interval slowly converges
        if (curr_interval_size.compare(prev_interval_size) <= 0)
            return EncodingMark::NONE;

        BitVector interval_size_delta = curr_interval_size.bvsub(prev_interval_size);
        if (curr_domain_size.bvudiv(interval_size_delta).compare(reduce_ratio_thres) >= 0) {
            // // display curr_domain_size.bvudiv(interval_size_delta)
            // std::cout << "curr_domain_size: " << curr_domain_size << std::endl;
            // std::cout << "interval_size_delta: " << interval_size_delta << std::endl;
            // std::cout << "curr_domain_size.bvudiv(interval_size_delta): " << curr_domain_size.bvudiv(interval_size_delta) << std::endl;
            // std::cout << "reduce_ratio_thres: " << reduce_ratio_thres << std::endl;
            return EncodingMark::INTERVAL;
        }
        return EncodingMark::NONE;
    }
}


// /*
//     curr domain is expected to be looser than prev domain, so typically:
//     - curr_fixed_count <= prev_fixed_count
//     - curr_interval_size >= prev_interval_size
// */
// // totally fixed encoded in bits
// EncodingMark calc_encoding_mark_diff(uint32_t width,
//                                      uint32_t curr_fixed_count,
//                                      const BitVector &curr_interval_size,
//                                      uint32_t prev_fixed_count,
//                                      const BitVector &prev_interval_size,
//                                      const BitVector &prev_domain_size) {
//     // Nothing to gain if the current snapshot is already fully fixed.
//     if (width == curr_fixed_count) {
//         return EncodingMark::NONE;
//     }

//     // Decide which components of the previous domain are worth encoding in
//     // absolute terms, then keep only those that tighten the current snapshot.
//     const EncodingMark prev_mark = calc_encoding_mark(width,
//                                                       prev_fixed_count,
//                                                       prev_interval_size,
//                                                       prev_domain_size,
//                                                       false);
//     const bool prev_wants_bits =
//         prev_mark == EncodingMark::BITS || prev_mark == EncodingMark::BOTH;
//     const bool prev_wants_interval =
//         prev_mark == EncodingMark::INTERVAL || prev_mark == EncodingMark::BOTH;

//     EncodingMark mark = EncodingMark::NONE;

//     if (prev_wants_bits && prev_fixed_count > curr_fixed_count) {
//         const uint32_t delta_fixed = prev_fixed_count - curr_fixed_count;
//         if (delta_fixed >= INTERIOR_BITS_ENCODE_THRES) {
//             mark = EncodingMark::BITS;
//         }
//     }

//     if (prev_wants_interval && curr_interval_size.compare(prev_interval_size) > 0) {
//         mark = (mark == EncodingMark::BITS) ? EncodingMark::BOTH : EncodingMark::INTERVAL;
//     }

//     return mark;
// }

FeasibleDomain
create_constant_feasible_domain(uint32_t width, const BitVector &value) {
    FeasibleDomain domain(width);
    for (uint32_t i = 0; i < width; ++i) {
        domain.set_fixed(i, value.bit(i));
    }
    domain.apply_interval_constraint(value, value, false);
    return domain;
}

FeasibleDomain
create_constant_feasible_domain(uint32_t width, uint32_t value) {
    BitVector bv_value = BitVector::from_ui(width, value);
    return create_constant_feasible_domain(width, bv_value);
}

}  // namespace bzla::preprocess::pass::fdp
