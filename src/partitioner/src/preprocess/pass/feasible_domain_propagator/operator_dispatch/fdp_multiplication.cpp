#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_multiplication.h"

#include <algorithm>
#include <cstdint>
#include <vector>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

namespace {

inline uint32_t
minimum_trailing_one(const FeasibleDomain& domain) {
    const uint32_t width = domain.width();
    uint32_t i = 0;
    for (; i < width; ++i) {
        if (!domain.is_fixed_zero(i)) {
            break;
        }
    }
    return i;
}

inline uint32_t
maximum_trailing_one(const FeasibleDomain& domain) {
    const uint32_t width = domain.width();
    uint32_t i = 0;
    for (; i < width; ++i) {
        if (domain.is_fixed_one(i)) {
            break;
        }
    }
    return i;
}

// Set bits in [from, to) to zero in the domain.
Result set_to_zero(FeasibleDomain& domain, uint32_t from, uint32_t to) {
    assert(from <= to);
    assert(to <= domain.width());

    Result result = Result::UNCHANGED;
    for (uint32_t i = from; i < to; ++i) {
        result |= domain.set_fixed(i, false);
        if (is_conflict(result)) {
            return result;
        }
    }
    return result;
}

// Remove from x any trailing "boths", that don't have support in y and output.
Result reason_trailing_bit(FeasibleDomain& x,
                           FeasibleDomain& y,
                           FeasibleDomain& output) {
    Result result = Result::UNCHANGED;
    const uint32_t bitwidth = output.width();
    const uint32_t y_min = minimum_trailing_one(y);
    const uint32_t y_max = maximum_trailing_one(y);
    const uint32_t output_max = maximum_trailing_one(output);
    const uint32_t limit = std::min(y_max, output_max);

    for (uint32_t i = 0; i < bitwidth; ++i) {
        if (x.is_fixed_zero(i)) {
            continue;
        }
        if (x.is_fixed_one(i)) {
            break;
        }

        // #TODO: improve efficiency here.
        for (uint32_t j = y_min; j <= limit; ++j) {
            if (j + i >= bitwidth ||
                (!y.is_fixed_zero(j) && !output.is_fixed_zero(i + j))) {
                return result;
            }
        }

        result |= x.set_fixed(i, false);
        if (is_conflict(result)) {
            return Result::CONFLICT;
        }
    }

    return result;
}

// If x has n trailing zeroes, and y has m trailing zeroes, then the output has n+m trailing zeroes.
// If the output has n trailing zeroes and x has p trailing zeroes, then y has n-p trailing zeroes.
Result fix_trailing_zeroes(FeasibleDomain& x,
                           FeasibleDomain& y,
                           FeasibleDomain& output) {
    Result r0 = reason_trailing_bit(x, y, output);
    if (is_conflict(r0)) {
        return Result::CONFLICT;
    }
    Result r1 = reason_trailing_bit(y, x, output);
    if (is_conflict(r1)) {
        return Result::CONFLICT;
    }

    const uint32_t bitwidth = output.width();
    const uint64_t min_zeroes = static_cast<uint64_t>(minimum_trailing_one(x)) +
                                static_cast<uint64_t>(minimum_trailing_one(y));
    const uint32_t limit = static_cast<uint32_t>(std::min<uint64_t>(bitwidth, min_zeroes));

    Result r2 = set_to_zero(output, 0, limit);
    if (is_conflict(r2)) {
        return Result::CONFLICT;
    }

    if (is_changed(r0) || is_changed(r1) || is_changed(r2)) {
        return Result::CHANGED;
    }

    return Result::UNCHANGED;
}

// Try to fix bits in x and y based on the aspirational sum for the given column.
Result adjust_multiplication(FeasibleDomain& x,
                             FeasibleDomain& y,
                             uint32_t index,
                             int aspirational_sum) {
    // Classify the partial product bits contributing to this column:
    //  - column_ones      : both contributing inputs are fixed to one
    //  - column_one_fixed : exactly one input bit is fixed to one
    //  - column_unfixed   : neither input bit is fixed
    uint32_t column_unfixed = 0;
    uint32_t column_one_fixed = 0;
    uint32_t column_ones = 0;

    for (uint32_t i = 0; i <= index; ++i) {
        const uint32_t xi = index - i;
        const bool x_one = x.is_fixed_one(xi), y_one = y.is_fixed_one(i);

        if (x_one && y_one)
            ++column_ones;
        else if (x_one || y_one)
            ++column_one_fixed;
        else
            ++column_unfixed;
    }

    Result result = Result::UNCHANGED;

    const uint32_t need_all =
        column_ones + column_one_fixed + column_unfixed;

    // We need every value that is unfixed to be set to one.
    if (aspirational_sum == static_cast<int>(need_all) &&
        (column_one_fixed + column_unfixed) > 0) {
        for (uint32_t i = 0; i <= index; ++i) {
            const uint32_t xi = index - i;
            if (!y.is_fixed(i) && !x.is_fixed_zero(xi)) {
                result |= y.set_fixed(i, true);
                if (is_conflict(result)) return result;
            }
            if (!x.is_fixed(xi) && !y.is_fixed_zero(i)) {
                result |= x.set_fixed(xi, true);
                if (is_conflict(result)) return result;
            }
        }
        return result;
    }

    // We have all the ones that we need already. Set everything we can to zero.
    if (aspirational_sum == static_cast<int>(column_ones) &&
        (column_unfixed > 0 || column_one_fixed > 0)) {
        for (uint32_t i = 0; i <= index; ++i) {
            const uint32_t xi = index - i;
            if (!y.is_fixed(i) && x.is_fixed_one(xi)) {
                result |= y.set_fixed(i, false);
                if (is_conflict(result)) return result;
            }
            if (!x.is_fixed(xi) && y.is_fixed_one(i)) {
                result |= x.set_fixed(xi, false);
                if (is_conflict(result)) return result;
            }
        }
    }

    return result;
}

class ColumnCounts {
  public:
    ColumnCounts(std::vector<int>& column_h,
                 std::vector<int>& column_l,
                 std::vector<int>& sum_h,
                 std::vector<int>& sum_l,
                 uint32_t bit_width,
                 FeasibleDomain& output)
        : d_column_h(column_h),
          d_column_l(column_l),
          d_sum_h(sum_h),
          d_sum_l(sum_l),
          d_bit_width(bit_width),
          d_output(output) {
        for (uint32_t i = 0; i < d_bit_width; ++i) {
            d_column_l[i] = 0;
            d_column_h[i] = static_cast<int>(i) + 1;
        }
    }

    void rebuild_sums() {
        d_sum_l[0] = d_column_l[0];
        d_sum_h[0] = d_column_h[0];
        snap_to(0);
        for (uint32_t i = 1; i < d_bit_width; ++i) {
            assert((d_column_h[i] >= d_column_l[i]) && (d_column_l[i] >= 0));
            d_sum_h[i] = d_column_h[i] + (d_sum_h[i - 1] / 2);
            d_sum_l[i] = d_column_l[i] + (d_sum_l[i - 1] / 2);
            snap_to(i);
        }
    }

    Result fixed_point() {
        if (in_conflict()) {
            return Result::CONFLICT;
        }

        bool changed = false;
        for (bool loop = true; loop;) {
            loop = false;
            Result snap = snap_to();
            if (is_conflict(snap)) {
                return Result::CONFLICT;
            }
            if (is_changed(snap)) {
                loop = true;
                changed = true;
            }

            Result prop = propagate();
            if (is_conflict(prop)) {
                return Result::CONFLICT;
            }
            if (is_changed(prop)) {
                loop = true;
                changed = true;
            }
        }

        if (in_conflict()) {
            return Result::CONFLICT;
        }

#ifndef NDEBUG
        assert(propagate() == Result::UNCHANGED);
        assert(snap_to() == Result::UNCHANGED);
#endif

        return changed ? Result::CHANGED : Result::UNCHANGED;
    }

    std::vector<int>& d_column_h;
    std::vector<int>& d_column_l;
    std::vector<int>& d_sum_h;
    std::vector<int>& d_sum_l;

  private:
    Result snap_to(uint32_t index) {
        if (!d_output.is_fixed(index)) {
            return Result::UNCHANGED;
        }

        const int expected = d_output.get_value(index) ? 1 : 0;
        bool changed = false;

        // Output's parity is different from sum's parity, so adjust sum.
        if ((d_sum_h[index] & 1) != expected) {
            --d_sum_h[index];
            changed = true;
        }
        if ((d_sum_l[index] & 1) != expected) {
            ++d_sum_l[index];
            changed = true;
        }

        if (d_sum_h[index] < d_sum_l[index] || d_sum_l[index] < 0) {
            return Result::CONFLICT;
        }

        return changed ? Result::CHANGED : Result::UNCHANGED;
    }

    // Update the sum of a column to the parity of the output for that column.
    // e.g. [0,2] if the answer is 1, goes to [1,1].
    Result snap_to() {
        Result result = Result::UNCHANGED;
        // Make sure each column's sum is consistent with the output.
        for (uint32_t i = 0; i < d_bit_width; ++i) {
            result |= snap_to(i);
            if (is_conflict(result)) {
                return Result::CONFLICT;
            }
        }
        return result;
    }

    // Make sure that all the counts are consistent.
    Result propagate() {
        bool changed = false;

        if (d_sum_l[0] > d_column_l[0]) {
            d_column_l[0] = d_sum_l[0];
            changed = true;
        }
        else if (d_sum_l[0] < d_column_l[0]) {
            d_sum_l[0] = d_column_l[0];
            changed = true;
        }

        if (d_sum_h[0] < d_column_h[0]) {
            d_column_h[0] = d_sum_h[0];
            changed = true;
        }
        else if (d_sum_h[0] > d_column_h[0]) {
            d_sum_h[0] = d_column_h[0];
            changed = true;
        }

        for (uint32_t i = 1; i < d_bit_width; ++i) {
            int& a_low = d_sum_l[i];
            int& a_high = d_sum_h[i];
            int& b_low = d_column_l[i];
            int& b_high = d_column_h[i];
            const int c_low = d_sum_l[i - 1] / 2;  // Carry term's interval.
            const int c_high = d_sum_h[i - 1] / 2;

            if (a_low < b_low + c_low) {
                a_low = b_low + c_low;
                changed = true;
            }

            if (a_high > b_high + c_high) {
                a_high = b_high + c_high;
                changed = true;
            }

            if (a_low - b_high > c_low) {
                const int candidate_low = (a_low - b_high) * 2;
                assert(candidate_low > d_sum_l[i - 1]);
                d_sum_l[i - 1] = candidate_low;
                changed = true;
            }

            if (a_high - b_low < c_high) {
                const int candidate_high = (a_high - b_low) * 2 + 1;
                assert(candidate_high < d_sum_h[i - 1]);
                d_sum_h[i - 1] = candidate_high;
                changed = true;
            }

            const int new_b_low = a_low - c_high;
            if (new_b_low > b_low) {
                b_low = new_b_low;
                changed = true;
            }

            const int new_b_high = a_high - c_low;
            if (new_b_high < b_high) {
                b_high = new_b_high;
                changed = true;
            }
        }

        return changed ? Result::CHANGED : Result::UNCHANGED;
    }

    bool in_conflict() const {
        for (uint32_t i = 0; i < d_bit_width; ++i) {
            if (d_sum_l[i] > d_sum_h[i] || d_column_l[i] > d_column_h[i]) {
                return true;
            }
        }
        return false;
    }

    uint32_t d_bit_width;
    FeasibleDomain& d_output;
};

// Uses the zeroes / ones present adjust the column counts.
void adjust_columns(const FeasibleDomain& x,
                    const FeasibleDomain& y,
                    std::vector<int>& column_l,
                    std::vector<int>& column_h) {
    const uint32_t width = x.width();
    std::vector<uint8_t> y_zero(width, 0);
    std::vector<uint8_t> x_zero(width, 0);

    for (uint32_t i = 0; i < width; ++i) {
        y_zero[i] = static_cast<uint8_t>(y.is_fixed_zero(i));
        x_zero[i] = static_cast<uint8_t>(x.is_fixed_zero(i));
    }

    for (uint32_t i = 0; i < width; ++i) {
        if (y_zero[i]) {
            for (uint32_t j = i; j < width; ++j) {
                --column_h[j];
            }
        }

        if (x_zero[i]) {
            for (uint32_t j = i; j < width; ++j) {
                // if the row hasn't already been zeroed out.
                if (!y_zero[j - i]) {
                    --column_h[j];
                }
            }
        }

        // check if there are any pairs of ones.
        if (x.is_fixed_one(i)) {
            const uint32_t limit = width - i;
            for (uint32_t j = 0; j < limit; ++j) {
                if (y.is_fixed_one(j)) {
                    // a pair of ones. Increase the lower bound.
                    ++column_l[i + j];
                }
            }
        }
    }
}
}  // namespace

FdpMulOperator::FdpMulOperator(OperatorStatistics& stats,
                               DomainVector& children,
                               FeasibleDomain* self)
    : FdpOperator(node::Kind::BV_MUL, "fdp_bv_mul", children, self),
      d_stats(stats) {}

Result
FdpMulOperator::fixed_bits_both_way() {
    // This logic is ported/translated from the STP project.

    assert(d_children.size() == 2);

    FeasibleDomain& x = *d_children[0];
    FeasibleDomain& y = *d_children[1];
    FeasibleDomain& output = *d_self;

    const uint32_t width = x.width();
    assert(width == y.width());
    assert(width == output.width());

    Result result = Result::UNCHANGED;

    Result trailing = fix_trailing_zeroes(x, y, output);
    if (is_conflict(trailing)) {
        return Result::CONFLICT;
    }
    result |= trailing;

    std::vector<int> column_h(width);  // maximum number of true partial products.
    std::vector<int> column_l(width);  // minimum number of true partial products.
    std::vector<int> sum_h(width);     // maximum number of column based on partial products summation.
    std::vector<int> sum_l(width);     // minimum number of column based on partial products summation.

    ColumnCounts counts(column_h, column_l, sum_h, sum_l, width, output);
    // Use the number of zeroes and ones in a column to update the possible counts.
    adjust_columns(x, y, column_l, column_h);
    counts.rebuild_sums();

    Result fixed_point = counts.fixed_point();
    if (is_conflict(fixed_point)) {
        return Result::CONFLICT;
    }

    auto& sum_low = counts.d_sum_l;
    auto& sum_high = counts.d_sum_h;
    auto& col_low = counts.d_column_l;
    auto& col_high = counts.d_column_h;

    // If any of the sums have a cardinality of 1. Set the result.
    for (uint32_t column = 0; column < width; ++column) {
        if (sum_low[column] == sum_high[column]) {
            // (1) If the output has a known value. Set the output.
            const bool bit_value = (sum_high[column] & 1) != 0;
            result |= output.set_fixed(column, bit_value);
            if (is_conflict(result)) {
                return Result::CONFLICT;
            }
        }
    }

    for (uint32_t column = 0; column < width; ++column) {
        if (col_low[column] == col_high[column]) {
            // (2) Knowledge of the sum may fix the operands.
            Result r = adjust_multiplication(
                x, y, column, col_high[column]);
            if (is_conflict(r)) {
                return Result::CONFLICT;
            }
            result |= r;
        }
    }

    if (is_changed(result)) {
        Result r = fix_trailing_zeroes(x, y, output);
        if (is_conflict(r)) {
            return Result::CONFLICT;
        }
    }

    return result;
}

Result
FdpMulOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    FeasibleDomain& c0 = *d_children[0];
    FeasibleDomain& c1 = *d_children[1];
    FeasibleDomain& output = *d_self;

    // input => output
    result |= update_interval_by_multiply(c0, c1, output);
    if (is_conflict(result)) {
        return Result::CONFLICT;
    }

    // output => input.
    // When multiplication overflows, the output interval is modulo
    // reduced and we cannot recover the correct feasible domains anymore,
    // so only perform the backward propagation if c0_max * c1_max fits.
    BitVector c0_max = c0.is_interval_complementary() ? BitVector::mk_ones(c0.width()) : c0.interval_max(),
              c1_max = c1.is_interval_complementary() ? BitVector::mk_ones(c1.width()) : c1.interval_max();
    if (!c0_max.is_umul_overflow(c1_max)) {
        result |= update_interval_by_division(output, c0, c1);
        if (is_conflict(result)) {
            return Result::CONFLICT;
        }
        result |= update_interval_by_division(output, c1, c0);
    }

    return result;
}

Result
FdpMulOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_mul);
    ++d_stats.d_num_fdp_mul;

    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_children[1]->width());
    assert(d_children[0]->width() == d_self->width());

    Result result = Result::UNCHANGED;

    result |= fixed_bits_both_way();
    if (is_conflict(result)) {
        return result;
    }
    result |= interval_both_way();

    return result;
}

}  // namespace bzla::preprocess::pass::fdp
