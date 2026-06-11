#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_shifting.h"

#include <cassert>
#include <iostream>
#include <limits>

#include "env.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp {

namespace {

void deep_copy_feasible_domain(FeasibleDomain &domain,
                               const FeasibleDomain &source) {
    // Preserve both the fixed-bit encoding and interval representation exactly.
    domain.copy_from(source);
}

// domain <= source1 \cup source2
Result deep_union_feasible_domain(FeasibleDomain &domain,
                                  const FeasibleDomain &source1,
                                  const FeasibleDomain &source2) {
    assert(domain.width() == source1.width());
    assert(domain.width() == source2.width());

    uint32_t width = domain.width();
    Result result = Result::UNCHANGED;

    for (uint32_t i = 0; i < width; ++i) {
        if (!source1.is_fixed(i) || !source2.is_fixed(i)) {
            if (domain.is_fixed(i)) {
                return Result::CONFLICT;
            }
            continue;
        }
        else if (source1.get_value(i) != source2.get_value(i)) {
            if (domain.is_fixed(i)) {
                return Result::CONFLICT;
            }
            continue;
        }

        // Both source1 and source2 have the same fixed value
        if (!domain.is_fixed(i)) {
            result |= domain.set_fixed(i, source1.get_value(i));
        }
        else if (domain.get_value(i) != source1.get_value(i)) {
            return Result::CONFLICT;
        }
    }

    return result;
}

uint32_t uint32_bit_mask(uint32_t index) {
    assert(index < 32);
    return uint32_t{1} << index;
}

void get_uint32_shift_range(const FeasibleDomain &shift,
                            uint32_t &min_shift,
                            uint32_t &max_shift) {
    const uint32_t width = shift.width();
    const uint32_t uint32_width = 32;  // uint32_t max shift
    bool max_overflow = false;
    bool min_overflow = false;
    min_shift = 0;
    max_shift = 0;

    // use interval to get min/max shift amount
    if (!shift.is_interval_complementary()) {
        BitVector min_bv = shift.interval_min();
        BitVector max_bv = shift.interval_max();

        for (uint32_t i = uint32_width; i < width; ++i) {
            if (min_bv.bit(i))
                min_overflow = true;
            if (max_bv.bit(i))
                max_overflow = true;
            if (max_overflow && min_overflow)
                break;
        }

        if (!min_overflow) {
            min_shift = static_cast<uint32_t>(std::stoul(min_bv.str(10)));
        }
        else {
            min_shift = std::numeric_limits<uint32_t>::max();
        }

        if (!max_overflow) {
            max_shift = static_cast<uint32_t>(std::stoul(max_bv.str(10)));
        }
        else {
            max_shift = std::numeric_limits<uint32_t>::max();
        }
    }
    else {
        // complementary interval, compute min/max from fixed bits
        for (uint32_t i = uint32_width; i < width; ++i) {
            if (!shift.is_fixed_zero(i))
                max_overflow = true;
            if (shift.is_fixed_one(i))
                min_overflow = true;
            if (max_overflow && min_overflow)
                break;
        }

        for (uint32_t i = 0; i < std::min(width, uint32_width); ++i) {
            if (!shift.is_fixed(i)) {
                max_shift |= uint32_bit_mask(i);
            }
            else if (shift.is_fixed_one(i)) {
                max_shift |= uint32_bit_mask(i);
                min_shift |= uint32_bit_mask(i);
            }
        }
    }

    if (max_overflow)
        max_shift = std::numeric_limits<uint32_t>::max();
    if (min_overflow)
        min_shift = std::numeric_limits<uint32_t>::max();

    return;
}

uint32_t get_u32int_shift_from_alternation(const FeasibleDomain &domain) {
    uint32_t alter_bit = std::numeric_limits<uint32_t>::max();
    const uint32_t width = domain.width();

    assert(domain.is_fixed(width - 1));

    bool value = domain.get_value(width - 1);  // get sign bit value
    bool found = false;

    for (int32_t i = width - 1; i >= 0; --i) {
        if (!domain.is_fixed(i))
            continue;
        if (domain.get_value(i) != value) {
            alter_bit = i;
            found = true;
            break;
        }
    }

    if (!found)
        return alter_bit;
    else
        return width - 2 - alter_bit;
}

}  // namespace

FdpLeftShiftOperator::FdpLeftShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_SHL, "fdp_bv_shl", children, self), d_stats(stats) {}

Result
FdpLeftShiftOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;

    FeasibleDomain &op = *d_children[0];
    FeasibleDomain &shift = *d_children[1];
    FeasibleDomain &output = *d_self;

    const uint32_t self_width = d_self->width();
    constexpr uint32_t uint32_width = 32;

    // The topmost number of possible shifts in uint32_t
    // this will shift out every bit
    const uint32_t max_possible_shift_num = self_width + 1;

    std::unique_ptr<bool[]> possible_shifts =
        std::make_unique<bool[]>(max_possible_shift_num);

    for (unsigned i = 0; i < max_possible_shift_num; ++i)
        possible_shifts[i] = false;

    // get possible shift amounts
    uint32_t min_shift, max_shift;
    get_uint32_shift_range(shift, min_shift, max_shift);

    int position_of_first_one = -1;
    for (uint32_t i = 0; i < d_self->width(); i++) {
        if (output.is_fixed_one(i)) {
            position_of_first_one = i;
            break;
        }
    }

    // The shift must be less than the position of the first one in the output
    if (position_of_first_one >= 0) {
        if ((uint32_t)position_of_first_one < min_shift)
            return Result::CONFLICT;

        max_shift = std::min(max_shift, (uint32_t)position_of_first_one);
    }

    // Mark possible shift amounts
    for (uint32_t i = min_shift; i <= std::min(self_width, max_shift); i++) {
        if (feasible_domain_holds_unsigned(shift, i))
            possible_shifts[i] = true;
    }

    if (max_shift >= self_width)  // shift out all bits is possible
        possible_shifts[self_width] = true;

    // Check each possible shift amount for conflicts
    for (uint32_t shift_iter = 0; shift_iter < max_possible_shift_num; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;

        for (uint32_t column = 0; column < self_width; ++column) {
            if (column < shift_iter) {
                if (output.is_fixed_one(column)) {
                    // output bit is 1 but shifted from 0 bit
                    possible_shifts[shift_iter] = false;
                    break;
                }
            }
            else {
                // if they are fixed to different values. That's wrong.
                if (output.is_fixed(column) && op.is_fixed(column - shift_iter) &&
                    (output.get_value(column) != op.get_value(column - shift_iter))) {
                    possible_shifts[shift_iter] = false;
                    break;
                }
            }
        }
    }

    // Now we get the number of possible shifts
    int32_t possible_shifts_count = 0;
    for (uint32_t i = 0; i < max_possible_shift_num; ++i) {
        if (possible_shifts[i]) {
            possible_shifts_count++;
        }
    }

    if (possible_shifts_count == 0)
        return Result::CONFLICT;

    // We have a list of all the possible shift amounts.
    // We take the union of all the bits that are possible to fix shift
    std::unique_ptr<bool[]> v_fixed = std::make_unique<bool[]>(self_width);
    std::unique_ptr<bool[]> v_value = std::make_unique<bool[]>(self_width);
    for (uint32_t i = 0; i < self_width; ++i) {
        v_fixed[i] = false;
        v_value[i] = false;
    }

    bool first = true;
    for (uint32_t shift_iter = 0; shift_iter < max_possible_shift_num; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;
        if (first) {
            first = false;
            for (uint32_t i = 0; i < self_width; ++i) {
                v_fixed[i] = true;
                if (i < uint32_width)
                    v_value[i] = (0 != (shift_iter & uint32_bit_mask(i)));
                else
                    v_value[i] = false;
            }
        }
        else {
            // union with previous possible shift
            for (uint32_t i = 0; i < self_width && i < uint32_width; ++i) {
                if (!v_fixed[i])
                    continue;
                if (v_value[i] != (0 != (shift_iter & uint32_bit_mask(i))))
                    v_fixed[i] = false;
            }
        }
    }

    if (possible_shifts[max_possible_shift_num - 1]) {
        // This means we may shift out everything
        // Need the union for every possible shift >= self_width

        FeasibleDomain bit_width = create_constant_feasible_domain(self_width, self_width);
        FeasibleDomain working(shift);
        // working >= self_width  <->  NOT working < self_width

        FeasibleDomain output(1);
        output.set_fixed(0, false);

        DomainVector children = {&working, &bit_width};

        [[maybe_unused]] Result res =
            FdpLessThanOperator(d_stats, children, &output).apply();
        assert(!is_conflict(res));

        for (uint32_t i = 0; i < self_width; ++i) {
            if (!working.is_fixed(i) && v_fixed[i]) {
                v_fixed[i] = false;
            }
            if (working.is_fixed(i) && v_fixed[i] &&
                (working.get_value(i) != v_value[i])) {
                v_fixed[i] = false;
            }
            if (first && working.is_fixed(i)) {  // no less shift possible
                v_fixed[i] = true;
                v_value[i] = working.get_value(i);
            }
        }
    }

#if 0
    std::cout << "Set of possible shifts:" << std::endl;
    for (int32_t i = self_width - 1; i >= 0; --i) {
        if (v_fixed[i])
            std::cout << (v_value[i] ? '1' : '0');
        else
            std::cout << '*';
    }
    std::cout << std::endl;
#endif

    // Now, we can update shift
    for (uint32_t i = 0; i < self_width; ++i) {
        if (v_fixed[i]) {
            if (!shift.is_fixed(i)) {
                result |= shift.set_fixed(i, v_value[i]);
            }
            else if (shift.get_value(i) != v_value[i]) {
                return Result::CONFLICT;
            }
        }
    }

    // We start to update op
    std::unique_ptr<bool[]> candidates = std::make_unique<bool[]>(self_width);
    for (uint32_t i = 0; i < self_width; ++i)
        candidates[i] = !op.is_fixed(i);

    // candidate : ture - the bit unfixed

    for (uint32_t shift_iter = 0; shift_iter < max_possible_shift_num; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;
        for (uint32_t j = 0; j < shift_iter; ++j) {
            candidates[self_width - 1 - j] = false;
        }
    }
    // candidate : true  - the unfixed input bits in every possible fixing

    // Check all candidates have the same output values.
    for (uint32_t candidate = 0; candidate < self_width; ++candidate) {
        bool first = true, value = false;
        if (!candidates[candidate])
            continue;

        for (uint32_t shift_iter = 0; shift_iter < max_possible_shift_num; ++shift_iter) {
            if (!possible_shifts[shift_iter])
                continue;

            uint32_t idx = candidate + shift_iter;
            if (!output.is_fixed(idx)) {
                candidates[candidate] = false;
                break;
            }
            else {
                if (first) {
                    first = false;
                    value = output.get_value(idx);
                }
                else if (value != output.get_value(idx)) {
                    candidates[candidate] = false;
                    break;
                }
            }
        }

        if (candidates[candidate]) {
            assert(!op.is_fixed(candidate));
            result |= op.set_fixed(candidate, value);
        }
    }
    // update op done

    // judge output fixed bits by every possible shift and op
    for (uint32_t column = 0; column < self_width; ++column) {
        bool same = true;
        bool value = false;
        bool first = true;

        for (uint32_t shift_iter = min_shift; shift_iter < max_possible_shift_num; ++shift_iter) {
            if (!same)
                break;

            if (!possible_shifts[shift_iter])
                continue;

            if (column < shift_iter) {  // weill have shifted in zero
                if (first) {
                    first = false;
                    value = false;
                }
                else if (value) {
                    same = false;
                }
            }

            else {
                uint32_t idx = column - shift_iter;
                if (!op.is_fixed(idx)) {
                    same = false;
                }
                else if (first) {
                    first = false;
                    value = op.get_value(idx);
                }
                else if (value != op.get_value(idx)) {
                    same = false;
                }
            }
        }

        if (same) {
            if (!output.is_fixed(column)) {
                result |= output.set_fixed(column, value);
            }
            else if (output.get_value(column) != value) {
                return Result::CONFLICT;
            }
        }
    }

    return result;
}

Result
FdpLeftShiftOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    // #TODO: Support interval propagation for left shift
    return result;
}

Result
FdpLeftShiftOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_leftshift);
    ++d_stats.d_num_fdp_leftshift;

    // Assertions
    assert(d_children.size() == 2);
    assert(d_children[0]->width() == d_self->width());

    Result result = Result::UNCHANGED;

    result |= fixed_bits_both_way();
    // result |= interval_both_way();

    return result;
}

FdpRightShiftOperator::FdpRightShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_SHR, "fdp_bv_shr", children, self), d_stats(stats) {}

Result
FdpRightShiftOperator::fixed_bits_both_way() {
    // Use left shift to implement right shift fixed bits propagation

    Result result = Result::UNCHANGED;

    FeasibleDomain &op = *d_children[0];
    FeasibleDomain &shift = *d_children[1];
    FeasibleDomain &output = *d_self;

    const uint32_t width = d_self->width();

    FeasibleDomain op_reverse(op.width());
    FeasibleDomain output_reverse(output.width());

    // Reverse the output and Input
    for (uint32_t i = 0; i < width; ++i) {
        if (op.is_fixed(i)) {
            op_reverse.set_fixed(width - 1 - i, op.get_value(i));
        }
        if (output.is_fixed(i)) {
            output_reverse.set_fixed(width - 1 - i, output.get_value(i));
        }
    }

    DomainVector children = {&op_reverse, &shift};

    result |= FdpLeftShiftOperator(d_stats, children, &output_reverse).fixed_bits_both_way();

    if (result == Result::CONFLICT)
        return result;

    // Reverse back the output and Input
    for (uint32_t i = 0; i < width; ++i) {
        int rev_bit = width - 1 - i;
        if (op_reverse.is_fixed(rev_bit)) {
            if (!op.is_fixed(i)) {
                result |= op.set_fixed(i, op_reverse.get_value(rev_bit));
            }
            else if (op.get_value(i) != op_reverse.get_value(rev_bit)) {
                return Result::CONFLICT;
            }
        }
        if (output_reverse.is_fixed(rev_bit)) {
            if (!output.is_fixed(i)) {
                result |= output.set_fixed(i, output_reverse.get_value(rev_bit));
            }
            else if (output.get_value(i) != output_reverse.get_value(rev_bit)) {
                return Result::CONFLICT;
            }
        }
    }

    return result;
}

Result
FdpRightShiftOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    return result;
}

Result
FdpRightShiftOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_rightshift);
    ++d_stats.d_num_fdp_rightshift;

    // Assertions
    assert(d_children.size() == 2);

    Result result = Result::UNCHANGED;

    result |= fixed_bits_both_way();

    // result |= interval_both_way();

    return result;
}

FdpArithRightShiftOperator::FdpArithRightShiftOperator(OperatorStatistics &stats, DomainVector &children, FeasibleDomain *self)
    : FdpOperator(node::Kind::BV_ASHR, "fdp_bv_ashr", children, self), d_stats(stats) {}

Result
FdpArithRightShiftOperator::fixed_bits_both_way() {
    Result result = Result::UNCHANGED;

    const uint32_t width = d_self->width();
    const uint32_t sign_bit_index = width - 1;

    FeasibleDomain &op = *d_children[0];
    FeasibleDomain &shift = *d_children[1];
    FeasibleDomain &output = *d_self;

    if (!op.is_fixed(sign_bit_index)) {
        // if sign bit is not fixed, we split into two cases
        FeasibleDomain op_sign_zero(width);
        FeasibleDomain op_sign_one(width);
        FeasibleDomain shift_1(shift.width());
        FeasibleDomain shift_2(shift.width());
        FeasibleDomain output_1(width);
        FeasibleDomain output_2(width);

        // Preserve the full feasible domains in each split branch. Copying only
        // fixed bits would discard interval constraints and could widen the
        // surviving branch when it is copied back after the other branch
        // conflicts, violating monotonicity of propagation.
        deep_copy_feasible_domain(op_sign_zero, op);
        deep_copy_feasible_domain(op_sign_one, op);
        deep_copy_feasible_domain(shift_1, shift);
        deep_copy_feasible_domain(shift_2, shift);
        deep_copy_feasible_domain(output_1, output);
        deep_copy_feasible_domain(output_2, output);

        op_sign_zero.set_fixed(sign_bit_index, false);
        op_sign_one.set_fixed(sign_bit_index, true);

        DomainVector children_1 = {&op_sign_zero, &shift_1};
        DomainVector children_2 = {&op_sign_one, &shift_2};

        Result r1 =
            FdpArithRightShiftOperator(d_stats, children_1, &output_1).fixed_bits_both_way();
        Result r2 =
            FdpArithRightShiftOperator(d_stats, children_2, &output_2).fixed_bits_both_way();

        if (r1 == Result::CONFLICT && r2 == Result::CONFLICT) {
            return Result::CONFLICT;
        }
        else if (r1 == Result::CONFLICT) {
            deep_copy_feasible_domain(op, op_sign_one);
            deep_copy_feasible_domain(shift, shift_2);
            deep_copy_feasible_domain(output, output_2);
            return r2;
        }
        else if (r2 == Result::CONFLICT) {
            deep_copy_feasible_domain(op, op_sign_zero);
            deep_copy_feasible_domain(shift, shift_1);
            deep_copy_feasible_domain(output, output_1);
            return r1;
        }

        // union the two feasible domains
        result |= deep_union_feasible_domain(op, op_sign_zero, op_sign_one);
        result |= deep_union_feasible_domain(shift, shift_1, shift_2);
        result |= deep_union_feasible_domain(output, output_1, output_2);

        return result;
    }

    assert(op.is_fixed(sign_bit_index));

    const uint32_t max_possible_shift = width + 1;
    std::unique_ptr<bool[]> possible_shifts =
        std::make_unique<bool[]>(max_possible_shift);
    for (unsigned i = 0; i < max_possible_shift; ++i)
        possible_shifts[i] = false;

    // if either of the top two bits are fixed they must be the same
    if (output.is_fixed(sign_bit_index)) {
        if (output.get_value(sign_bit_index) != op.get_value(sign_bit_index)) {
            return Result::CONFLICT;
        }
    }
    else {  // output sign bit unfixed
        result |= output.set_fixed(sign_bit_index, op.get_value(sign_bit_index));
    }

    assert(output.is_fixed(sign_bit_index));

    uint32_t min_shift, max_shift;
    get_uint32_shift_range(shift, min_shift, max_shift);

    max_shift = std::min(max_shift, get_u32int_shift_from_alternation(output));

    for (uint32_t shift_iter = min_shift; shift_iter <= std::min(width, max_shift); shift_iter++) {
        if (feasible_domain_holds_unsigned(shift, shift_iter))
            possible_shifts[shift_iter] = true;
    }

    if (max_shift >= width)  // shift out all bits is possible
        possible_shifts[width] = true;

    // Check each possible shift amount for conflicts
    for (uint32_t shift_iter = min_shift; shift_iter < max_possible_shift; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;

        for (uint32_t column = 0; column < width; ++column) {
            uint32_t op_index = column + shift_iter;
            if (op_index > width - 1)  // which should be shifted in sign bit
                continue;

            // column + shift_iter <= width - 1
            if (output.is_fixed(column) && op.is_fixed(op_index) &&
                (output.get_value(column) != op.get_value(op_index))) {
                possible_shifts[shift_iter] = false;
                break;  // break column loop
            }
        }
    }

    int32_t possible_shifts_count = 0;
    for (uint32_t i = 0; i < max_possible_shift; ++i) {
        if (possible_shifts[i]) {
            possible_shifts_count++;
            max_shift = i;
        }
    }

    // No possible shifts
    if (possible_shifts_count == 0)
        return Result::CONFLICT;

    // get union of all possible shifts
    std::unique_ptr<bool[]> v_fixed = std::make_unique<bool[]>(width);
    std::unique_ptr<bool[]> v_value = std::make_unique<bool[]>(width);
    for (uint32_t i = 0; i < width; ++i) {
        v_fixed[i] = false;
        v_value[i] = false;
    }

    bool first = true;
    for (uint32_t shift_iter = 0; shift_iter < max_possible_shift - 1; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;
        if (first) {
            first = false;
            for (uint32_t i = 0; i < width; ++i) {
                v_fixed[i] = true;
                if (i < 32)
                    v_value[i] = (0 != (shift_iter & uint32_bit_mask(i)));
                else
                    v_value[i] = false;
            }
        }
        else {
            // union with previous possible shift
            for (uint32_t i = 0; i < width && i < 32; ++i) {
                if (!v_fixed[i])
                    continue;
                if (v_value[i] != (0 != (shift_iter & uint32_bit_mask(i))))
                    v_fixed[i] = false;
            }
        }
    }

    if (possible_shifts[width]) {
        // This means we may shift out everything
        // Need the union for every possible shift >= self_width
        FeasibleDomain bit_width = create_constant_feasible_domain(width, width);
        FeasibleDomain working(shift);
        // working >= self_width  <->  NOT working < self_width

        FeasibleDomain output(1);
        output.set_fixed(0, false);
        DomainVector children = {&working, &bit_width};
        [[maybe_unused]] Result res =
            FdpLessThanOperator(d_stats, children, &output).apply();
        assert(!is_conflict(res));

        for (uint32_t i = 0; i < width; ++i) {
            if (!working.is_fixed(i) && v_fixed[i]) {
                v_fixed[i] = false;
            }
            if (working.is_fixed(i) && v_fixed[i] &&
                (working.get_value(i) != v_value[i])) {
                v_fixed[i] = false;
            }
            if (first && working.is_fixed(i)) {  // no less shift possible
                v_fixed[i] = true;
                v_value[i] = working.get_value(i);
            }
        }
    }

#if 0
    std::cout << "Set of possible shifts:" << std::endl;
    for (int32_t i = width - 1; i >= 0; --i) {
        if (v_fixed[i])
            std::cout << (v_value[i] ? '1' : '0');
        else
            std::cout << '*';
    }
    std::cout << std::endl;
#endif

    // update shift
    for (uint32_t i = 0; i < width; ++i) {
        if (!v_fixed[i])
            continue;

        if (!shift.is_fixed(i)) {
            result |= shift.set_fixed(i, v_value[i]);
        }
        else if (shift.get_value(i) != v_value[i]) {
            return Result::CONFLICT;
        }
    }

    // update shift done

    std::unique_ptr<bool[]> candidates = std::make_unique<bool[]>(width);
    for (uint32_t i = 0; i < width; ++i)
        candidates[i] = !op.is_fixed(i);
    // candidate : ture - the bit unfixed

    for (uint32_t shift_iter = 0; shift_iter < max_possible_shift; ++shift_iter) {
        if (!possible_shifts[shift_iter])
            continue;
        // if this shift is possible, then some bits will be shifted out
        for (uint32_t j = 0; j < shift_iter; ++j) {
            candidates[j] = false;
        }
    }
    // cadidate: true - the unfixed input bits in every possible fixing

    for (uint32_t candidate = 0; candidate < width; ++candidate) {
        bool first = true;
        bool value = false;
        if (!candidates[candidate])
            continue;
        // candidates[candidate] == true

        for (uint32_t shift_iter = 0; shift_iter < max_possible_shift; ++shift_iter) {
            if (!possible_shifts[shift_iter])
                continue;

            if (shift_iter > candidate)
                continue;

            uint32_t idx = candidate - shift_iter;
            if (!output.is_fixed(idx)) {
                candidates[candidate] = false;
                break;
            }
            else {
                if (first) {
                    first = false;
                    value = output.get_value(idx);
                }
                else if (value != output.get_value(idx)) {
                    candidates[candidate] = false;
                    break;
                }
            }
        }

        if (candidates[candidate]) {
            assert(!op.is_fixed(candidate));
            result |= op.set_fixed(candidate, value);
        }
    }
    // update op done

    assert(op.is_fixed(sign_bit_index));
    bool sign_bit_value = op.get_value(sign_bit_index);

    for (uint32_t column = 0; column < width; ++column) {
        bool same = true;
        bool value = false;
        bool first = true;

        for (uint32_t shift_iter = 0; shift_iter < max_possible_shift; ++shift_iter) {
            if (!same)
                break;

            if (!possible_shifts[shift_iter])
                continue;

            // possible_shifts[shift_iter] == true
            uint32_t op_index = column + shift_iter;
            if (op_index > width - 1) {
                // shifted in sign bit
                if (first) {
                    first = false;
                    value = sign_bit_value;
                }
                else if (value != sign_bit_value) {
                    same = false;
                }
            }
            else {
                if (!op.is_fixed(op_index)) {
                    same = false;
                }
                else if (first) {
                    first = false;
                    value = op.get_value(op_index);
                }
                else if (value != op.get_value(op_index)) {
                    same = false;
                }
            }
        }

        if (same) {
            if (!output.is_fixed(column)) {
                result |= output.set_fixed(column, value);
            }
            else if (output.get_value(column) != value) {
                return Result::CONFLICT;
            }
        }
    }

    return result;
}

Result
FdpArithRightShiftOperator::interval_both_way() {
    Result result = Result::UNCHANGED;
    return result;
}

Result
FdpArithRightShiftOperator::apply() {
    util::Timer timer(d_stats.d_time_fdp_arith_rightshift);
    ++d_stats.d_num_fdp_arith_rightshift;

    // Assertions
    assert(d_children.size() == 2);

    Result result = Result::UNCHANGED;

    result |= fixed_bits_both_way();

    // result |= interval_both_way();

    return result;
}

}  // namespace bzla::preprocess::pass::fdp
