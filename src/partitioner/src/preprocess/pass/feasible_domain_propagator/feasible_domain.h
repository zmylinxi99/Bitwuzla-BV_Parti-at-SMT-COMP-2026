#pragma once

#include <cassert>
#include <cstdint>
#include <iosfwd>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

enum class EncodingMark : uint8_t {
    NONE = 0,
    BITS,
    INTERVAL,
    BOTH,
    IMPLIED,
    REMOVED,
};

const char *mark_to_cstr(EncodingMark mark);

class FeasibleDomain {
    friend std::ostream &operator<<(std::ostream &out,
                                    const FeasibleDomain &domain);

  public:
    FeasibleDomain(uint32_t width);
    ~FeasibleDomain();

    FeasibleDomain(FeasibleDomain &&) noexcept = default;
    FeasibleDomain &operator=(FeasibleDomain &&) noexcept = default;
    FeasibleDomain(const FeasibleDomain &);
    FeasibleDomain &operator=(const FeasibleDomain &) = delete;

    uint32_t width() const { return d_width; }
    bool is_one_bit() const { return d_width == 1; }

    bool is_fixed(uint32_t index) const {
        assert(index < d_width);
        return d_fixed[index];
    }

    bool is_fixed_zero(uint32_t index) const {
        assert(index < d_width);
        return d_fixed[index] && !d_value[index];
    }

    bool is_fixed_one(uint32_t index) const {
        assert(index < d_width);
        return d_fixed[index] && d_value[index];
    }

    /**
     * Replace the current domain with an exact copy of another one.
     * This is used when restoring snapshots from the persistent history
     * where we do not want to trigger any pending-state bookkeeping or
     * extra tightening passes (those will happen when we actually propagate).
     */
    void copy_from(const FeasibleDomain &domain);

    bool get_value(uint32_t index) const {
        assert(index < d_width);
        assert(d_fixed[index]);
        return d_value[index];
    }

    void reset();
    void reset(const FeasibleDomain &domain, EncodingMark mark = EncodingMark::BOTH);
    Result overwrite(const FeasibleDomain &domain, EncodingMark mark = EncodingMark::BOTH);
    Result merge(const FeasibleDomain &domain);

    Result pending_state() const { return d_pending_state; }
    Result consume_state();

    Result set_constant(bool value);
    Result set_constant(const BitVector &value);
    BitVector get_fixed_value() const;

    bool try_set_update_by_interval();
    void stage_last_interval_size();
    void stage_last_interval_size(BitVector &&size);
    Result tighten_interval_by_fixed_bits();
    Result tighten_fixed_bits_by_interval();

    Result set_fixed(uint32_t index, bool value);
    Result judge_valid() const;

    uint32_t count_fixed() const { return d_fixed_count; }
    bool is_totally_fixed() const { return d_fixed_count == d_width; }
    bool is_totally_unfixed() const { return d_fixed_count == 0; }

    // Returns the size of the feasible interval.
    BitVector calc_interval_size() const;
    // Returns the size of the feasible domain by dynamic programming.
    BitVector calc_domain_size() const;

    bool is_full() const {
        if (d_fixed_count > 0)
            return false;
        if (d_complementary)
            return false;
        return d_interval_min.is_zero() && d_interval_max.is_ones();
    }

    const BitVector &interval_min() const { return d_interval_min; }
    const BitVector &interval_max() const { return d_interval_max; }

    bool is_interval_complementary() const { return d_complementary; }
    void set_interval_complementary(bool value) { d_complementary = value; }

    bool is_interval_constant() const;
    bool is_interval_complete() const;

    Result intersect_interval_constraint(const BitVector &min,
                                         const BitVector &max,
                                         bool complementary);

    Result apply_interval_constraint(const BitVector &min,
                                     const BitVector &max,
                                     bool complementary);

    static constexpr unsigned d_interval_update_thres = 2;  // >> 2 means 25% range change

    std::string to_string(bool verbose = false) const;
    bool check_invariants(std::ostream *out = nullptr) const;

    Result d_pending_state{Result::UNCHANGED};

  private:
    BitVector build_value_from_fixed_bits() const;
    void sync_interval_if_totally_fixed();
    uint32_t d_width;
    // Fixed Bits
    uint32_t d_fixed_count{0};
    std::unique_ptr<bool[]> d_fixed;
    std::unique_ptr<bool[]> d_value;

    // Feasible Inteval
    // using ConstBV to represent interval, bit-vector represent multiprecision number
    bool d_complementary;  // \neg [a, b] = [0, a] \cup [b, 2^n-1]
    BitVector d_interval_max;
    BitVector d_interval_min;
    BitVector d_last_interval_size;

    void mark_pending_state(Result state);

    // Helper functions for tighten_interval_by_fixed_bits
    // return valid. If valid is false, no value satisfies the constraints.
    BitVector find_min_consistent(const BitVector &limit, bool &overflow) const;
    BitVector find_max_consistent(const BitVector &limit, bool &overflow) const;
};

}  // namespace bzla::preprocess::pass::fdp
