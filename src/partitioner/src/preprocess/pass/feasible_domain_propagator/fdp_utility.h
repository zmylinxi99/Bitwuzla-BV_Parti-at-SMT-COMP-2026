#pragma once

#include <cstdint>
#include <vector>

#include "bv/bitvector.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"

namespace bzla::preprocess::pass::fdp {

struct IntervalTerm {
    const BitVector &d_min;
    const BitVector &d_max;
    bool d_complementary;

    IntervalTerm(const BitVector &min, const BitVector &max, bool complementary);
    explicit IntervalTerm(const FeasibleDomain &domain);
};

bool intersects(const FeasibleDomain &a, const FeasibleDomain &b);
bool intersects(const BitVector &min_a,
                const BitVector &max_a,
                bool comp_a,
                const FeasibleDomain &b);
bool intersects(const BitVector &min_a,
                const BitVector &max_a,
                bool comp_a,
                const BitVector &min_b,
                const BitVector &max_b,
                bool comp_b);

uint32_t intersects_count(const FeasibleDomain &a, const FeasibleDomain &b);
uint32_t intersects_count(const BitVector &min_a,
                          const BitVector &max_a,
                          bool comp_a,
                          const FeasibleDomain &b);
uint32_t intersects_count(const BitVector &min_a,
                          const BitVector &max_a,
                          bool comp_a,
                          const BitVector &min_b,
                          const BitVector &max_b,
                          bool comp_b);

bool is_full_interval(const FeasibleDomain &domain);
bool is_full_interval(const BitVector &min, const BitVector &max, bool comp);

Result align_interval(FeasibleDomain &a, FeasibleDomain &b);
bool in_interval(const FeasibleDomain &domain, const BitVector &value);

// This function return the size of the interval - 1
BitVector feasible_domain_size(const BitVector &min,
                               const BitVector &max,
                               bool complementary);
int compare_feasible_domain_size(const BitVector &min_a,
                                 const BitVector &max_a,
                                 bool comp_a,
                                 const FeasibleDomain &b);
int compare_feasible_domain_size(const BitVector &min_a,
                                 const BitVector &max_a,
                                 bool comp_a,
                                 const BitVector &min_b,
                                 const BitVector &max_b,
                                 bool comp_b);

Result update_interval_by_plus(const IntervalTerm &a,
                               const IntervalTerm &b,
                               FeasibleDomain &output);
Result update_interval_by_minus(const FeasibleDomain &a,
                                const FeasibleDomain &b,
                                FeasibleDomain &output);
Result update_interval_by_plus(const FeasibleDomain &a,
                               const FeasibleDomain &b,
                               FeasibleDomain &output);
Result update_interval_by_multiply(const FeasibleDomain &a,
                                   const FeasibleDomain &b,
                                   FeasibleDomain &output);
Result update_interval_by_division(const FeasibleDomain &dividend,
                                   const FeasibleDomain &divisor,
                                   FeasibleDomain &output);

bool try_set_update_by_intervals(std::vector<FeasibleDomain *> &children,
                                 FeasibleDomain *output);

Result tighten_each_interval(std::vector<FeasibleDomain *> &children,
                             FeasibleDomain *output);
Result tighten_each_fixed_bits(std::vector<FeasibleDomain *> &children,
                               FeasibleDomain *output);

bool feasible_domain_holds_value(const FeasibleDomain &domain,
                                 const BitVector &value);
bool feasible_domain_holds_unsigned(const FeasibleDomain &domain,
                                    uint32_t value);

BitVector make_unsigned_max(const FeasibleDomain &domain);
BitVector make_unsigned_min(const FeasibleDomain &domain);

BitVector make_signed_max(const FeasibleDomain &domain);
BitVector make_signed_min(const FeasibleDomain &domain);

EncodingMark calc_encoding_mark(uint32_t width, uint32_t fixed_count, const BitVector &interval_size, const BitVector &domain_size, bool is_symbol);
EncodingMark calc_encoding_mark_diff(uint32_t width,
                                     uint32_t fixed_count,
                                     const BitVector &interval_size,
                                     const BitVector &domain_size,
                                     uint32_t prev_fixed_count,
                                     const BitVector &prev_interval_size,
                                     const BitVector &prev_domain_size);
// EncodingMark calc_encoding_mark_diff(uint32_t width,
//                                      uint32_t curr_fixed_count,
//                                      const BitVector &curr_interval_size,
//                                      uint32_t prev_fixed_count,
//                                      const BitVector &prev_interval_size,
//                                      const BitVector &prev_domain_size);

FeasibleDomain
create_constant_feasible_domain(uint32_t width, const BitVector &value);

FeasibleDomain
create_constant_feasible_domain(uint32_t width, uint32_t value);
}  // namespace bzla::preprocess::pass::fdp
