#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "node/node.h"
#include "node/node_manager.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "util/statistics.h"

namespace bzla::preprocess::pass::fdp::test {

inline OperatorStatistics& TestOperatorStats() {
    static util::Statistics stats;
    static OperatorStatistics op_stats(stats, "test::fdp::");
    return op_stats;
}

inline void ExpectInvariant(const FeasibleDomain& domain) {
    std::ostringstream err;
    ASSERT_TRUE(domain.check_invariants(&err))
        << "domain=" << domain.to_string(true) << " error=" << err.str();
}

inline void ExpectAllInvariant(
    std::initializer_list<FeasibleDomain*> domains) {
    for (FeasibleDomain* domain : domains) {
        ExpectInvariant(*domain);
    }
}

inline uint64_t MaskForWidth(uint32_t width) {
    if (width == 0) {
        return 0;
    }
    if (width >= 64) {
        return std::numeric_limits<uint64_t>::max();
    }
    return (uint64_t{1} << width) - 1;
}

inline BitVector BvFromUint(uint32_t width, uint64_t value) {
    return BitVector::from_ui(width, value & MaskForWidth(width));
}

inline void SetUnsigned(FeasibleDomain* domain, uint64_t value) {
    domain->set_constant(BvFromUint(domain->width(), value));
}

inline uint64_t DomainValue(const FeasibleDomain& domain) {
    uint64_t result = 0;
    for (uint32_t i = 0; i < domain.width(); ++i) {
        EXPECT_TRUE(domain.is_fixed(i)) << "bit=" << i;
        if (domain.is_fixed(i) && domain.get_value(i)) {
            result |= (uint64_t{1} << i);
        }
    }
    return result;
}

inline bool IsTrueNode(const Node& node) {
    return node.is_value() && node.value<bool>();
}

inline Node EncodeFixedBits(NodeManager& nm,
                            const Node& term,
                            const FeasibleDomain& domain) {
    if (term.type().is_bool()) {
        if (!domain.is_fixed(0)) {
            return nm.mk_value(true);
        }
        return domain.get_value(0) ? term : nm.mk_node(node::Kind::NOT, {term});
    }

    BitVector mask = BitVector::mk_zero(domain.width());
    BitVector constant = BitVector::mk_zero(domain.width());
    for (uint32_t i = 0; i < domain.width(); ++i) {
        mask.set_bit(i, domain.is_fixed(i) ? 1 : 0);
        if (domain.is_fixed(i) && domain.get_value(i)) {
            constant.set_bit(i, 1);
        }
    }

    if (mask.is_zero()) {
        return nm.mk_value(true);
    }
    Node masked = nm.mk_node(node::Kind::BV_AND, {term, nm.mk_value(mask)});
    return nm.mk_node(node::Kind::EQUAL, {masked, nm.mk_value(constant)});
}

inline Node EncodeInterval(NodeManager& nm,
                           const Node& term,
                           const FeasibleDomain& domain) {
    if (!term.type().is_bv()) {
        return nm.mk_value(true);
    }

    BitVector lower = domain.interval_min();
    BitVector upper = domain.interval_max();
    Node encoded;
    if (domain.is_interval_complementary()) {
        BitVector range = lower.bvadd(upper.bvneg());
        encoded = nm.mk_node(node::Kind::BV_ULE,
                             {nm.mk_node(node::Kind::BV_SUB,
                                         {term, nm.mk_value(upper)}),
                              nm.mk_value(range)});
    }
    else {
        if (!lower.is_zero()) {
            encoded = nm.mk_node(node::Kind::BV_ULE, {nm.mk_value(lower), term});
        }
        if (!upper.is_ones()) {
            Node upper_node =
                nm.mk_node(node::Kind::BV_ULE, {term, nm.mk_value(upper)});
            encoded = encoded.is_null() ? upper_node
                                        : nm.mk_node(node::Kind::AND,
                                                     {encoded, upper_node});
        }
    }

    return encoded.is_null() ? nm.mk_value(true) : encoded;
}

inline Node EncodeFeasibleDomain(NodeManager& nm,
                                 const Node& term,
                                 const FeasibleDomain& domain) {
    std::vector<Node> parts;
    Node bits = EncodeFixedBits(nm, term, domain);
    if (!IsTrueNode(bits)) {
        parts.push_back(bits);
    }
    Node interval = EncodeInterval(nm, term, domain);
    if (!IsTrueNode(interval)) {
        parts.push_back(interval);
    }

    if (parts.empty()) {
        return nm.mk_value(true);
    }
    if (parts.size() == 1) {
        return parts[0];
    }
    return nm.mk_node(node::Kind::AND, parts);
}

inline bool DomainHasModel(const FeasibleDomain& domain) {
    const uint32_t width = domain.width();
    if (width > 8) {
        // Generation keeps widths small, but don't fail noisily for larger ones.
        return true;
    }
    const uint64_t upper = uint64_t{1} << width;
    for (uint64_t value = 0; value < upper; ++value) {
        if (feasible_domain_holds_unsigned(domain, static_cast<uint32_t>(value))) {
            return true;
        }
    }
    return false;
}

inline FeasibleDomain RandomDomain(std::mt19937& rng,
                                   uint32_t width,
                                   bool is_bool = false) {
    constexpr uint32_t kMaxAttempts = 8;
    for (uint32_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
        FeasibleDomain domain(width);
        auto rand_bool = [&]() {
            return std::uniform_int_distribution<int>(0, 1)(rng) != 0;
        };

        if (is_bool) {
            if (rand_bool()) {
                domain.set_constant(rand_bool());
            }
        }
        else {
            const uint64_t mask = MaskForWidth(width);
            std::uniform_int_distribution<uint64_t> dist(0, mask);
            const bool use_interval = rand_bool();
            if (use_interval) {
                uint64_t a = dist(rng);
                uint64_t b = dist(rng);
                auto [lo, hi] = std::minmax(a, b);
                BitVector lower = BvFromUint(width, lo);
                BitVector upper = BvFromUint(width, hi);
                const bool complementary = rand_bool() && lo < hi;
                Result interval_res =
                    domain.apply_interval_constraint(lower,
                                                     upper,
                                                     complementary);
                if (is_conflict(interval_res)) {
                    continue;
                }
            }

            std::uniform_int_distribution<uint32_t> bit_dist(0, width - 1);
            std::uniform_int_distribution<uint32_t> count_dist(
                0, std::max<uint32_t>(1, width / 2));
            uint32_t fixed_bits = count_dist(rng);
            bool conflict = false;
            for (uint32_t i = 0; i < fixed_bits; ++i) {
                Result res = domain.set_fixed(bit_dist(rng), rand_bool());
                if (is_conflict(res)) {
                    conflict = true;
                    break;
                }
            }
            if (conflict) {
                continue;
            }

            Result tighten_res = domain.tighten_interval_by_fixed_bits();
            if (is_conflict(tighten_res)) {
                continue;
            }
            tighten_res = domain.tighten_fixed_bits_by_interval();
            if (is_conflict(tighten_res)) {
                continue;
            }
            domain.consume_state();
        }

        if (DomainHasModel(domain)) {
            return domain;
        }
    }

    return FeasibleDomain(width);
}

inline Result RunOperatorToFixedPoint(FdpOperator& op,
                                      FdpOperator::DomainVector& children,
                                      FeasibleDomain& output) {
    Result res = Result::UPDATED;
    while (is_updated(res)) {
        res = op.apply();
        if (is_conflict(res)) {
            return res;
        }
        res |= tighten_each_interval(children, &output);
        if (is_conflict(res)) {
            return res;
        }
        res |= tighten_each_fixed_bits(children, &output);
        if (is_conflict(res)) {
            return res;
        }
    }
    return res;
}

}  // namespace bzla::preprocess::pass::fdp::test
