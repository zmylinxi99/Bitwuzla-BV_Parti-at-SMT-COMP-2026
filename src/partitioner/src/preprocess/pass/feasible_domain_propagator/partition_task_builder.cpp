#include "preprocess/pass/feasible_domain_propagator/partition_task_builder.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <queue>
#include <utility>
#include <vector>

#include "bv/bitvector.h"
#include "node/node.h"
#include "node/node_kind.h"
#include "node/node_manager.h"
#include "preprocess/pass/feasible_domain_propagator/propagator.h"


// #define BZLA_FDP_PARTI_ENABLE_DEBUG
#define BZLA_FDP_PARTI_INFO

namespace bzla::preprocess::pass::fdp {


namespace {
// constexpr bool kEnableProbing = true;
}  // namespace

PartitionTaskBuilder::PartitionTaskBuilder(Propagator& propagator,
                                           uint64_t seed)
    : d_propagator(propagator), d_seed(seed) {
}

const char* PartitionTaskBuilder::kind_to_string(PartitionKind kind) {
    switch (kind) {
        case PartitionKind::BOOL:
            return "BOOL";
        case PartitionKind::BV_BIT:
            return "BV_BIT";
        case PartitionKind::BV_INTERVAL:
            return "BV_INTERVAL";
        case PartitionKind::BV_ADD_OVERFLOW:
            return "BV_ADD_OVERFLOW";
        default:
            return "UNKNOWN";
    }
}

const char* PartitionTaskBuilder::overflow_split_kind_to_string(
    OverflowSplitKind kind) {
    switch (kind) {
        case OverflowSplitKind::COMPARE_CHILD:
            return "COMPARE_CHILD";
        case OverflowSplitKind::OUTPUT_BOUNDARY:
            return "OUTPUT_BOUNDARY";
        default:
            return "UNKNOWN";
    }
}

struct PartitionTaskBuilder::PartitionAction {
    PartitionKind d_kind{PartitionKind::BV_BIT};
    uint32_t d_node_id{UINT32_MAX};
    OverflowSplitKind d_overflow_split{
        OverflowSplitKind::COMPARE_CHILD};

    double d_heuristic{0.0};

    void display() const {
        std::cout << "PartitionAction: kind="
                  << PartitionTaskBuilder::kind_to_string(d_kind)
                  << ", node_id=" << d_node_id
                  << ", heuristic=" << d_heuristic;
        if (d_kind == PartitionKind::BV_ADD_OVERFLOW) {
            std::cout << ", overflow_split="
                      << PartitionTaskBuilder::overflow_split_kind_to_string(
                             d_overflow_split);
        }
        std::cout << "\n";
    }
};

struct PartitionTaskBuilder::BranchEval {
    double d_uncertainty_reduction{0.0};
    bool d_conflict{false};
    bool d_valid{true};
    Node d_guard;
};

int PartitionTaskBuilder::compare_action_key(
    const PartitionAction& a,
    const PartitionAction& b) {
    if (a.d_kind != b.d_kind) {
        return a.d_kind < b.d_kind ? -1 : 1;
    }
    if (a.d_node_id != b.d_node_id) {
        return a.d_node_id < b.d_node_id ? -1 : 1;
    }
    if (a.d_kind == PartitionKind::BV_ADD_OVERFLOW &&
        b.d_kind == PartitionKind::BV_ADD_OVERFLOW &&
        a.d_overflow_split != b.d_overflow_split) {
        return a.d_overflow_split < b.d_overflow_split ? -1 : 1;
    }
    return 0;
}

bool PartitionTaskBuilder::action_key_less(const PartitionAction& a,
                                           const PartitionAction& b) {
    return compare_action_key(a, b) < 0;
}

bool PartitionTaskBuilder::build(std::vector<Node>& left,
                                 std::vector<Node>& right) {
    if (!init_history()) {
        return false;
    }

    compute_partition_heuristics();

    std::vector<PartitionAction> actions = collect_partition_actions();
    if (actions.empty()) {
        return false;
    }
    PartitionAction best;
    BranchEval best_left;
    BranchEval best_right;
    if (!select_best_action(actions, best, best_left, best_right)) {
        return false;
    }

    emit_partition(best, best_left, best_right, left, right);
    return true;
}

bool PartitionTaskBuilder::init_history() {
    if (!d_propagator.d_fd_prefix) {
        return false;
    }
    d_history = d_propagator.d_fd_prefix.get();
    d_baseline_level = d_history->max_level();
    return true;
}

const FeasibleDomain* PartitionTaskBuilder::baseline_domain_ptr(
    uint32_t id) const {
    if (d_history) {
        const FeasibleDomain* dom_ptr = d_history->get_ptr(id, d_baseline_level);
        if (dom_ptr) {
            return dom_ptr;
        }
    }
    return nullptr;
}

const FeasibleDomain& PartitionTaskBuilder::baseline_domain(uint32_t id) const {
    assert(id < d_propagator.d_num_nodes);
    const FeasibleDomain* dom_ptr = baseline_domain_ptr(id);
    return dom_ptr ? *dom_ptr : d_propagator.d_domains[id];
}

void PartitionTaskBuilder::compute_difficulty() {
    const size_t num_nodes = d_propagator.d_num_nodes;
    d_difficulty.assign(num_nodes, 0.0);
    std::vector<uint32_t> pending(num_nodes, 0);
    std::queue<uint32_t> work;
    for (uint32_t i = 0; i < num_nodes; ++i) {
        uint32_t parent_count = d_propagator.d_parents[i].size();
        pending[i] = parent_count;
        if (pending[i] == 0) {
            work.push(i);
        }
    }

    while (!work.empty()) {
        uint32_t cur = work.front();
        work.pop();
        const Node& cur_node = d_propagator.d_nodes[cur];
        double base = d_difficulty[cur] + node_difficulty(cur_node);
        for (uint32_t child_id : d_propagator.d_children[cur]) {
            d_difficulty[child_id] += base;
            if (pending[child_id] > 0 && --pending[child_id] == 0) {
                work.push(child_id);
            }
        }
    }
}

void PartitionTaskBuilder::compute_partition_heuristics() {
    compute_difficulty();

    const size_t num_nodes = d_propagator.d_num_nodes;
    d_baseline_uncertainty.assign(num_nodes, 0.0);
    d_node_weight.assign(num_nodes, 0.0);
    d_node_heuristic.assign(num_nodes, 0.0);
    d_baseline_total = 0.0;
    constexpr double kDifficultyWeight = 0.02;
    double max_weighted_uncertainty = 0.001;
    double max_occ = 0.0;
    double max_diff = 0.0;

    struct CandidateMetrics {
        uint32_t d_id;
        double d_weighted_uncertainty;
        double d_info_norm;
        double d_complementary;
        double d_occ;
        double d_diff;
    };
    std::vector<CandidateMetrics> candidates;
    candidates.reserve(num_nodes);

    for (uint32_t i = 0; i < num_nodes; ++i) {
        const Node& node = d_propagator.d_nodes[i];
        const Type& type = node.type();
        if (!type.is_bv() && !type.is_bool()) {
            continue;
        }
        const FeasibleDomain& dom = baseline_domain(i);
        const uint32_t width =
            type.is_bv() ? static_cast<uint32_t>(type.bv_size()) : 1u;
        const uint32_t fixed = dom.count_fixed();
        const uint32_t unfixed = width - fixed;

        double interval_bits;
        double complementary = 0.0;
        if (type.is_bv()) {
            interval_bits = interval_log2_size(dom);
            complementary = dom.is_interval_complementary() ? 1.0 : 0.0;
        }
        else {
            interval_bits = dom.is_totally_fixed() ? 0.0 : 1.0;
        }

        const double uncertainty =
            type.is_bool()
                ? (dom.is_totally_fixed() ? 0.0 : 1.0)
                : static_cast<double>(unfixed) + interval_bits;
        d_baseline_uncertainty[i] = uncertainty;

        double weight = 1.0;
        const double diff = d_difficulty[i];
        if (diff > 0.0) {
            weight += kDifficultyWeight * std::log1p(diff);
        }
        d_node_weight[i] = weight;

        const double weighted_uncertainty = weight * uncertainty;
        d_baseline_total += weighted_uncertainty;
        max_weighted_uncertainty =
            std::max(max_weighted_uncertainty, weighted_uncertainty);

        if (unfixed == 0) {
            continue;
        }

        const double info =
            static_cast<double>(fixed) +
            (static_cast<double>(width) - interval_bits);
        const double info_norm =
            width > 0 ? (info / (2.0 * static_cast<double>(width))) : 0.0;
        const double occ =
            static_cast<double>(d_propagator.d_parents[i].size());

        candidates.push_back({i,
                              weighted_uncertainty,
                              info_norm,
                              complementary,
                              occ,
                              diff});
        max_occ = std::max(max_occ, occ);
        max_diff = std::max(max_diff, diff);
    }

    const double occ_denom = max_occ > 0.0 ? std::log1p(max_occ) : 1.0;
    const double diff_denom = max_diff > 0.0 ? std::log1p(max_diff) : 1.0;

    for (const auto& cand : candidates) {
        const double unc_norm =
            max_weighted_uncertainty > 0.0
                ? cand.d_weighted_uncertainty / max_weighted_uncertainty
                : 0.0;
        const double occ_norm =
            max_occ > 0.0 ? std::log1p(cand.d_occ) / occ_denom : 0.0;
        const double diff_norm =
            max_diff > 0.0 ? std::log1p(cand.d_diff) / diff_denom : 0.0;
        const double structural = 0.6 * occ_norm + 0.4 * diff_norm;

        // Score prioritizes split potential, precision, and structural impact.
        double score = 0.0;
        score += 0.55 * unc_norm;
        score += 0.20 * cand.d_info_norm;
        score += 0.10 * cand.d_complementary;
        score += 0.10 * structural;
        if (d_propagator.d_nodes[cand.d_id].kind() ==
            node::Kind::CONSTANT) {
            score += 0.05;
        }
        d_node_heuristic[cand.d_id] = score;
    }
}

double PartitionTaskBuilder::interval_log2_size(const FeasibleDomain& dom) const {
    if (dom.is_one_bit()) {
        return dom.is_totally_fixed() ? 0.0 : 1.0;
    }
    if (dom.is_interval_complete()) {
        return static_cast<double>(dom.width());
    }
    BitVector size = dom.calc_interval_size();
    const uint64_t bitlen = dom.width() - size.count_leading_zeros();
    if (bitlen == 0) {
        return 0.0;
    }
    return static_cast<double>(bitlen - 1);
}

double PartitionTaskBuilder::node_difficulty(const Node& node) const {
    const node::Kind k = node.kind();
    const size_t degree = node.num_children();
    if (degree == 0) {
        return 0.0;
    }
    const uint32_t width =
        node.type().is_bv() ? static_cast<uint32_t>(node.type().bv_size()) : 1;
    const double w = static_cast<double>(std::max<uint32_t>(width, 1));

    auto mul_booth_cost = [&](const BitVector& bv) {
        bool last = bv.bit(0);
        uint32_t changes = 0;
        for (uint32_t i = 1; i < width; ++i) {
            bool cur = bv.bit(i);
            if (cur != last) {
                ++changes;
            }
            last = cur;
        }
        return 4.0 * w * static_cast<double>(changes);
    };

    if (k == node::Kind::BV_MUL && degree == 2) {
        // Booth-encoded multiplication: cost depends on runs in constant.
        for (size_t i = 0; i < 2; ++i) {
            const Node& child = node[i];
            if (child.is_value() && child.type().is_bv()) {
                return mul_booth_cost(child.value<BitVector>());
            }
        }
    }

    double score = 0.0;
    if (k == node::Kind::BV_MUL) {
        score = 4.0 * w * w * static_cast<double>(degree);
    }
    else if (k == node::Kind::BV_UDIV || k == node::Kind::BV_SDIV ||
             k == node::Kind::BV_UREM || k == node::Kind::BV_SREM ||
             k == node::Kind::BV_SMOD) {
        score = 16.0 * w * w;
    }
    else if (k == node::Kind::BV_CONCAT || k == node::Kind::BV_EXTRACT ||
             k == node::Kind::NOT || k == node::Kind::BV_NOT) {
        score = 0.0;
    }
    else if (k == node::Kind::EQUAL || k == node::Kind::BV_UGE ||
             k == node::Kind::BV_UGT || k == node::Kind::BV_ULE ||
             k == node::Kind::BV_ULT || k == node::Kind::BV_SGE ||
             k == node::Kind::BV_SGT || k == node::Kind::BV_SLE ||
             k == node::Kind::BV_SLT) {
        score = 6.0 * w;
    }
    else if (k == node::Kind::BV_SUB) {
        score = 20.0 * w;
    }
    else if (k == node::Kind::BV_NEG || k == node::Kind::BV_NEGO) {
        score = 6.0 * w;
    }
    else if (k == node::Kind::BV_ADD) {
        score = 14.0 * w *
                static_cast<double>(std::max<size_t>(degree, 1) - 1);
    }
    else if (k == node::Kind::BV_SHR || k == node::Kind::BV_SHL) {
        score = 29.0 * w;
    }
    else if (k == node::Kind::BV_ASHR) {
        score = 30.0 * w;
    }
    else if (k == node::Kind::BV_SIGN_EXTEND ||
             k == node::Kind::BV_ZERO_EXTEND) {
        score = 0.0;
    }
    else {
        score = w * static_cast<double>(degree);
    }
    return score;
}

double PartitionTaskBuilder::node_uncertainty(const FeasibleDomain& dom,
                                              const Type& type) const {
    if (type.is_bool()) {
        return dom.is_totally_fixed() ? 0.0 : 1.0;
    }
    if (!type.is_bv()) {
        return 0.0;
    }
    const uint32_t width = static_cast<uint32_t>(type.bv_size());
    const uint32_t unfixed = width - dom.count_fixed();
    const double interval_bits = interval_log2_size(dom);
    return static_cast<double>(unfixed) + interval_bits;
}

uint32_t PartitionTaskBuilder::choose_partition_bit(uint32_t id,
                                                    const FeasibleDomain& dom) const {
    const Type& type = d_propagator.d_nodes[id].type();
    const uint32_t width = static_cast<uint32_t>(type.bv_size());
    assert(width > 0);

    for (uint32_t idx = width; idx-- > 0;) {
        if (!dom.is_fixed(idx)) {
            return idx;
        }
    }
    return width - 1;
}

bool PartitionTaskBuilder::compute_interval_split(uint32_t node_id,
                                                  BitVector& l,
                                                  BitVector& u,
                                                  BitVector& l2,
                                                  BitVector& u2) const {
    const Node& node = d_propagator.d_nodes[node_id];
    const Type& type = node.type();
    assert(type.is_bv());
    const uint32_t width = static_cast<uint32_t>(type.bv_size());
    if (width == 0) {
        return false;
    }
    const FeasibleDomain& dom = baseline_domain(node_id);
    if (!dom.is_interval_complementary()) {
        return false;
    }
    const BitVector zero = BitVector::mk_zero(width);
    const BitVector ones = BitVector::mk_ones(width);
    l = zero;
    u = dom.interval_min();
    l2 = dom.interval_max();
    u2 = ones;
    return true;
}

bool PartitionTaskBuilder::overflow_const_child_info(const Node& add,
                                                     uint32_t& const_idx,
                                                     uint32_t& var_idx,
                                                     BitVector& limit) const {
    const bool c0_const = add[0].is_value();
    const bool c1_const = add[1].is_value();
    if (c0_const == c1_const) {
        return false;
    }
    const_idx = c0_const ? 0 : 1;
    var_idx = const_idx == 0 ? 1 : 0;
    BitVector c = add[const_idx].value<BitVector>();
    limit = c.bvneg();  // 2^k - c
    return true;
}

bool PartitionTaskBuilder::select_overflow_compare_child(
    uint32_t add_id,
    uint32_t& child_topo) const {
    if (add_id >= d_propagator.d_num_nodes) {
        return false;
    }
    const auto& children = d_propagator.d_children[add_id];
    if (children.size() < 2) {
        return false;
    }
    auto metrics = [&](uint32_t topo) {
        const double diff =
            topo < d_difficulty.size() ? d_difficulty[topo] : 0.0;
        const double unc =
            topo < d_baseline_uncertainty.size() ? d_baseline_uncertainty[topo]
                                                 : 0.0;
        return std::pair<double, double>(diff, unc);
    };
    const auto [diff0, unc0] = metrics(children[0]);
    const auto [diff1, unc1] = metrics(children[1]);

    const bool child0_dom = diff0 < diff1 && unc0 < unc1;
    const bool child1_dom = diff1 < diff0 && unc1 < unc0;
    if (child0_dom) {
        child_topo = children[0];
        return true;
    }
    if (child1_dom) {
        child_topo = children[1];
        return true;
    }

    // Prefer lower difficulty, then lower uncertainty.
    if (diff0 != diff1) {
        child_topo = diff0 < diff1 ? children[0] : children[1];
    }
    else if (unc0 != unc1) {
        child_topo = unc0 < unc1 ? children[0] : children[1];
    }
    else {
        child_topo = children[0];
    }
    return true;
}

void PartitionTaskBuilder::normalize_wrapping_interval(const BitVector& l,
                                                       const BitVector& u,
                                                       BitVector& out_min,
                                                       BitVector& out_max,
                                                       bool& out_comp) const {
    // Represent [l,u] as either normal interval (comp=false, min<=max)
    // or complementary interval (comp=true, [0,min] U [max, ones]).
    if (l.compare(u) <= 0) {
        out_min = l;
        out_max = u;
        out_comp = false;
    }
    else {
        out_min = u;
        out_max = l;
        out_comp = true;
    }
}

Result PartitionTaskBuilder::apply_wrapping_interval_constraint(
    FeasibleDomain& dom,
    const BitVector& l,
    const BitVector& u) const {
    BitVector min;
    BitVector max;
    bool comp = false;
    normalize_wrapping_interval(l, u, min, max, comp);
    return dom.apply_interval_constraint(min, max, comp);
}

Node PartitionTaskBuilder::mk_interval_lower_lit(const Node& term,
                                                 const BitVector& bound) const {
    if (bound.is_zero()) {
        return d_propagator.d_nm.mk_value(true);
    }
    Node bound_node = d_propagator.d_nm.mk_value(bound);
    Node lit =
        d_propagator.d_nm.mk_node(node::Kind::BV_UGE, {term, bound_node});
    return d_propagator.d_rewriter.rewrite(lit);
}

Node PartitionTaskBuilder::mk_interval_upper_lit(const Node& term,
                                                 const BitVector& bound) const {
    if (bound.is_ones()) {
        return d_propagator.d_nm.mk_value(true);
    }
    Node bound_node = d_propagator.d_nm.mk_value(bound);
    Node lit =
        d_propagator.d_nm.mk_node(node::Kind::BV_ULE, {term, bound_node});
    return d_propagator.d_rewriter.rewrite(lit);
}

Node PartitionTaskBuilder::mk_bit_lit(const Node& term,
                                      uint32_t bit,
                                      bool value) const {
    Node bit_node =
        d_propagator.d_nm.mk_node(node::Kind::BV_EXTRACT, {term}, {bit, bit});
    Node rhs = d_propagator.d_nm.mk_value(
        value ? BitVector::mk_one(1) : BitVector::mk_zero(1));
    Node eq = d_propagator.d_nm.mk_node(node::Kind::EQUAL, {bit_node, rhs});
    return d_propagator.d_rewriter.rewrite(eq);
}

void PartitionTaskBuilder::clear_queue() {
    while (!d_propagator.d_queue.empty()) {
        uint32_t id = d_propagator.d_queue.top();
        d_propagator.d_queue.pop();
        d_propagator.d_enqueued[id] = false;
    }
}

void PartitionTaskBuilder::restore_baseline_nodes() {
    clear_queue();
    // for (uint32_t id : d_propagator.d_changed_nodes) {
    //     const FeasibleDomain* base_dom = baseline_domain_ptr(id);
    //     if (base_dom) {
    //         d_propagator.d_domains[id].copy_from(*base_dom);
    //     }
    //     else {
    //         d_propagator.d_domains[id].reset();
    //     }
    //     d_propagator.d_changed[id] = false;
    //     d_propagator.d_enqueued[id] = false;
    // }
    
    // brute-force restore
    for (uint32_t id = 0; id < d_propagator.d_num_nodes; ++id) {
        const FeasibleDomain* base_dom = baseline_domain_ptr(id);
        if (base_dom) {
            d_propagator.d_domains[id].copy_from(*base_dom);
        }
        else {
            d_propagator.d_domains[id].reset();
        }
        d_propagator.d_changed[id] = false;
        d_propagator.d_enqueued[id] = false;
    }

    d_propagator.d_changed_nodes.clear();
    d_propagator.d_conflict = false;
}

PartitionTaskBuilder::BranchEval PartitionTaskBuilder::probe_branch(
    const PartitionAction& action,
    bool take_left) {
    // Ensure probe starts from a clean baseline and always rolls back,
    // even when returning early on invalid/conflict shortcuts.
    restore_baseline_nodes();
    struct ProbeRollbackGuard {
        PartitionTaskBuilder* builder;
        ~ProbeRollbackGuard() { builder->restore_baseline_nodes(); }
    } rollback{this};

    const uint32_t node_id = action.d_node_id;
    uint32_t target_id = node_id;
    Result res = Result::UNCHANGED;
    BranchEval eval;
    eval.d_guard = d_propagator.d_nm.mk_value(true);
    const Node& term = d_propagator.d_nodes[node_id];

    if (action.d_kind == PartitionKind::BOOL) {
        if (take_left) {
            eval.d_guard = d_propagator.d_rewriter.rewrite(
                d_propagator.d_nm.mk_node(node::Kind::NOT, {term}));
        }
        else {
            eval.d_guard = term;
        }
        res = d_propagator.d_domains[node_id].set_constant(!take_left);
    }
    else if (action.d_kind == PartitionKind::BV_BIT) {
        const FeasibleDomain& dom = baseline_domain(node_id);
        const uint32_t bit = choose_partition_bit(node_id, dom);
        eval.d_guard = mk_bit_lit(term, bit, !take_left);
        res = d_propagator.d_domains[node_id].set_fixed(bit, !take_left);
    }
    else if (action.d_kind == PartitionKind::BV_INTERVAL) {
        BitVector l;
        BitVector u;
        BitVector l2;
        BitVector u2;
        if (!compute_interval_split(node_id, l, u, l2, u2)) {
            eval.d_valid = false;
            return eval;
        }
        eval.d_guard = take_left ? mk_interval_upper_lit(term, u)
                                 : mk_interval_lower_lit(term, l2);
        res = apply_wrapping_interval_constraint(
            d_propagator.d_domains[node_id],
            take_left ? l : l2,
            take_left ? u : u2);
    }
    else if (action.d_kind == PartitionKind::BV_ADD_OVERFLOW) {
        const Node& add = term;
        assert(add.kind() == node::Kind::BV_ADD && add.num_children() == 2);
        if (action.d_overflow_split == OverflowSplitKind::OUTPUT_BOUNDARY) {
            BitVector l;
            BitVector u;
            BitVector l2;
            BitVector u2;
            if (!compute_interval_split(node_id, l, u, l2, u2)) {
                eval.d_valid = false;
                return eval;
            }
            eval.d_guard = take_left ? mk_interval_upper_lit(term, u)
                                     : mk_interval_lower_lit(term, l2);
            res = apply_wrapping_interval_constraint(
                d_propagator.d_domains[node_id],
                take_left ? l : l2,
                take_left ? u : u2);
        }
        else {
            uint32_t const_idx = 0;
            uint32_t var_idx = 0;
            BitVector limit;
            if (overflow_const_child_info(add, const_idx, var_idx, limit)) {
                const Node& var = add[var_idx];
                Node limit_node = d_propagator.d_nm.mk_value(limit);
                Node lt = d_propagator.d_nm.mk_node(node::Kind::BV_ULT,
                                                    {var, limit_node});
                Node ge = d_propagator.d_nm.mk_node(node::Kind::BV_UGE,
                                                    {var, limit_node});
                eval.d_guard =
                    d_propagator.d_rewriter.rewrite(take_left ? lt : ge);
                const auto& children = d_propagator.d_children[node_id];
                if (var_idx >= children.size()) {
                    eval.d_valid = false;
                    return eval;
                }
                target_id = children[var_idx];
                if (limit.is_zero()) {
                    // x < 0 is unsat; x >= 0 is tautology.
                    if (take_left) {
                        eval.d_conflict = true;
                        return eval;
                    }
                    return eval;
                }
                const BitVector l =
                    take_left ? BitVector::mk_zero(limit.size()) : limit;
                const BitVector u = take_left ? limit.bvdec()
                                              : BitVector::mk_ones(limit.size());
                res = apply_wrapping_interval_constraint(
                    d_propagator.d_domains[target_id],
                    l,
                    u);
            }
            else {
                const bool c0_const = add[0].is_value();
                const bool c1_const = add[1].is_value();
                if (c0_const && c1_const) {
                    eval.d_valid = false;
                    return eval;
                }
                uint32_t child_topo = 0;
                if (!select_overflow_compare_child(node_id, child_topo)) {
                    eval.d_valid = false;
                    return eval;
                }
                const Node& t = d_propagator.d_nodes[child_topo];
                Node cmp = d_propagator.d_nm.mk_node(node::Kind::BV_ULT,
                                                     {add, t});
                cmp = d_propagator.d_rewriter.rewrite(cmp);
                if (take_left) {
                    eval.d_guard = cmp;
                }
                else {
                    eval.d_guard = d_propagator.d_rewriter.rewrite(
                        d_propagator.d_nm.mk_node(node::Kind::NOT, {cmp}));
                }
                const FeasibleDomain& x_dom = baseline_domain(child_topo);
                const uint32_t width =
                    static_cast<uint32_t>(add.type().bv_size());
                BitVector x_min = x_dom.is_interval_complementary()
                                      ? BitVector::mk_zero(width)
                                      : x_dom.interval_min();
                BitVector x_max = x_dom.is_interval_complementary()
                                      ? BitVector::mk_ones(width)
                                      : x_dom.interval_max();
                if (take_left) {
                    if (x_max.is_zero()) {
                        eval.d_conflict = true;
                        return eval;
                    }
                    const BitVector l = BitVector::mk_zero(width);
                    const BitVector u = x_max.bvdec();
                    res = apply_wrapping_interval_constraint(
                        d_propagator.d_domains[target_id],
                        l,
                        u);
                }
                else {
                    const BitVector l = x_min;
                    const BitVector u = BitVector::mk_ones(width);
                    res = apply_wrapping_interval_constraint(
                        d_propagator.d_domains[target_id],
                        l,
                        u);
                }
            }
        }
    }
    else {
        eval.d_valid = false;
        return eval;
    }

    if (is_conflict(res)) {
        d_propagator.collect_changed(target_id);
        eval.d_conflict = true;
        return eval;
    }

    if (is_changed(res) || is_updated(res)) {
        d_propagator.propagate_enqueue(target_id);
        d_propagator.feasible_domain_propagate();
    }

    if (d_propagator.d_conflict) {
        eval.d_conflict = true;
        return eval;
    }
    assert(d_baseline_total > 0.0);

    double prop_gain = 0.0;
    for (uint32_t id : d_propagator.d_changed_nodes) {
        const Node& n = d_propagator.d_nodes[id];
        const Type& t = n.type();
        const double base_unc = d_baseline_uncertainty[id];
        const double now_unc = node_uncertainty(d_propagator.d_domains[id],
                                                t);
        const double weight = d_node_weight[id];
        prop_gain += weight * (base_unc - now_unc);
    }
    eval.d_uncertainty_reduction =
        std::max(0.0, prop_gain / d_baseline_total);
    return eval;
}

void PartitionTaskBuilder::collect_action_buckets(
    std::vector<std::vector<PartitionAction>>& buckets) const {
    const size_t num_nodes = d_propagator.d_num_nodes;
    for (uint32_t i = 0; i < num_nodes; ++i) {
        const Node& node = d_propagator.d_nodes[i];
        const Type& type = node.type();
        if (!type.is_bv() && !type.is_bool()) {
            continue;
        }
        const FeasibleDomain& dom = baseline_domain(i);
        if (dom.is_totally_fixed()) {
            continue;
        }

        const bool is_var = node.kind() == node::Kind::CONSTANT;
        const bool want_var_actions = is_var;
        const bool want_overflow_action =
            type.is_bv() && node.kind() == node::Kind::BV_ADD &&
            node.num_children() == 2 && dom.is_interval_complementary();

        const double base_score = d_node_heuristic[i];
        if (base_score <= 0.0) {
            continue;
        }

        bool has_non_bool_child = false;
        if (type.is_bool() && !is_var) {
            for (uint32_t child_id : d_propagator.d_children[i]) {
                if (child_id >= d_propagator.d_num_nodes) {
                    continue;
                }
                if (!d_propagator.d_nodes[child_id].type().is_bool()) {
                    has_non_bool_child = true;
                    break;
                }
            }
        }

        // Branch on Boolean atoms that connect BV structure into the Boolean
        // skeleton, while keeping variable actions preferred.
        const bool want_branch_actions =
            type.is_bool() && !is_var && has_non_bool_child;
        if (want_branch_actions) {
            constexpr double kBranchActionScale = 0.5;
            PartitionAction act;
            act.d_kind = PartitionKind::BOOL;
            act.d_node_id = i;
            act.d_heuristic = base_score * kBranchActionScale;
            add_action(buckets, std::move(act));
        }

        if (want_var_actions) {
            if (type.is_bool()) {
                PartitionAction act;
                act.d_kind = PartitionKind::BOOL;
                act.d_node_id = i;
                act.d_heuristic = base_score;
                add_action(buckets, std::move(act));
            }
            else if (type.is_bv()) {
                const uint32_t width = static_cast<uint32_t>(type.bv_size());
                if (width == 0) {
                    continue;
                }

                // Bit fixing.
                {
                    PartitionAction act;
                    act.d_kind = PartitionKind::BV_BIT;
                    act.d_node_id = i;
                    act.d_heuristic = base_score;
                    add_action(buckets, std::move(act));
                }

                // Interval partitioning: only complementary ranges, split at 0.
                if (dom.is_interval_complementary()) {
                    PartitionAction act;
                    act.d_kind = PartitionKind::BV_INTERVAL;
                    act.d_node_id = i;
                    act.d_heuristic = base_score;
                    add_action(buckets, std::move(act));
                }
            }
        }

        // Overflow-aware cubes for binary addition when the output domain is
        // complementary (wrap-around ambiguity).
        if (want_overflow_action) {
            const Node& x = node[0];
            const Node& y = node[1];
            const bool x_const = x.is_value();
            const bool y_const = y.is_value();
            if (x_const && y_const) {
                continue;
            }

            PartitionAction act;
            act.d_kind = PartitionKind::BV_ADD_OVERFLOW;
            act.d_node_id = i;  // z is the BV_ADD node itself.
            act.d_heuristic = base_score;
            add_action(buckets, std::move(act));
        }
    }
}

void PartitionTaskBuilder::add_action(
    std::vector<std::vector<PartitionAction>>& buckets,
    PartitionAction&& act) const {
    auto& bucket = buckets[kind_index(act.d_kind)];
    bucket.emplace_back(std::move(act));
}

size_t PartitionTaskBuilder::kind_index(PartitionKind kind) const {
    return static_cast<size_t>(kind);
}

std::vector<PartitionTaskBuilder::PartitionAction>
PartitionTaskBuilder::collect_partition_actions() const {
    constexpr size_t kNumCandidates = 18;
    constexpr size_t kPartitionKindCount = 4;
    // constexpr size_t kDefaultMaxPerKind =
    //     (kNumCandidates + kPartitionKindCount - 1) / kPartitionKindCount;
    constexpr size_t kMaxPerKind[kPartitionKindCount] = {
        /*BOOL*/ 4,
        /*BV_BIT*/ 8,
        /*BV_INTERVAL*/ 4,
        /*BV_ADD_OVERFLOW*/ 2,
    };

    std::vector<std::vector<PartitionAction>> buckets(kPartitionKindCount);
    collect_action_buckets(buckets);

    std::vector<PartitionAction> actions;
    actions.reserve(kNumCandidates);
    // std::vector<PartitionAction> spillover;
    auto cmp = [](const PartitionAction& a, const PartitionAction& b) {
        if (a.d_heuristic != b.d_heuristic) {
            return a.d_heuristic > b.d_heuristic;
        }
        return PartitionTaskBuilder::action_key_less(a, b);
    };
    for (size_t kind_idx = 0; kind_idx < buckets.size(); ++kind_idx) {
        auto& bucket = buckets[kind_idx];
        if (bucket.empty()) {
            continue;
        }
        const size_t max_per_kind = kMaxPerKind[kind_idx];
        size_t take;
        if (bucket.size() > max_per_kind) {
            std::nth_element(bucket.begin(),
                             bucket.begin() +
                                 static_cast<ptrdiff_t>(max_per_kind),
                             bucket.end(),
                             cmp);
            std::sort(bucket.begin(),
                      bucket.begin() +
                          static_cast<ptrdiff_t>(max_per_kind),
                      cmp);
            take = max_per_kind;
        }
        else {
            take = bucket.size();
            std::sort(bucket.begin(), bucket.end(), cmp);
        }
        for (size_t i = 0; i < take; ++i) {
            actions.emplace_back(std::move(bucket[i]));
        }
        // for (size_t i = take; i < bucket.size(); ++i) {
        //     spillover.emplace_back(std::move(bucket[i]));
        // }
    }
    // if (actions.size() < kNumCandidates && !spillover.empty()) {
    //     std::sort(spillover.begin(),
    //               spillover.end(),
    //               cmp);
    //     for (size_t i = 0; i < spillover.size(); ++i) {
    //         actions.emplace_back(std::move(spillover[i]));
    //         if (actions.size() >= kNumCandidates) {
    //             break;
    //         }
    //     }
    // }
    return actions;
}

double PartitionTaskBuilder::score_action_with_probe(
    const PartitionAction& action,
    const BranchEval& left,
    const BranchEval& right) const {
    const double reduction =
        0.5 * (left.d_uncertainty_reduction +
               right.d_uncertainty_reduction);
    return action.d_heuristic + 0.6 * reduction;
}

bool PartitionTaskBuilder::select_best_action(
    const std::vector<PartitionAction>& actions,
    PartitionAction& best,
    BranchEval& best_left,
    BranchEval& best_right) {
    if (actions.empty()) {
        return false;
    }
    d_forced_common_guards.clear();

    struct Candidate {
        size_t d_index;
        PartitionAction d_action;
        double d_score;
        BranchEval d_left;
        BranchEval d_right;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(actions.size() * 2);

    auto evaluate_candidate = [&](PartitionAction action,
                                  size_t index) -> bool {
        BranchEval left = probe_branch(action, true);
        BranchEval right = probe_branch(action, false);

        if (!left.d_valid || !right.d_valid) {
            return true;
        }

        if (left.d_conflict && right.d_conflict) {
            // display action that caused conflict in both branches
#ifdef BZLA_FDP_PARTI_INFO
            std::cout << "Action " << index
                      << " caused conflict in both branches:\n";
            action.display();
#endif
            d_propagator.mark_partition_unsat();
            return false;
        }
        else if (left.d_conflict != right.d_conflict) {
            Node guard = left.d_conflict ? right.d_guard : left.d_guard;
            const bool is_bool =
                guard.is_value() && guard.type().is_bool();
            if (is_bool) {
                if (guard.value<bool>()) {
                    return true;
                }
                d_propagator.mark_partition_unsat();
                return false;
            }
            // display common guard:
#ifdef BZLA_FDP_PARTI_INFO
            std::cout << "Action " << index
                      << " caused conflict in one branch, adding common guard:\n";
            action.display();
            std::cout << "Common guard:\n";
            std::cout << guard << "\n";
#endif
            d_forced_common_guards.emplace_back(guard);
            return true;
        }

        const double score = score_action_with_probe(action, left, right);
        candidates.push_back({index,
                              std::move(action),
                              score,
                              std::move(left),
                              std::move(right)});
        return true;
    };

    for (size_t i = 0; i < actions.size(); ++i) {
        const PartitionAction& action = actions[i];
        if (action.d_kind == PartitionKind::BV_ADD_OVERFLOW) {
            
            PartitionAction compare_action = action;
            compare_action.d_overflow_split =
                OverflowSplitKind::COMPARE_CHILD;
            if (!evaluate_candidate(std::move(compare_action), i)) {
                return false;
            }

            PartitionAction boundary_action = action;
            boundary_action.d_overflow_split =
                OverflowSplitKind::OUTPUT_BOUNDARY;
            if (!evaluate_candidate(std::move(boundary_action), i)) {
                return false;
            }
        }
        else {
            if (!evaluate_candidate(action, i)) {
                return false;
            }
        }
    }

    if (candidates.empty()) {
        return false;
    }

    std::vector<double> weights;
    weights.reserve(candidates.size());
    double total = 0.0;
    for (const auto& cand : candidates) {
        double weight = cand.d_score > 0.0 ? cand.d_score : 0.0;
        weights.push_back(weight);
        total += weight;
#ifdef BZLA_FDP_PARTI_ENABLE_DEBUG
        std::cout << "Action " << cand.d_index << ": score=" << cand.d_score
                  << ", weight=" << weight << "\n";
        cand.d_action.display();
        // display domain of selected node
        std::cout << "Node domain:\n";
        std::cout << d_propagator.d_domains[cand.d_action.d_node_id]
                  << "\n";
        std::cout << d_propagator.d_nodes[cand.d_action.d_node_id]
                  << "\n\n";
#endif
    }

    std::mt19937_64 rng(d_seed);
    size_t picked = 0;
    if (total <= 0.0) {
        picked = static_cast<size_t>(rng() % candidates.size());
    }
    else {
        std::uniform_real_distribution<> dist(0.0, total);
        const double r = dist(rng);
        double acc = 0.0;
        for (size_t i = 0; i < weights.size(); ++i) {
            acc += weights[i];
            if (r <= acc) {
                picked = i;
                break;
            }
        }
    }
    best = candidates[picked].d_action;
    best_left = candidates[picked].d_left;
    best_right = candidates[picked].d_right;
    return true;
}

void PartitionTaskBuilder::emit_partition(
        const PartitionAction& best,
        const BranchEval& left_eval,
        const BranchEval& right_eval,
        std::vector<Node>& left,
        std::vector<Node>& right) {
#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[emit_partition] Picked partition action:\n";
    best.display();
    std::cout << "Node domain:\n";
    std::cout << d_propagator.d_domains[best.d_node_id] << "\n";
    // std::cout << d_propagator.d_nodes[best.d_node_id] << "\n\n";
#endif
    left = d_propagator.d_final_roots;
    right = d_propagator.d_final_roots;
#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[common_guards] Adding " << d_forced_common_guards.size() << " forced common guards\n";
#endif
    for (const Node& guard : d_forced_common_guards) {
        left.emplace_back(guard);
        right.emplace_back(guard);
// #ifdef BZLA_FDP_PARTI_INFO
//         std::cout << guard << "\n";
// #endif
    }

// #ifdef BZLA_FDP_PARTI_INFO
//     std::cout << "Left guard:\n";
//     std::cout << left_eval.d_guard << "\n";
//     std::cout << "Right guard:\n";
//     std::cout << right_eval.d_guard << "\n";
// #endif
    if (!left_eval.d_guard.is_null()) {
        left.emplace_back(left_eval.d_guard);
    }
    if (!right_eval.d_guard.is_null()) {
        right.emplace_back(right_eval.d_guard);
    }
}

}  // namespace bzla::preprocess::pass::fdp
