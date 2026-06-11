#include "preprocess/pass/feasible_domain_propagator/propagator.h"

#include <sys/types.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "bv/bitvector.h"
#include "env.h"
#include "node/node.h"
#include "node/node_kind.h"
#include "node/node_manager.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_prefix_array.h"
#include "preprocess/pass/feasible_domain_propagator/fdp_utility.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/partition_task_builder.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_arithmetic.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_boolean.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_composition.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_division.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_ite.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_multiplication.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_shifting.h"
#include "printer/smt2_printer.h"
#include "rewrite/rewriter.h"
#include "solver/bv/aig_bitblaster.h"
#include "util/statistics.h"

#ifdef BZLA_FDP_ENABLE_DEBUG_CHECKS
#define FDP_DEBUG_RUN(stmt) \
    do {                    \
        stmt;               \
    } while (0)
#else
#define FDP_DEBUG_RUN(stmt) \
    do {                    \
    } while (0)
#endif

// #define BZLA_FDP_PARTI_INFO

namespace {

constexpr uint64_t REMOVE_SKIP_THRESHOLD = 1'000'000;

using FdpClock = std::chrono::steady_clock;
FdpClock::time_point g_fdp_start;
bool g_fdp_timer_active = false;

void fdp_timer_start() {
    g_fdp_start = FdpClock::now();
    g_fdp_timer_active = true;
}

double fdp_elapsed_ms() {
    if (!g_fdp_timer_active) {
        return 0.0;
    }
    auto elapsed = FdpClock::now() - g_fdp_start;
    return std::chrono::duration<double, std::milli>(elapsed).count();
}

[[maybe_unused]] void fdp_time_log(const char* stage) {
    if (!g_fdp_timer_active || stage == nullptr) {
        return;
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << fdp_elapsed_ms();
    std::cout << "[fdp-timing] " << stage << " " << oss.str() << " ms\n";
}

struct BlastSizeEstimate {
    uint64_t aig_ands{0};
};

BlastSizeEstimate estimate_blast_size(const std::vector<bzla::Node>& roots) {
    BlastSizeEstimate estimate;
    bzla::bv::AigBitblaster bitblaster;

    for (const bzla::Node& root : roots) {
        assert(root.type().is_bool());
        bitblaster.bitblast(root);
    }

    estimate.aig_ands = bitblaster.num_aig_ands();
    return estimate;
}

double compute_improvement_ratio(uint64_t original, uint64_t simplified) {
    if (original == 0) {
        return simplified == 0 ? 0.0 : -1.0;
    }
    return (static_cast<double>(original) - static_cast<double>(simplified)) /
           static_cast<double>(original);
}

bool has_meaningful_blast_improvement(const BlastSizeEstimate& original,
                                      const BlastSizeEstimate& simplified) {
    constexpr double kMinImprovementRatio = 0.05;
    const double aig_improvement =
        compute_improvement_ratio(original.aig_ands, simplified.aig_ands);

#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[fdp] blast estimate original(aig=" << original.aig_ands
              << ") simplified(aig=" << simplified.aig_ands << "), improvement ratio: "
              << std::fixed << std::setprecision(2) << (aig_improvement * 100) << "%\n";
#endif

    return aig_improvement > kMinImprovementRatio;
}

}
namespace bzla::preprocess::pass::fdp {

Propagator::Propagator(Env& env, OperatorStatistics& stats)
    : d_nm(env.nm()), d_rewriter(env.rewriter()), d_stats(stats), d_conflict(false) {}

void Propagator::reset_run_state() {
    d_conflict = false;
    d_partition_unsat = false;
    d_num_nodes = 0;
    d_num_bool_nodes = 0;
    std::priority_queue<uint32_t>().swap(d_queue);
    d_nodes.clear();
    d_topo_dict.clear();
    d_domains.clear();
    d_roots.clear();
    d_constants.clear();
    d_parents.clear();
    d_children.clear();
    d_propagated_frontier.clear();
    d_reduced_frontier.clear();
    d_fd_prefix.reset();
    d_enqueued.reset();
    d_changed.reset();
    d_changed_nodes.clear();
    d_marks.clear();
    d_final_roots.clear();
    d_final_nodes.clear();
    d_final_ids.clear();
}

void Propagator::set_unsat_final() {
    d_final_roots.clear();
    d_final_nodes.clear();
    d_final_ids.clear();
    d_final_roots.emplace_back(d_nm.mk_value(false));
}

void Propagator::stage_prefix_level_zero() {
    d_fd_prefix = std::make_unique<FdPrefixArray>(d_num_nodes);
    for (uint32_t id : d_changed_nodes) {
        d_fd_prefix->update(0, id, d_domains[id]);
        d_changed[id] = false;
    }
    d_changed_nodes.clear();
    d_fd_prefix->finalize_level_zero();
}

void Propagator::init_nodes_in_topo(const std::vector<Node>& roots) {
    // Reset cached topology data before recomputing it.
    d_nodes.clear();
    d_roots.clear();
    d_parents.clear();
    d_children.clear();
    d_topo_dict.clear();

    std::unordered_map<uint64_t, uint32_t> indeg;
    indeg.reserve(roots.size());
    std::stack<Node> stack;

    d_num_nodes = 0;
    for (const Node& root : roots) {
        if (++indeg[root.id()] == 1)
            stack.push(root);
    }
    while (!stack.empty()) {
        const Node node = stack.top();
        stack.pop();
        ++d_num_nodes;
        for (size_t i = 0, sz = node.num_children(); i < sz; ++i) {
            const Node& child = node[i];
            uint64_t child_id = child.id();
            indeg.try_emplace(child_id, 0);
            if (++indeg[child_id] == 1)
                stack.push(child);
        }
    }
    d_nodes.reserve(d_num_nodes);
    d_topo_dict.reserve(d_num_nodes);
    // Topological Sort
    for (const Node& root : roots) {
        if (--indeg[root.id()] == 0)
            stack.push(root);
    }
    while (!stack.empty()) {
        const Node node = stack.top();
        stack.pop();
        d_topo_dict[node.id()] = d_nodes.size();
        d_nodes.emplace_back(node);
        for (size_t i = 0, sz = node.num_children(); i < sz; ++i) {
            const Node& child = node[i];
            uint64_t child_id = child.id();
            if (--indeg[child_id] == 0)
                stack.push(child);
        }
    }

    d_roots.reserve(roots.size());
    for (const Node& root : roots) {
        d_roots.emplace_back(d_topo_dict[root.id()]);
    }

    d_children.resize(d_num_nodes, std::vector<uint32_t>());
    d_parents.resize(d_num_nodes, std::vector<uint32_t>());

    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        const Node& node = d_nodes[i];
        for (size_t j = 0, sz = node.num_children(); j < sz; ++j) {
            const Node& child = node[j];
            uint64_t child_id = child.id();
            uint32_t child_topo = d_topo_dict[child_id];
            d_children[i].emplace_back(child_topo);
            d_parents[child_topo].emplace_back(i);
        }
    }
    // if (d_num_nodes == 0)
    //     return;
    // d_dag_size.resize(d_num_nodes, 1);
    // // calculate dag size for each node
    // for (int32_t i = int32_t(d_num_nodes) - 1; i >= 0; --i) {
    //     d_dag_size[i] = 1;
    //     for (uint32_t child_id : d_children[i]) {
    //         d_dag_size[i] += d_dag_size[child_id];
    //     }
    // }
}

void Propagator::initialize_domains() {
    d_domains.reserve(d_num_nodes);
    d_constants.reserve(d_num_nodes);
    d_enqueued = std::make_unique<bool[]>(d_num_nodes);
    d_changed = std::make_unique<bool[]>(d_num_nodes);
    d_marks.assign(d_num_nodes, EncodingMark::NONE);
    d_num_bool_nodes = 0;
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        d_enqueued[i] = false;
        d_changed[i] = false;
        const Node& node = d_nodes[i];
        const auto& type = node.type();
        if (type.is_bv()) {
            d_domains.emplace_back(static_cast<uint32_t>(type.bv_size()));
        }
        else {
            ++d_num_bool_nodes;
            d_domains.emplace_back(1);
        }
        if (node.kind() == node::Kind::VALUE) {
            d_constants.emplace_back(i);
        }
    }
}

void Propagator::collect_changed(uint32_t node_id) {
    // assert(node_id < d_num_nodes);
    if (!d_changed[node_id]) {
        d_changed_nodes.emplace_back(node_id);
        d_changed[node_id] = true;
    }
}

void Propagator::collect_changed_on_conflict(uint32_t node_id) {
    assert(node_id < d_num_nodes);
    collect_changed(node_id);
    for (uint32_t child_id : d_children[node_id]) {
        collect_changed(child_id);
    }
    // Conflict may surface before consume_state() runs, so include nodes with
    // pending updates or queued work to keep rollback complete.
    for (uint32_t id = 0; id < d_num_nodes; ++id) {
        if (d_changed[id] || d_enqueued[id] ||
            !is_unchanged(d_domains[id].pending_state())) {
            collect_changed(id);
        }
    }
}

Result Propagator::consume_domain_state(uint32_t node_id) {
    FeasibleDomain& domain = d_domains[node_id];
    Result state = domain.consume_state();
    if (is_unchanged(state)) {
        return Result::UNCHANGED;
    }
    collect_changed(node_id);
    if (is_updated(state)) {
        domain.stage_last_interval_size();
    }
    return state;
}

void Propagator::propagate_enqueue(uint32_t node_id) {
    assert(node_id < d_num_nodes);
    if (!d_enqueued[node_id]) {
        d_queue.push(node_id);
        d_enqueued[node_id] = true;
    }
}

void Propagator::propagate_node(uint32_t node_id) {
    const node::Kind k = d_nodes[node_id].kind();
    // CONSTANT means symbol, VALUE means constant value
    if (k == node::Kind::CONSTANT || k == node::Kind::VALUE)
        return;

    FeasibleDomain& output = d_domains[node_id];
    std::vector<FeasibleDomain*> children;
    for (uint32_t child_id : d_children[node_id]) {
        children.emplace_back(&d_domains[child_id]);
    }

    // Feasible Domain Propagation.
    Result res = Result::UPDATED;
    while (is_updated(res)) {
#if 0
        std::stringstream ss;
        ss << "FDP Propagating Node " << k << " with children:\n";
        for (uint32_t child_id : d_children[node_id]) {
            ss << d_domains[child_id].to_string() << "\n";
        }
        ss << "Output before:\n"
           << output.to_string() << "\n";
#endif

#define DISPATCH(kind, name)                                           \
    case node::Kind::kind: {                                           \
        res = Fdp##name##Operator(d_stats, children, &output).apply(); \
        break;                                                         \
    }
        switch (k) {
            // Shifting
            DISPATCH(BV_SHL, LeftShift)
            DISPATCH(BV_SHR, RightShift)
            DISPATCH(BV_ASHR, ArithRightShift)

            // Unsigned Comparison
            DISPATCH(BV_ULT, LessThan)

            // Signed Comparison
            DISPATCH(BV_SLT, SignedLessThan)

            // Logic.
            DISPATCH(XOR, Xor)
            DISPATCH(BV_XOR, Xor)
            DISPATCH(AND, And)
            DISPATCH(BV_AND, And)
            DISPATCH(EQUAL, Eq)
            // DISPATCH(BV_COMP, Eq)
            DISPATCH(NOT, Not)
            DISPATCH(BV_NOT, Not)

            // Arithmetic
            DISPATCH(BV_ADD, Add)
            DISPATCH(BV_MUL, Mul)
            DISPATCH(BV_UREM, Rem)
            DISPATCH(BV_UDIV, Div)

            // OTHER
            DISPATCH(ITE, Ite)
            // DISPATCH(BV_EXTRACT, Extract)
            DISPATCH(BV_CONCAT, Concat)

            case node::Kind::BV_EXTRACT: {
                uint64_t low = d_nodes[node_id].index(1);
                uint64_t high = d_nodes[node_id].index(0);
                res = FdpExtractOperator(d_stats, children, &output, low, high).apply();
                break;
            }

            default: {
                std::cout << "Propagator::propagate_node: not handled " << k << "\n";
                // assert(false);
                exit(-1);
                return;
            }
        }
#undef DISPATCH

        if (is_conflict(res)) {
            set_conflict();
            return;
        }

#if 0
        if (is_changed(res) || is_updated(res)) {
            ss << "After Apply: " << k << " with children:\n";
            for (uint32_t child_id : d_children[node_id]) {
                ss << d_domains[child_id].to_string() << "\n";
            }
            ss << "Output before:\n"
               << output.to_string() << "\n";
        }
#endif

        // Fixed bits => intervals
        res |= tighten_each_interval(children, &output);
        if (is_conflict(res)) {
            set_conflict();
            return;
        }

        // Intervals => fixed bits
        res |= tighten_each_fixed_bits(children, &output);
        if (is_conflict(res)) {
            set_conflict();
            return;
        }

#if 0
        if (is_changed(res) || is_updated(res)) {
            ss << "After Tighten: " << k << " with children:\n";
            for (uint32_t child_id : d_children[node_id]) {
                ss << d_domains[child_id].to_string() << "\n";
            }
            ss << "Output after:\n"
               << output.to_string() << "\n";
            ss << "----------------------------------------\n\n";

            std::cout << ss.str();
        }
#endif
    }
}

void Propagator::feasible_domain_propagate() {
    while (!d_queue.empty()) {
        uint32_t node_id = d_queue.top();
        d_queue.pop();
        d_enqueued[node_id] = false;
        ++d_propagate_cnt;
        propagate_node(node_id);

        if (d_conflict) {
            collect_changed_on_conflict(node_id);
            return;
        }

        Result node_state = consume_domain_state(node_id);
        // only state UPDATED can propagate further
        // to prevent interval slow convergence
        if (is_updated(node_state)) {
            for (uint32_t parent_id : d_parents[node_id]) {
                propagate_enqueue(parent_id);
            }
        }

        for (uint32_t child_id : d_children[node_id]) {
            // // Leaf children (symbols/values) never propagate further down the
            // // DAG, so handle them inline to reduce queue churn.
            // const node::Kind child_kind = d_nodes[child_id].kind();
            // if (child_kind == node::Kind::CONSTANT ||
            //     child_kind == node::Kind::VALUE) {
            //     Result child_state = d_domains[child_id].pending_state();
            //     if (is_changed(child_state)) {
            //         child_state = consume_domain_state(child_id);
            //         if (is_updated(child_state)) {
            //             for (uint32_t parent_id : d_parents[child_id]) {
            //                 if (parent_id != node_id) {
            //                     propagate_enqueue(parent_id);
            //                 }
            //             }
            //         }
            //     }
            //     continue;
            // }

            // Result child_state = consume_domain_state(child_id);
            Result child_state = d_domains[child_id].pending_state();
            if (is_updated(child_state)) {
                propagate_enqueue(child_id);
            }
            else if (is_changed(child_state)) {
                // even if child is only changed, still need to collect it
                collect_changed(child_id);
            }
        }
    }
    assert(d_queue.empty());
    // std::cout << "feasible_domain_propagate: propagation queue empty" << std::endl;
    // clean up when implement bounded propagation
    // while (!d_queue.empty()) {
    //     uint32_t node_id = d_queue.top();
    //     d_queue.pop();
    //     d_enqueued[node_id] = false;
    // }
}

// set initial constant values
void Propagator::set_constant_domains() {
    for (uint32_t node_id : d_constants) {
        FeasibleDomain& domain = d_domains[node_id];
        const Node& node = d_nodes[node_id];
        const Type& type = node.type();
        Result res = Result::UNCHANGED;
        if (type.is_bv()) {
            const BitVector& value = node.value<BitVector>();
            res = domain.set_constant(value);
        }
        else if (type.is_bool()) {
            const bool value = node.value<bool>();
            res = domain.set_constant(value);
        }
        else {
            std::cerr << "Propagator::set_constant_domains: unsupported type "
                      << type << " for node " << node << "\n";
            assert(false && "Unsupported type for constant domain");
        }
        if (is_conflict(res)) {
            set_conflict();
            return;
        }
        // if (is_changed(res)) {
        //     // VALUE nodes do not propagate to children, so we can directly wake
        //     // up their parents and avoid pushing them onto the queue.
        //     consume_domain_state(node_id);
        //     for (uint32_t parent_id : d_parents[node_id]) {
        //         propagate_enqueue(parent_id);
        //     }
        // }
        if (is_changed(res)) {
            propagate_enqueue(node_id);
        }
    }
}

void Propagator::run_full_propagation() {
    set_constant_domains();
    if (d_conflict)
        return;
    // set top true for roots
    for (uint32_t root_idx : d_roots) {
        FeasibleDomain& domain = d_domains[root_idx];
        Result res = domain.set_constant(true);
        if (is_conflict(res)) {
            set_conflict();
            return;
        }
        if (is_changed(res)) {
            propagate_enqueue(root_idx);
        }
    }
    feasible_domain_propagate();
}

void Propagator::collect_propagated_frontier() {
    d_propagated_frontier.clear();

    // for (uint32_t root_idx : d_roots) {
    //     d_propagated_frontier.emplace_back(root_idx, true);
    // }
    // return;
    std::queue<uint32_t> queue;
    std::unordered_set<uint32_t> visited(d_num_bool_nodes);
    auto enqueue = [&](uint32_t id) {
        if (visited.find(id) == visited.end()) {
            queue.push(id);
            visited.insert(id);
        }
    };
    for (uint32_t root_idx : d_roots) {
        enqueue(root_idx);
        // std::cerr << "root idx: " << root_idx << std::endl;
        // std::cerr << "root: " << d_nodes[root_idx] << std::endl;
        //  d_propagated_frontier.emplace_back(root_idx, true);
    }
    // return;
    while (!queue.empty()) {
        uint32_t node_id = queue.front();
        // std::cerr << "visiting node id: " << node_id << std::endl;
        // std::cerr << "visiting node: " << d_nodes[node_id] << std::endl;
        queue.pop();
        Node& node = d_nodes[node_id];
        assert(node.type().is_bool());
        const node::Kind k = node.kind();
        if (k == node::Kind::VALUE)
            continue;
        if (k == node::Kind::CONSTANT) {
            // std::cerr << "d_propagated_frontier node id: " << node_id << std::endl;
            // std::cerr << "d_propagated_frontier node: " << d_nodes[node_id] << std::endl;
            d_propagated_frontier.emplace_back(node_id, d_domains[node_id].get_value(0));
            continue;
        }
        FeasibleDomain& output = d_domains[node_id];
        assert(output.is_totally_fixed());
        std::vector<FeasibleDomain*> children;
        bool is_frontier = false;
        for (uint32_t child_id : d_children[node_id]) {
            // if any child is not boolean, mark as frontier
            if (!d_nodes[child_id].type().is_bool()) {
                is_frontier = true;
                break;
            }
            children.emplace_back(&d_domains[child_id]);
        }
        if (is_frontier) {
            // std::cerr << "d_propagated_frontier node id: " << node_id << std::endl;
            // std::cerr << "d_propagated_frontier node: " << d_nodes[node_id] << std::endl;
            d_propagated_frontier.emplace_back(node_id, output.get_value(0));
            continue;
        }

        std::vector<uint32_t> child_ids;
        bool res(false);
#define DISPATCH(kind, name)                                                      \
    case node::Kind::kind: {                                                      \
        res = Fdp##name##Operator(d_stats, children, &output).implied_by(child_ids); \
        break;                                                                    \
    }
        switch (k) {
            // // Logic.
            DISPATCH(XOR, Xor)
            // in boolean operators, eq is same as xor
            DISPATCH(EQUAL, Xor)
            DISPATCH(AND, And)
            DISPATCH(NOT, Not)

            // // OTHER
            DISPATCH(ITE, Ite)

            default: {
                std::cerr << "Propagator::collect_propagated_frontier: not handled "
                          << k << "\n";
                assert(false && "Unsupported operator for frontier collection");
            }
        }
#undef DISPATCH
        // // display result of implied_by with domain information
        // std::cerr << "implied_by result for node " << node_id << " (" << k << "): " << " is " << res << " with output domain: " << output.to_string() << std::endl;
        // std::cerr << "child ids and domains: ";
        // for (uint32_t child_id : child_ids) {
        //     std::cerr << "child id: " << child_id << " domain: " << d_domains[child_id].to_string() << "; ";
        // }
        // std::cerr << std::endl;

        if (res) {
            if (!child_ids.empty()) {
                for (uint32_t idx : child_ids) {
                    uint32_t child_id = d_children[node_id][idx];
                    enqueue(child_id);
                }
            }
            else {
                // std::cerr << "d_propagated_frontier node id: " << node_id << " value: " << output.get_value(0) << std::endl;
                // std::cerr << "d_propagated_frontier node: " << d_nodes[node_id] << std::endl;
                d_propagated_frontier.emplace_back(node_id, output.get_value(0));
            }
        }
    }
}

void Propagator::mark_encoding_domains() {
    // Traverse the graph and collect candidate nodes for encoding:
    // - Symbols are fully encoded and all fixed bits are kept.
    // - If the delta between the fixed-bit domain and its refinement exceeds
    //   the threshold, encode the refinement as well.
    // - Internal non-Boolean nodes use stricter thresholds.
    // - Boolean nodes at or above the frontier are skipped.
    // Sort marked nodes by priority.

    std::unique_ptr<bool[]> is_relevant = std::make_unique<bool[]>(d_num_nodes);
    std::fill_n(is_relevant.get(), d_num_nodes, false);

    for (const auto& [node_id, value] : d_propagated_frontier) {
    // for (const auto& [node_id, value] : d_reduced_frontier) {
        is_relevant[node_id] = true;
    }

    // mark relevant nodes by topo order
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        if (is_relevant[i]) {
            for (uint32_t child_id : d_children[i]) {
                is_relevant[child_id] = true;
            }
        }
        else if (d_nodes[i].is_const()) {
            is_relevant[i] = true;
        }
    }

    struct MarkedNodeInfo {
        uint32_t d_topo_id;
        FeasibleDomain d_domain;
        BitVector d_interval_size;
        BitVector d_domain_size;
        // BitVector reduce_ratio;
        bool d_is_symbol;
        MarkedNodeInfo(uint32_t id,
                       const FeasibleDomain& dom,
                       const BitVector& interval,
                       const BitVector& size,
                       bool symbol)
            : d_topo_id(id),
              d_domain(dom),
              d_interval_size(interval),
              d_domain_size(size),
              d_is_symbol(symbol) {}

        bool operator<(const MarkedNodeInfo& other) const {
            if (d_is_symbol != other.d_is_symbol)
                return d_is_symbol;  // Symbols have higher priority.
            // int cmp = reduce_ratio.compare(other.reduce_ratio);
            // if (cmp != 0)
            //     return cmp > 0; // Higher reduction ratio first
            return d_topo_id > other.d_topo_id;  // Higher topo_id first.
        }
    };


    std::vector<MarkedNodeInfo> marked_infos;
    std::vector<uint32_t> marked_ids;

    for (uint32_t id = 0; id < d_num_nodes; ++id) {
    // for (uint32_t id : d_changed_nodes) {
        if (is_relevant[id]) {
            for (uint32_t child_id : d_children[id]) {
                is_relevant[child_id] = true;
            }
        }
        else if (d_nodes[id].is_const()) {
            is_relevant[id] = true;
        }
        if (!is_relevant[id]) {
            continue;
        }
        const Node& node = d_nodes[id];
        const node::Kind k = node.kind();
        if (k == node::Kind::VALUE)
            continue;
        FeasibleDomain& domain = d_domains[id];
        if (domain.is_full())
            continue;
        BitVector interval_size = domain.calc_interval_size();
        BitVector domain_size = domain.calc_domain_size();
        if (k == node::Kind::CONSTANT) {
            // Candidate is a symbol.
            marked_ids.emplace_back(marked_infos.size());
            marked_infos.emplace_back(id, domain, interval_size, domain_size, true);
            // marked_infos.emplace_back(i, domain, domain_size, reduce_ratio, true);
        }
        else if (node.type().is_bv()) {
            // Only consider non-Boolean interior nodes for encoding.
            EncodingMark mark = calc_encoding_mark(
                domain.width(),
                domain.count_fixed(),
                interval_size,
                domain_size,
                false);
            if (mark != EncodingMark::NONE) {
                marked_ids.emplace_back(marked_infos.size());
                marked_infos.emplace_back(id, domain, interval_size, domain_size, false);
                // marked_infos.emplace_back(i, domain, domain_size, reduce_ratio, false);
            }
        }

        // // reset changed nodes
        // d_changed[id] = false;
        // d_domains[id].reset();
    }

    d_propagate_cnt = 0;
    // reset changed nodes
    for (uint32_t id : d_changed_nodes) {
        d_changed[id] = false;
        d_domains[id].reset();
    }

    d_changed_nodes.clear();

    // // brute-force reset
    // for (uint32_t id = 0; id < d_num_nodes; ++id) {
    //     d_changed[id] = false;
    //     d_domains[id].reset();
    // }
    // d_changed_nodes.clear();


    std::sort(marked_ids.begin(), marked_ids.end(), [&marked_infos](uint32_t lhs, uint32_t rhs) {
        return marked_infos[lhs] < marked_infos[rhs];
    });

    set_constant_domains();
    if (d_conflict)
        return;

    // feasible_domain_propagate();
    // if (d_conflict)
    //     return;

    // Process marked nodes by quality:
    // - Symbols are always handled before interior nodes.
    // - Higher reduction ratios are preferred.
    // - Skip nodes whose improvement is below the threshold.

    // Encode symbols first.
    uint32_t idx = 0, sz = marked_infos.size();
    while (idx < sz) {
        uint32_t mark_id = marked_ids[idx];
        const MarkedNodeInfo& info = marked_infos[mark_id];
        if (!info.d_is_symbol)
            break;
        ++idx;
        uint32_t topo_id = info.d_topo_id;
        const FeasibleDomain& domain = info.d_domain;
        EncodingMark mark = calc_encoding_mark(
            domain.width(),
            domain.count_fixed(),
            info.d_interval_size,
            info.d_domain_size,
            true);
        d_marks[topo_id] = mark;
        if (mark != EncodingMark::NONE) {
            // // display marked and type
            // std::cout << "Marking node " << topo_id << " (" << d_nodes[topo_id] << ") as symbol with domain:\n"
            //           << domain.to_string() << ", mark: " << signed(mark) << "\n";
            Result res = d_domains[topo_id].overwrite(domain, mark);
            if (is_conflict(res)) {
                set_conflict();
                return;
            }
            propagate_enqueue(topo_id);
        }
    }
    feasible_domain_propagate();
    if (d_conflict)
        return;

    // // encode symbols only
    // d_base_propagate_cnt = d_propagate_cnt;
    // return;

    // Encode interior nodes next.
    while (idx < sz) {
        uint32_t mark_id = marked_ids[idx];
        const MarkedNodeInfo& info = marked_infos[mark_id];
        ++idx;
        uint32_t topo_id = info.d_topo_id;
        const FeasibleDomain& domain = info.d_domain;
        const FeasibleDomain& current_domain = d_domains[topo_id];
        EncodingMark mark = calc_encoding_mark_diff(
            domain.width(),
            current_domain.count_fixed(),
            current_domain.calc_interval_size(),
            current_domain.calc_domain_size(),
            domain.count_fixed(),
            info.d_interval_size,
            info.d_domain_size);
        d_marks[topo_id] = mark;
        if (mark != EncodingMark::NONE && mark != EncodingMark::IMPLIED) {
            // // display marked and type
            // std::cout << "Marking node " << topo_id << " (" << d_nodes[topo_id] << ") as interior with domain:\n"
            //             << domain.to_string() << ", mark: " << signed(mark) << "\n";
            // // display parameters to calc_encoding_mark_diff
            // std::cout << "Current domain fixed bits: " << current_domain.count_fixed() << ", interval size: " << current_domain.calc_interval_size()
            //           << ", domain size: " << current_domain.calc_domain_size() << "\n";
            // std::cout << "Proposed domain fixed bits: " << domain.count_fixed() << ", interval size: " << info.d_interval_size
            //           << ", domain size: " << info.d_domain_size << "\n";
            Result res = d_domains[topo_id].overwrite(domain, mark);
            if (is_conflict(res)) {
                set_conflict();
                return;
            }
            propagate_enqueue(topo_id);
            feasible_domain_propagate();
            if (d_conflict)
                return;
        }
    }
    d_base_propagate_cnt = d_propagate_cnt;
}

void Propagator::remove_redundant_frontier_nodes() {
    if (d_propagated_frontier.empty())
        return;

    d_reduced_frontier.clear();

    // // brute force reduced frontier
    // for (auto [node_id, value] : d_propagated_frontier) {
    //     // Push: add the frontier assignment and propagate its effect.
    //     d_reduced_frontier.emplace_back(node_id, value);
    // }
    // return;

    FdPrefixArray& history = *d_fd_prefix;
    struct FrontierNodeInfo {
        uint32_t d_topo_id;
        bool d_value;
        uint64_t d_fixed_cnt;
        // TODO: add a distance-to-top metric.
        // Nodes with less information (fewer fixed bits) are processed later.
        bool operator<(const FrontierNodeInfo& other) const {
            if (d_fixed_cnt != other.d_fixed_cnt)
                return d_fixed_cnt > other.d_fixed_cnt;  // More fixed bits first.
            return d_topo_id > other.d_topo_id;          // Higher topo_id first.
        }
        FrontierNodeInfo() = default;
        FrontierNodeInfo(uint32_t id, bool val, uint64_t cnt)
            : d_topo_id(id), d_value(val), d_fixed_cnt(cnt) {}
    };

    std::vector<FrontierNodeInfo> frontier_infos;
    std::vector<uint32_t> frontier_ids;
    frontier_infos.reserve(d_propagated_frontier.size());
    frontier_ids.reserve(d_propagated_frontier.size());
    {
        for (auto [node_id, value] : d_propagated_frontier) {
            // Push: add the frontier assignment and propagate its effect.
            FeasibleDomain& domain = d_domains[node_id];
            Result res = domain.set_constant(value);
            if (is_conflict(res)) {
                set_conflict();
                return;
            }

            if (is_unchanged(res))
                continue;

            propagate_enqueue(node_id);
            feasible_domain_propagate();

            if (d_conflict)
                return;

            uint64_t fixed_cnt = 0;

            for (uint32_t changed_id : d_changed_nodes) {
                d_changed[changed_id] = false;
                FeasibleDomain& domain = d_domains[changed_id];
                fixed_cnt += static_cast<uint64_t>(domain.count_fixed());
                const FeasibleDomain* history_domain =
                    history.get_ptr(changed_id, 0);
                if (history_domain != nullptr) {
                    d_domains[changed_id].copy_from(*history_domain);
                    fixed_cnt -= static_cast<uint64_t>(history_domain->count_fixed());
                }
                else {
                    d_domains[changed_id].reset();
                }
            }

            // std::cout << "Frontier node " << node_id << " (" << d_nodes[node_id] << ") = " << value
            //           << " fixed bits changed: " << fixed_cnt << "\n";
            // std::cout << "changed nodes count: " << d_changed_nodes.size() << "\n";
            d_changed_nodes.clear();

            frontier_ids.emplace_back(frontier_infos.size());
            frontier_infos.emplace_back(node_id, value, fixed_cnt);
        }

        std::sort(frontier_ids.begin(), frontier_ids.end(), [&frontier_infos](uint32_t lhs, uint32_t rhs) {
            return frontier_infos[lhs] < frontier_infos[rhs];
        });
    }

    d_propagate_cnt = 0;
    FdPrefixArray prefix_history(d_num_nodes);
    uint32_t num_frontiers = frontier_ids.size();
    {
        // Initialize prefix history level 0 using the staged domains.
        // Each level maintains its own set of changed nodes in the persistent array.
        const auto& base_changed = history.level0_changed_nodes();
        // std::cout << "Number of base changed nodes to restore: " << base_changed.size() << "\n";
        for (uint32_t id : base_changed) {
            const FeasibleDomain* history_domain = history.get_ptr(id, 0);
            assert(history_domain != nullptr);
            prefix_history.update(0, id, *history_domain);
        }

        // Level 0 is fully stored; reset tracking for higher levels.
        prefix_history.finalize_level_zero();

        uint32_t cur_size = 0;
        for (uint32_t i = 0; i < num_frontiers; ++i) {
            uint32_t idx = frontier_ids[i];
            uint32_t node_id = frontier_infos[idx].d_topo_id;
            FeasibleDomain& domain = d_domains[node_id];
            // std::cout << "\n";
            // std::cout << "Testing frontier " << i << ": " << node_id << " (" << d_nodes[node_id] << ") = " << frontier_infos[idx].d_value << "\n";
            Result res = domain.set_constant(frontier_infos[idx].d_value);
            if (is_conflict(res)) {
                set_conflict();
                return;
            }
            if (is_unchanged(res)) {
                // std::cout << "removed frontier " << i << ": " << node_id << " (" << d_nodes[node_id] << ") = " << frontier_infos[idx].d_value << "\n";
                continue;
            }
            // Use contiguous level ids for kept frontiers to avoid gaps in the
            // prefix history (which would miss nodes when queried later).

            if (cur_size < i)
                frontier_ids[cur_size] = frontier_ids[i];
            ++cur_size;
            propagate_enqueue(node_id);
            feasible_domain_propagate();
            if (d_conflict)
                return;
            for (uint32_t id : d_changed_nodes) {
                prefix_history.update(cur_size, id, d_domains[id]);
                d_changed[id] = false;
            }
            d_changed_nodes.clear();
        }
        num_frontiers = cur_size;
        frontier_ids.resize(num_frontiers);
        CHECK_ROOTS_FIXED_TRUE();

        // display all frontiers
        // std::cout << "Final kept frontiers (" << num_frontiers << "):\n";
        // for (uint32_t i = 0; i < num_frontiers; ++i) {
        //     uint32_t idx = frontier_ids[i];
        //     uint32_t node_id = frontier_infos[idx].d_topo_id;
        //     bool value = frontier_infos[idx].d_value;
        //     std::cout << "kept frontier " << i << ": " << node_id << " (" << d_nodes[node_id] << ") = " << value << "\n";
        // }
        // std::cout << "\n";
    }
    d_frontier_propagate_cnt = d_propagate_cnt;
    if (num_frontiers == 0)
        return;
    // std::cout << "d_propagated_frontier size: " << d_propagated_frontier.size() << "\n";
    // std::cout << "d_frontier_propagate_cnt: " << d_frontier_propagate_cnt << "\n";
    const uint64_t estimated_cost =
        static_cast<uint64_t>(d_propagated_frontier.size()) * d_frontier_propagate_cnt;
#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[remove_redundant_frontier_nodes] estimated cost: " << estimated_cost << "\n";
#endif
    if (estimated_cost > REMOVE_SKIP_THRESHOLD) {
        d_reduced_frontier.reserve(num_frontiers);
        for (uint32_t idx : frontier_ids) {
            const auto& info = frontier_infos[idx];
            d_reduced_frontier.emplace_back(info.d_topo_id, info.d_value);
        }
        return;
    }

    // Reset current domains back to the staged level-0 snapshot.
    // Only nodes touched while building the prefix history need restoring.
    const auto affected_prefix = prefix_history.changed_nodes(num_frontiers);
    // std::cout << "Number of affected nodes to restore: " << affected_prefix.size() << "\n";
    for (uint32_t id : affected_prefix) {
        const FeasibleDomain* history_domain = history.get_ptr(id, 0);
        if (history_domain != nullptr) {
            // .overwrite(*history_domain, EncodingMark::BOTH);
            d_domains[id].copy_from(*history_domain);
        }
        else {
            d_domains[id].reset();
        }
    }

    // Iterate from least informative to most informative frontiers.
    // `history` stores suffix domains for frontiers that must remain.
    // `prefix_history` stores the cumulative effect of kept frontiers on the left.
    uint32_t suffix_level = 0;
    bool top_true_failed = false;
    for (int32_t i = num_frontiers - 1; i >= 0; --i) {
        // uint32_t lvl = i + 1;
        // prefix[i]   = encoding + [0, i)
        // suffix[lvl] = encoding + [0, lvl)
        uint32_t pre_lvl = i;
        uint32_t idx = frontier_ids[i];
        uint32_t node_id = frontier_infos[idx].d_topo_id;
        const bool value = frontier_infos[idx].d_value;
        FeasibleDomain& domain = d_domains[node_id];
        if (domain.is_totally_fixed()) {
            if (domain.get_value(0) == value) {
                // can be removed directly
                continue;
            }
            else {
                set_conflict();
                return;
            }
        }

        if (!top_true_failed) {
            // At this point the domain reflects the suffix [i+1, end); merge prefix[i-1].
            const auto& prefix_nodes = prefix_history.changed_nodes(pre_lvl);
            // std::cout << "Number of prefix nodes to merge at level " << pre_lvl << ": " << prefix_nodes.size() << "\n";
            for (uint32_t idy : prefix_nodes) {
                const FeasibleDomain* history_domain =
                    prefix_history.get_ptr(idy, pre_lvl);
                if (history_domain != nullptr) {
                    Result merge_res = d_domains[idy].merge(*history_domain);
                    if (is_conflict(merge_res)) {
                        set_conflict();
                        return;
                    }
                    if (is_changed(merge_res)) {
                        propagate_enqueue(idy);
                    }
                }
            }

            // 2908
            // uint32_t temp_id = 2910; // choose a node to display
            // std::cout << "\n";
            // std::cout << "suffix: " << temp_id << " (" << d_nodes[temp_id] << "): " << d_domains[temp_id].to_string() << "\n";
            // std::cout << "prefix: " << temp_id << " (" << d_nodes[temp_id] << "): " << prefix_history.get_ptr(temp_id, pre_lvl)->to_string() << "\n";

            // const uint32_t temp_root_id = 199;
            // std::cout << "root suffix: " << temp_root_id << " (" << d_nodes[temp_root_id] << "): " << d_domains[temp_root_id].to_string() << "\n";
            // std::cout << "root prefix: " << temp_root_id << " (" << d_nodes[temp_root_id] << "): " << prefix_history.get_ptr(temp_root_id, pre_lvl)->to_string() << "\n";

            // std::cout << "merged: " << temp_id << " (" << d_nodes[temp_id] << "): " << d_domains[temp_id].to_string() << "\n";

            feasible_domain_propagate();
            if (d_conflict)
                return;
            // Remove the frontier if all roots stay fixed to true.

            // std::cout << "root top true: " << temp_root_id << " (" << d_nodes[temp_root_id] << "): " << d_domains[temp_root_id].to_string() << "\n";

            // std::cout << "after propagate: " << temp_id << " (" << d_nodes[temp_id] << "): " << d_domains[temp_id].to_string() << "\n";

            bool can_remove = true;
            for (uint32_t root_idx : d_roots) {
                if (!d_domains[root_idx].is_fixed_one(0)) {
                    can_remove = false;
                    break;
                }
            }

            // std::cout << "Checking frontier node " << node_id
            //           << " (" << d_nodes[node_id] << ")"
            //           << " with value " << (value ? "true" : "false")
            //           << ", " << (can_remove ? "can be removed" : "must stay") << std::endl;

            Result tmp_res = domain.set_constant(value);
            if (is_conflict(tmp_res)) {
                set_conflict();
                return;
            }
            if (is_changed(tmp_res)) {
                propagate_enqueue(node_id);
                feasible_domain_propagate();
                if (d_conflict) {
                    set_conflict();
                    return;
                }
                if ([&]() {
                        for (uint32_t root_idx : d_roots) {
                            if (!d_domains[root_idx].is_fixed_one(0)) {
                                return true;
                            }
                        }
                        return false;
                    }()) {
                    top_true_failed = true;
                }
            }
            // // // display all domains
            // std::cout << "Domains after checking frontier node " << node_id << ":\n";
            // // for (uint32_t j = 0; j < d_num_nodes; ++j) {
            // //     std::cout << "Node " << j << " (" << d_nodes[j] << "): " << d_domains[j].to_string() << "\n";
            // // }

            // std::cout << "Node " << temp_id << " (" << d_nodes[temp_id] << "): " << d_domains[temp_id].to_string() << "\n";

            // CHECK_ROOTS_FIXED_TRUE();

            // Restore domains back to the suffix level.
            for (uint32_t changed_id : d_changed_nodes) {
                d_changed[changed_id] = false;
                const FeasibleDomain* history_domain =
                    history.get_ptr(changed_id, suffix_level);
                if (history_domain != nullptr) {
                    d_domains[changed_id].copy_from(*history_domain);
                }
                else {
                    d_domains[changed_id].reset();
                }
            }
            // std::cout << "restoration " << temp_id << " (" << d_nodes[temp_id] << "): " << d_domains[temp_id].to_string() << "\n";
            d_changed_nodes.clear();
            if (can_remove)
                continue;
        }

        // Keep this frontier and append its effect to the suffix history.
        Result res = domain.set_constant(value);
        if (is_conflict(res)) {
            set_conflict();
            return;
        }
        if (is_changed(res)) {
            propagate_enqueue(node_id);
            feasible_domain_propagate();
        }
        if (d_conflict)
            return;
        ++suffix_level;
        for (uint32_t id : d_changed_nodes) {
            history.update(suffix_level, id, d_domains[id]);
            d_changed[id] = false;
        }
        d_changed_nodes.clear();
        d_reduced_frontier.emplace_back(node_id, value);
    }
    // feasible_domain_propagate();
    // CHECK_ROOTS_FIXED_TRUE();

    std::reverse(d_reduced_frontier.begin(), d_reduced_frontier.end());
}

void Propagator::apply_frontier_assignments() {
    for (const auto& [node_id, value] : d_propagated_frontier) {
        Result res = d_domains[node_id].set_constant(value);
        if (is_conflict(res)) {
            set_conflict();
            return;
        }
        if (is_changed(res)) {
            propagate_enqueue(node_id);
        }
    }
    feasible_domain_propagate();
}

void Propagator::rebuild_dag_after_encoding() {
    d_final_roots.clear();
    d_final_nodes.clear();
    d_final_ids.clear();
    std::unique_ptr<bool[]> is_relevant = std::make_unique<bool[]>(d_num_nodes);
    std::fill_n(is_relevant.get(), d_num_nodes, false);

    // for (auto [node_id, value] : d_reduced_frontier) {
    //     Node root = d_nodes[node_id];
    //     if (!value)
    //         root = d_nm.mk_node(node::Kind::NOT, {root});
    //     d_final_roots.emplace_back(root);
    // }
    // return;

    // Seed relevance with the final reduced frontier, plus any nodes that will
    // contribute encoding/equality assertions from the level-0 snapshot.
    for (const auto& [node_id, value] : d_propagated_frontier) {
    // for (const auto& [node_id, value] : d_reduced_frontier) {
        is_relevant[node_id] = true;
    }

    // for (const auto& [node_id, value] : d_propagated_frontier) {
    //     is_relevant[node_id] = true;
    // }

    // const auto base_changed = history.level0_changed_nodes();
    // for (uint32_t node_id : base_changed) {
    //     const FeasibleDomain* dom_ptr = history.get_ptr(node_id, 0);
    //     if (dom_ptr == nullptr) {
    //         continue;
    //     }
    //     if (dom_ptr->is_totally_fixed()) {
    //         is_relevant[node_id] = true;
    //     }
    //     else if (d_marks[node_id] != EncodingMark::NONE && d_marks[node_id] != EncodingMark::IMPLIED) {
    //         is_relevant[node_id] = true;
    //     }
    // }


    // mark relevant nodes by topo order
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        if (is_relevant[i]) {
            for (uint32_t child_id : d_children[i]) {
                is_relevant[child_id] = true;
            }
        }
        else if (d_nodes[i].is_const()) {
            is_relevant[i] = true;
        }
    }

    assert(d_fd_prefix && "fdp prefix history missing when rebuilding DAG");
    FdPrefixArray& history = *d_fd_prefix;

    d_final_ids.resize(d_num_nodes, UINT32_MAX);
    // Rebuild the DAG in reverse topological order.
    for (int32_t i = static_cast<int32_t>(d_num_nodes) - 1; i >= 0; --i) {
        if (!is_relevant[i]) {
            d_final_ids[i] = UINT32_MAX;
            d_marks[i] = EncodingMark::REMOVED;
            // std::cout << "Removing node " << i << " (" << d_nodes[i] << ") from final DAG\n";
            continue;
        }
        const FeasibleDomain* dom_ptr = history.get_ptr(i, 0);
        // FeasibleDomain& domain = d_domains[i];
        const Type type = d_nodes[i].type();
        d_final_ids[i] = d_final_nodes.size();
        Node new_node;
        Node cur = d_nodes[i];

        if (cur.is_variable() || cur.is_value()) {
            new_node = cur;
        }
        else {
            if (cur.is_const()) {
                new_node = cur;
            }
            else {
                // interier nodes
                std::vector<Node> new_children;
                for (uint32_t child_id : d_children[i]) {
                    uint32_t final_id = d_final_ids[child_id];
                    assert(final_id != UINT32_MAX);
                    new_children.emplace_back(d_final_nodes[final_id]);
                }
                new_node = d_nm.mk_node(cur.kind(), new_children, cur.indices());
                new_node = d_rewriter.rewrite(new_node);
            }

            if (dom_ptr != nullptr) {
                // if (d_marks[i] != EncodingMark::NONE)
                if (!dom_ptr->is_totally_fixed()) {
                    encode_marked_node(new_node, *dom_ptr, d_marks[i]);
                }
                else if (type.is_bv() || cur.is_const()) {
                    // Replace totally fixed nodes with constants.
                    if (type.is_bv()) {
                        cur = d_nm.mk_value(dom_ptr->get_fixed_value());
                    }
                    else if (type.is_bool()) {
                        cur = d_nm.mk_value(dom_ptr->get_value(0));
                    }
                    else {
                        assert(false && "Unsupported type for totally fixed domain");
                    }
                    // // Add equality assertion between the original node and the constant.
                    // Node assertion = d_nm.mk_node(node::Kind::EQUAL, {new_node, cur});
                    // assertion = d_rewriter.rewrite(assertion);
                    // // Drop trivial assertions that rewrite to 'true'.
                    // if (!assertion.is_value() || !assertion.value<bool>()) {
                    //     // d_final_roots.emplace_back(assertion);
                    //     d_encoded_nodes.emplace_back(assertion);
                    // }
                    // if (d_marks[i] == EncodingMark::NONE) {
                    //     std::cout << "Warning: node " << i << " (" << d_nodes[i] << ") is totally fixed but has encoding mark NONE\n";
                    // }
                    if (d_marks[i] != EncodingMark::IMPLIED) {
                        // std::cout << "Warning: node " << i << " (" << d_nodes[i] << ") is totally fixed but has encoding mark " << static_cast<int>(d_marks[i]) << " instead of IMPLIED\n";
                        // assert(d_marks[i] == EncodingMark::BITS && "Only bit encoding expected for totally fixed nodes");
                        // Add equality assertion between the original node and the constant.
                        // std::cout << "Adding assertion for totally fixed node " << i << " (" << d_nodes[i] << "): " << cur << "\n";
                        Node assertion = d_nm.mk_node(node::Kind::EQUAL, {new_node, cur});
                        assertion = d_rewriter.rewrite(assertion);
                        // Drop trivial assertions that rewrite to 'true'.
                        if (!assertion.is_value() || !assertion.value<bool>()) {
                            // d_final_roots.emplace_back(assertion);
                            // display assertion
                            // std::cout << "Adding assertion for totally fixed node " << i << " (" << d_nodes[i] << "): " << assertion << "\n";
                            d_encoded_nodes.emplace_back(assertion);
                        }
                    }
                    new_node = cur;
                }
            }
        }

        d_final_nodes.emplace_back(new_node);
    }

    // d_reduced_frontier
    for (auto [node_id, value] : d_reduced_frontier) {
        uint32_t final_id = d_final_ids[node_id];
        assert(final_id != UINT32_MAX);
        Node root = d_final_nodes[final_id];
        if (!value)
            root = d_nm.mk_node(node::Kind::NOT, {root});
        if (!root.is_value() || !root.value<bool>()) {
            d_final_roots.emplace_back(root);
        }
    }

    // // d_propagated_frontier
    // for (auto [node_id, value] : d_propagated_frontier) {
    //     uint32_t final_id = d_final_ids[node_id];
    //     assert(final_id != UINT32_MAX);
    //     Node root = d_final_nodes[final_id];
    //     if (!value)
    //         root = d_nm.mk_node(node::Kind::NOT, {root});
    //     d_final_roots.emplace_back(root);
    // }
    d_encode_count = 0;
    for (auto& node : d_encoded_nodes) {
        // display encoded nodes
        if (!node.is_value() || !node.value<bool>()) {
            // std::cout << "Encoded node: " << node << "\n";
            d_final_roots.emplace_back(node);
            ++d_encode_count;
        }
    }
}

void Propagator::encode_marked_node(const Node node, const FeasibleDomain& domain, const EncodingMark mark) {
    // auto encode_bits = [&]() {
    //     Node new_node;
    //     // Encode fixed bits as (node & mask) == constant.
    //     if (node.type().is_bool()) {
    //         if (domain.is_fixed(0)) {
    //             new_node = domain.get_value(0) ? node : d_nm.mk_node(node::Kind::NOT, {node});
    //         }
    //         else {
    //             // do nothing
    //             return;
    //         }
    //     }
    //     else {
    //         assert(node.type().is_bv());
    //         BitVector mask = BitVector::mk_zero(domain.width()), constant = BitVector::mk_zero(domain.width());
    //         for (uint32_t i = 0; i < domain.width(); ++i) {
    //             mask.set_bit(i, domain.is_fixed(i) ? 1 : 0);
    //             if (domain.is_fixed(i) && domain.get_value(i)) {
    //                 constant.set_bit(i, 1);
    //             }
    //         }
    //         new_node = d_nm.mk_node(node::Kind::EQUAL, {d_nm.mk_node(node::Kind::BV_AND, {node, d_nm.mk_value(mask)}), d_nm.mk_value(constant)});
    //     }
    //     new_node = d_rewriter.rewrite(new_node);
    //     // std::cout << "Encoding bits for node " << node << "\nas\n" << new_node << "\n";
    //     d_encoded_nodes.emplace_back(new_node);
    // };

    auto encode_bits = [&]() {
        Node new_node;

        // Encode fixed bits as (node & mask) == constant.
        if (node.type().is_bool()) {
            if (domain.is_fixed(0)) {
                new_node = domain.get_value(0) ? node : d_nm.mk_node(node::Kind::NOT, {node});
                new_node = d_rewriter.rewrite(new_node);
                // std::cout << "Encoding bits for node " << node << "\nas\n" << new_node << "\n";
                d_encoded_nodes.emplace_back(new_node);
            }
        }
        else {
            assert(node.type().is_bv());
            // BitVector mask = BitVector::mk_zero(domain.width()), constant = BitVector::mk_zero(domain.width());
            // for (uint32_t i = 0; i < domain.width(); ++i) {
            //     mask.set_bit(i, domain.is_fixed(i) ? 1 : 0);
            //     if (domain.is_fixed(i) && domain.get_value(i)) {
            //         constant.set_bit(i, 1);
            //     }
            // }
            // new_node = d_nm.mk_node(node::Kind::EQUAL, {d_nm.mk_node(node::Kind::BV_AND, {node, d_nm.mk_value(mask)}), d_nm.mk_value(constant)});
            uint32_t width = domain.width(), lst = UINT32_MAX;
            for (uint32_t i = 0; i < width; ++i) {
                if (domain.is_fixed(i)) {
                    if (lst == UINT32_MAX) {
                        lst = i;
                    }
                }
                else {
                    if (lst != UINT32_MAX) {
                        uint32_t sz = i - lst;
                        BitVector bv_value(sz);
                        for (uint32_t j = 0; j < sz; ++j) {
                            bv_value.set_bit(j, domain.get_value(lst + j));
                        }
                        new_node = d_nm.mk_node(node::Kind::EQUAL, {d_nm.mk_node(node::Kind::BV_EXTRACT, {node}, {i - 1, lst}), d_nm.mk_value(bv_value)});
                        new_node = d_rewriter.rewrite(new_node);
                        // std::cout << "Encoding bits for node " << node << "\nas\n" << new_node << "\n";
                        d_encoded_nodes.emplace_back(new_node);
                        lst = UINT32_MAX;
                    }
                }
            }

            if (lst != UINT32_MAX) {
                uint32_t sz = width - lst;
                BitVector bv_value(sz);
                for (uint32_t j = 0; j < sz; ++j) {
                    bv_value.set_bit(j, domain.get_value(lst + j));
                }
                new_node = d_nm.mk_node(node::Kind::EQUAL, {d_nm.mk_node(node::Kind::BV_EXTRACT, {node}, {width - 1, lst}), d_nm.mk_value(bv_value)});
                new_node = d_rewriter.rewrite(new_node);
                // std::cout << "Encoding bits for node " << node << "\nas\n" << new_node << "\n";
                d_encoded_nodes.emplace_back(new_node);
            }
        }
    };

    auto encode_intervals = [&]() {
        // Complementary interval: x - upper <= lower - upper.
        // Regular interval: lower <= node <= upper.

        if (!node.type().is_bv()) return;

        Node new_node;
        BitVector lower = domain.interval_min();
        BitVector upper = domain.interval_max();
        if (domain.is_interval_complementary()) {
            assert(!lower.is_uadd_overflow(upper.bvneg()));
            BitVector range = lower.bvadd(upper.bvneg());
            new_node = d_nm.mk_node(
                node::Kind::BV_ULE,
                {d_nm.mk_node(node::Kind::BV_SUB, {node, d_nm.mk_value(upper)}), d_nm.mk_value(range)});
        }
        else {
            if (!lower.is_zero()) {
                new_node = d_nm.mk_node(
                    node::Kind::BV_ULE,
                    {d_nm.mk_value(lower), node});
            }
            if (!upper.is_ones()) {
                Node upper_node = d_nm.mk_node(
                    node::Kind::BV_ULE,
                    {node, d_nm.mk_value(upper)});
                if (new_node.is_null()) {
                    new_node = upper_node;
                }
                else {
                    new_node = d_nm.mk_node(
                        node::Kind::AND,
                        {new_node, upper_node});
                }
            }
        }
        if (new_node.is_null()) {
            return;
        }
        new_node = d_rewriter.rewrite(new_node);
        // std::cout << "Encoding interval for node " << node << "\nas\n" << new_node << "\n";
        d_encoded_nodes.emplace_back(new_node);
    };

    switch (mark) {
        case EncodingMark::NONE:
        case EncodingMark::REMOVED:
            break;
        case EncodingMark::BITS:
            encode_bits();
            break;
        case EncodingMark::INTERVAL:
            encode_intervals();
            break;
        case EncodingMark::BOTH:
            encode_bits();
            encode_intervals();
            break;
        default:
            assert(false && "Unsupported encoding mark");
            break;
    }
}

#ifdef BZLA_FDP_ENABLE_DEBUG_CHECKS
void Propagator::debug_check_frontier(const char* stage_label) {
    if (d_num_nodes == 0) {
        std::cout << "[FDP][debug] frontier check skipped (empty graph) after "
                  << (stage_label ? stage_label : "unknown") << std::endl;
        return;
    }
    if (d_conflict) {
        std::cout << "[FDP][debug] frontier check skipped due to conflict after "
                  << (stage_label ? stage_label : "unknown") << std::endl;
        return;
    }

    const auto saved_conflict = d_conflict;
    const auto saved_queue = d_queue;
    const auto saved_changed_nodes = d_changed_nodes;
    std::vector<FeasibleDomain> saved_domains;
    saved_domains.reserve(d_domains.size());
    for (const FeasibleDomain& domain : d_domains) {
        saved_domains.emplace_back(domain);
    }
    std::vector<bool> saved_enqueued(d_num_nodes);
    std::vector<bool> saved_changed(d_num_nodes);
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        saved_enqueued[i] = d_enqueued[i];
        saved_changed[i] = d_changed[i];
    }

    // Reset current domains to unconstrained states and clear propagation data.
    std::priority_queue<uint32_t>().swap(d_queue);
    d_changed_nodes.clear();
    d_conflict = false;
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        d_enqueued[i] = false;
        d_changed[i] = false;
        d_domains[i].reset();
    }

    // Seed known constants.
    for (uint32_t node_id : d_constants) {
        FeasibleDomain& domain = d_domains[node_id];
        const Node& node = d_nodes[node_id];
        const Type& type = node.type();
        if (type.is_bv()) {
            const BitVector& value = node.value<BitVector>();
            Result res = domain.set_constant(value);
            if (is_conflict(res)) {
                d_conflict = true;
                break;
            }
            if (is_changed(res)) {
                propagate_enqueue(node_id);
            }
        }
        else if (type.is_bool()) {
            const bool value = node.value<bool>();
            Result res = domain.set_constant(value);
            if (is_conflict(res)) {
                d_conflict = true;
                break;
            }
            if (is_changed(res)) {
                propagate_enqueue(node_id);
            }
        }
    }

    // Apply the current frontier assignments.
    for (const auto& frontier_entry : d_propagated_frontier) {
        uint32_t node_id = frontier_entry.first;
        bool value = frontier_entry.second;
        Result res = d_domains[node_id].set_constant(value);
        if (is_conflict(res)) {
            d_conflict = true;
            break;
        }
        if (is_changed(res)) {
            propagate_enqueue(node_id);
        }
    }

    feasible_domain_propagate();

    bool conflict_during_check = d_conflict;
    std::vector<uint32_t> failing_roots;
    bool invariant_failed = false;
    uint32_t invariant_node = UINT32_MAX;
    std::string invariant_message;
    if (!conflict_during_check) {
        for (uint32_t i = 0; i < d_num_nodes; ++i) {
            std::ostringstream oss;
            if (!d_domains[i].check_invariants(&oss)) {
                invariant_failed = true;
                invariant_node = i;
                invariant_message = oss.str();
                break;
            }
        }
    }
    if (!conflict_during_check) {
        for (uint32_t root_idx : d_roots) {
            if (!d_domains[root_idx].is_fixed_one(0)) {
                failing_roots.emplace_back(root_idx);
            }
        }
    }

    const bool success = !conflict_during_check && !invariant_failed && failing_roots.empty();
    const size_t frontier_size = d_propagated_frontier.size();

    // Restore solver state back to the original run context.
    d_conflict = saved_conflict;
    d_queue = saved_queue;
    d_changed_nodes = saved_changed_nodes;
    for (uint32_t i = 0; i < d_num_nodes; ++i) {
        d_enqueued[i] = saved_enqueued[i];
        d_changed[i] = saved_changed[i];
        d_domains[i].copy_from(saved_domains[i]);
    }

    const char* label = stage_label ? stage_label : "unknown";
    std::cout << "[FDP][debug] frontier->top check after " << label << ": "
              << (success ? "OK" : "FAILED") << " (frontier="
              << frontier_size << ")" << std::endl;
    if (!success) {
        if (conflict_during_check) {
            std::cout << "    propagation triggered a conflict" << std::endl;
        }
        if (invariant_failed) {
            std::cout << "    domain invariants violated @node " << invariant_node
                      << " -> " << invariant_message;
            if (!invariant_message.empty() && invariant_message.back() != '\n') {
                std::cout << '\n';
            }
            std::cout << "    domain state: "
                      << d_domains[invariant_node].to_string(true) << std::endl;
        }
        if (!failing_roots.empty()) {
            std::cout << "    roots not implied (showing up to 5 ids): ";
            size_t printed = 0;
            for (uint32_t idx : failing_roots) {
                std::cout << idx;
                ++printed;
                if (printed == failing_roots.size() || printed == 5)
                    break;
                std::cout << ", ";
            }
            if (failing_roots.size() > 5) {
                std::cout << " ... (" << failing_roots.size() << " total)";
            }
            std::cout << std::endl;
        }
        assert(false && "frontier assignments do not imply top == true");
    }
}
#endif

bool Propagator::build_partition_tasks(std::vector<Node>& left,
                                       std::vector<Node>& right) {
    clear_partition_unsat();
    PartitionTaskBuilder builder(*this, d_partition_seed);
    return builder.build(left, right);
}

void Propagator::check_roots_fixed_true_impl(const char* file,
                                             int line,
                                             const char* func) const {
    for (uint32_t root_idx : d_roots) {
        const FeasibleDomain& domain = d_domains[root_idx];
        if (!domain.is_totally_fixed() || !domain.get_value(0)) {
            std::cerr << "[check_roots_fixed_true] triggered at " << file << ":"
                      << line << " (" << func << ") root topo_id=" << root_idx
                      << " node=" << d_nodes[root_idx]
                      << " domain=" << domain.to_string(true) << std::endl;
            assert(domain.is_totally_fixed());
            assert(domain.get_value(0) == true);
        }
    }
}

void Propagator::run_propagation_only(const std::vector<Node>& roots) {
    reset_run_state();
    init_nodes_in_topo(roots);
    initialize_domains();

    run_full_propagation();
    if (d_conflict) {
        set_unsat_final();
        return;
    }
    CHECK_ROOTS_FIXED_TRUE();

    stage_prefix_level_zero();
    d_final_roots.assign(roots.begin(), roots.end());
}

void Propagator::run(const std::vector<Node>& roots) {
    // for (const Node & node : roots) {
    //     d_final_roots.emplace_back(node);
    // }
    // return ;

    reset_run_state();
    init_nodes_in_topo(roots);
    initialize_domains();

    run_full_propagation();
#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[full_propagation] " << d_propagate_cnt << " propagations\n";
    fdp_time_log("after_full_propagation");
#endif
    if (d_conflict) {
        set_unsat_final();
        return;
    }
    CHECK_ROOTS_FIXED_TRUE();
    // std::cout << (d_conflict ? "unsat" : "unknown") << std::endl;
    // exit(0);

    collect_propagated_frontier();
#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[collect_propagated_frontier] bool frontier size from "
                << roots.size() << " to "
                << d_propagated_frontier.size() << "\n";
    fdp_time_log("after_remove_redundant");

#endif
    FDP_DEBUG_RUN(debug_check_frontier("collect_propagated_frontier"));
    mark_encoding_domains();

#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[mark_encoding] " << d_base_propagate_cnt << " propagations\n";
    fdp_time_log("after_mark_encoding");
#endif
    // after marking, we have the marked nodes and updated domains without frontiers
    if (d_conflict) {
        set_unsat_final();
        return;
    }

    // // display all domains after marking
    // for (uint32_t id = 0; id < d_num_nodes; ++id) {
    //     std::cout << "Node " << id << " " << d_nodes[id] << " domain: "
    //               << d_domains[id].to_string(true) << " mark: "
    //               << mark_to_cstr(d_marks[id]) << "\n";
    // }

    // // display all domains after marking
    // for (uint32_t id = 0; id < d_num_nodes; ++id) {
    //     if (d_domains[id].is_full())
    //         continue;
    //     if (d_marks[id] == EncodingMark::NONE)
    //         continue;
    //         // std::cout << "Node " << id << " domain: "
    //     //           << d_domains[id].to_string(true) << " mark: "
    //     //           << mark_to_cstr(d_marks[id]) << "\n";
    //     std::cout << "Node " << id << " " << d_nodes[id] << " domain: "
    //     // std::cout << "Node " << id << " " << " domain: "
    //             << d_domains[id].to_string(true) << " mark: "
    //             << mark_to_cstr(d_marks[id]) << "\n";
    // }

    // apply_frontier_assignments();
    // if (d_conflict)
    //     return;

    // Stage current feasible domains (with encoded nodes applied and constants) as level 0.
    stage_prefix_level_zero();

    // Here, d_domains keep the encodings and constants only, then propagate to stabilize
    remove_redundant_frontier_nodes();
    if (d_conflict) {
        set_unsat_final();
        return;
    }
    FDP_DEBUG_RUN(debug_check_frontier("remove_redundant_frontier_nodes"));

#ifdef BZLA_FDP_PARTI_INFO
    std::cout << "[remove_redundant] reduced frontier size from "
                << d_propagated_frontier.size() << " to "
                << d_reduced_frontier.size() << "\n";
    fdp_time_log("after_remove_redundant");

#endif

    // // display all reduced frontier
    // std::cout << "Reduced frontier (" << d_reduced_frontier.size() << "):\n";
    // for (auto [node_id, value] : d_reduced_frontier) {
    //     // std::cout << "  Node " << node_id << " " << d_nodes[node_id]
    //     std::cout << "  Node " << node_id 
    //               << " = " << (value ? "true" : "false") << "\n";
    //     std::cout << "    Domain: " << d_domains[node_id].to_string(true) << "\n";
    // }

    // // minimal frontier
    // remove all non-relevant nodes in simplified DAG
    // rebuild_dag_with_roots();
    rebuild_dag_after_encoding();
    FDP_DEBUG_RUN(debug_check_frontier("encode_marked_nodes"));
#ifdef BZLA_FDP_PARTI_INFO
    // fdp_time_log("after_remove_redundant");

    std::cout << "[rebuild_dag_after_encoding] frontier size from "
                << d_reduced_frontier.size() << " to "
                << d_final_roots.size()
                << ", encodes: " << d_encoded_nodes.size() << "\n";
    std::flush(std::cout);
    fdp_time_log("after_rebuild_dag");
#endif
    // // Final propagation to check we can get top true with new frontier and encoded nodes
    // check_final_propagation();
}

PassFeasibleDomainPropagator::PassFeasibleDomainPropagator(
    Env& env, backtrack::BacktrackManager* backtrack_mgr)
    : PreprocessingPass(env, backtrack_mgr, "fdp", "feasible_domain"),
      d_stats(env.statistics(), "preprocess::" + name() + "::") {
}

void PassFeasibleDomainPropagator::apply(AssertionVector& assertions) {
    util::Timer timer(d_stats_pass.time_apply);
    ++d_stats.d_num_apply;
    d_conflict = false;
    d_rolled_back = false;
    fdp_timer_start();
#ifdef BZLA_FDP_PARTI_INFO
    fdp_time_log("enter_fdp");
#endif
    assert(assertions.size() > 0);

    std::vector<Node> roots;
    roots.reserve(assertions.size());
    for (size_t i = 0; i < assertions.size(); ++i) {
      roots.emplace_back(assertions[i]);
    }

    Propagator propagator(d_env, d_stats);
    propagator.run(roots);

// #define simplified_output
#ifdef simplified_output  // turn on for simpilfied smt
    if (propagator.has_conflict()) {
        std::cout << "(set-logic QF_BV)" << std::endl;
        std::cout << "(assert false)" << std::endl;
        std::cout << "(check-sat)" << std::endl;
        return;
    }
#endif
 
    // std::cerr << "[fdp] run done" << std::endl;

    Node null;
    NodeManager& nm = d_env.nm();
    if (propagator.has_conflict()) {
        // std::cout << "[fdp] detected conflict during propagation\n";
        d_conflict = true;
        assertions.push_back(nm.mk_value(false), null);
        return;
    }

    const auto& opts = d_env.options();
    const std::string& dump_prefix = opts.pp_fdp_dump_prefix();

#ifdef simplified_output
    std::vector<Node> print_nodes = propagator.d_final_roots;
    if (d_substitutions != nullptr) {
        for (const auto& [key, value] : *d_substitutions) {
            print_nodes.emplace_back(
                d_env.nm().mk_node(node::Kind::EQUAL, {key, value}));
        }
    }
    Smt2Printer::print_formula(std::cout, print_nodes);
    exit(0);
#endif

    if (propagator.d_encode_count == 0) {
        d_rolled_back = true;
#ifdef BZLA_FDP_PARTI_INFO
        std::cout << "[fdp] rollback simplified roots due to no encoding applied\n";
#endif
    }

    if (!d_rolled_back) {
        const BlastSizeEstimate original_size = estimate_blast_size(roots);
        const BlastSizeEstimate simplified_size =
            estimate_blast_size(propagator.d_final_roots);
#ifdef BZLA_FDP_PARTI_INFO
        fdp_time_log("after_blast_estimate");
#endif
        if (!has_meaningful_blast_improvement(original_size,
                                            simplified_size)) {
            d_rolled_back = true;
#ifdef BZLA_FDP_PARTI_INFO
            std::cout << "[fdp] rollback simplified roots due to <= threshold blast "
                            "improvement\n";
#endif
        }
    }

    if (dump_prefix.empty()) {
        if (d_rolled_back) {
            return;
        }

        std::unordered_set<Node> assertion_set(
            propagator.d_final_roots.begin(),
            propagator.d_final_roots.end());
        for (size_t i = 0; i < assertions.size(); ++i) {
            if (assertions[i].is_value())
                continue;
            else if (assertion_set.find(assertions[i]) != assertion_set.end()) {
                assertion_set.erase(assertions[i]);
            }
            else {
                assertions.replace(i, nm.mk_value(true));
            }
        }

        for (const auto& assertion : assertion_set) {
            assertions.push_back(assertion, null);
        }
        return;
    }

    {
        const uint64_t node_id = opts.pp_fdp_node_id();
        const uint64_t next_id = opts.pp_fdp_next_id();
        const uint64_t parti_seed = opts.pp_fdp_parti_seed() << 32 | node_id;
        // const uint64_t parti_seed = node_id;
        const bool need_slash = dump_prefix.back() != '/';
        const std::string base = dump_prefix + (need_slash ? "/" : "");
        const auto to_path = [&](const std::string& name) {
            return base + name;
        };

        std::vector<Node> print_nodes = d_rolled_back ?  roots : propagator.d_final_roots;

        if (d_substitutions != nullptr) {
          for (const auto& [key, value] : *d_substitutions) {
              print_nodes.emplace_back(
                  d_env.nm().mk_node(node::Kind::EQUAL, {key, value}));
          }
        }

        auto dump_formula = [&](const std::string& path,
                                const std::vector<Node>& nodes) {
            std::ofstream out(path);
            if (!out.is_open()) {
                std::cerr << "[fdp] failed to open " << path << " for writing\n";
                return;
            }
            Smt2Printer::print_formula(out, nodes);
        };

        dump_formula(
            to_path("task-" + std::to_string(node_id) + "-simplified.smt2"),
            print_nodes);
        
#ifdef BZLA_FDP_PARTI_INFO
        fdp_time_log("after_simplified_dump");
#endif
        if (next_id != 0) {
            Propagator partition_propagator(d_env, d_stats);
            // Re-run only the propagation stage to stage baseline domains for partitioning.
            if (!d_rolled_back) {
                partition_propagator.run_propagation_only(propagator.d_final_roots);
            }
            else {
                partition_propagator.run_propagation_only(roots);
            }
            if (partition_propagator.has_conflict()) {
                std::cerr << "[fdp] propagation-only run reported conflict; skipping partition\n";
                d_conflict = true;
                assertions.push_back(nm.mk_value(false), null);
                return;
            }
            partition_propagator.set_partition_seed(parti_seed);
            std::vector<Node> left, right;
            const bool built =
                partition_propagator.build_partition_tasks(left, right);
            if (partition_propagator.has_partition_unsat()) {
                std::cerr << "[fdp] partition probing reported conflict; "
                                "treating as unsat\n";
                d_conflict = true;
                assertions.push_back(nm.mk_value(false), null);
                return;
            }
#ifdef BZLA_FDP_PARTI_INFO
            fdp_time_log("after_partition");
#endif
            if (built) {
                if (d_substitutions != nullptr) {
                    for (const auto& [key, value] : *d_substitutions) {
                        Node eq = d_env.nm().mk_node(node::Kind::EQUAL,
                                                        {key, value});
                        left.emplace_back(eq);
                        right.emplace_back(eq);
                    }
                }
                dump_formula(
                    to_path("task-" + std::to_string(next_id) + ".smt2"),
                    left);
                dump_formula(
                    to_path("task-" + std::to_string(next_id + 1) + ".smt2"),
                    right);
            }
            else {
                std::cerr << "[fdp] partition requested but no suitable symbol found\n";
                exit(233);
            }
        }
        exit(0);
    }
}

}  // namespace bzla::preprocess::pass::fdp
