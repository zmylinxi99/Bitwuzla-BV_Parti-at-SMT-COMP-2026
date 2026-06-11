#pragma once

#include <cassert>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "preprocess/pass/feasible_domain_propagator/fdp_prefix_array.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"
#include "preprocess/pass/feasible_domain_propagator/operator_statistics.h"
#include "preprocess/preprocessing_pass.h"
#include "rewrite/rewriter.h"

namespace bzla {
class Node;
class NodeManager;
}  // namespace bzla

namespace bzla::preprocess::pass::fdp {

namespace {

// constexpr uint32_t kDomainEncodeThres = 3; // n means domain at least reduced to 1 / n of full size

}  // namespace

class PartitionTaskBuilder;

class Propagator {
  public:
    explicit Propagator(Env& env, OperatorStatistics& stats);

    /**
     * Execute the full feasible-domain propagation pipeline on the given roots.
     * This rebuilds the DAG, runs propagation, marks encodings, and emits new
     * assertions derived from encoded nodes and the reduced frontier.
     */
    void run(const std::vector<Node>& roots);
    /** Run only the initial propagation and stage baseline domains. */
    void run_propagation_only(const std::vector<Node>& roots);

    bool has_conflict() const { return d_conflict; }
    void set_conflict() { d_conflict = true; }
    bool has_partition_unsat() const { return d_partition_unsat; }
    void mark_partition_unsat() { d_partition_unsat = true; }
    void clear_partition_unsat() { d_partition_unsat = false; }
    const std::vector<FeasibleDomain>& domains() const { return d_domains; }
    void set_partition_seed(uint64_t seed) { d_partition_seed = seed; }
    bool build_partition_tasks(std::vector<Node>& left,
                               std::vector<Node>& right);

    std::vector<Node> d_final_roots;
    unsigned d_encode_count;

  private:
    friend class PartitionTaskBuilder;
    /** Build the topo-sorted DAG structure reachable from the given roots. */
    void init_nodes_in_topo(const std::vector<Node>& roots);
    /** Drain the propagation queue until a fixed point or conflict. */
    void feasible_domain_propagate();

    /** Apply per-operator domain propagation on a single node. */
    void propagate_node(uint32_t node_id);
    /** Enqueue a node if it is not already scheduled for propagation. */
    void propagate_enqueue(uint32_t node_id);
    /** Track nodes whose domains changed in the current propagation round. */
    void collect_changed(uint32_t node_id);
    /**
     * On conflict paths we can return before consume_state(), so eagerly
     * collect nodes that may carry temporary probe state for rollback.
     */
    void collect_changed_on_conflict(uint32_t node_id);
    /** Consume and normalize the pending state stored in a domain. */
    Result consume_domain_state(uint32_t node_id);

    /** Reset all cached state so a fresh propagation run can start. */
    void reset_run_state();
    /** Stage a level-0 prefix snapshot from the current changed domains. */
    void stage_prefix_level_zero();
    /** Set final roots to a single unsat assertion. */
    void set_unsat_final();
    /** Allocate and initialize per-node feasible domains. */
    void initialize_domains();
    /** */
    void set_constant_domains();
    /** Run the initial bottom-up propagation seeded by constants and roots. */
    void run_full_propagation();
    /** Collect Boolean frontier nodes that remain implied by propagation. */
    void collect_propagated_frontier();

    // void reset_domains();
    void check_roots_fixed_true_impl(const char* file,
                                     int line,
                                     const char* func) const;
    void mark_encoding_domains();
    void remove_redundant_frontier_nodes();
    void apply_frontier_assignments();
    void encode_marked_node(const Node node, const FeasibleDomain& domain, const EncodingMark mark);

    void rebuild_dag_after_encoding();
#ifdef BZLA_FDP_ENABLE_DEBUG_CHECKS
    void debug_check_frontier(const char* stage_label);
#endif

    NodeManager& d_nm;
    Rewriter& d_rewriter;
    std::vector<Node> d_nodes;
    OperatorStatistics& d_stats;

    size_t d_num_nodes;
    size_t d_num_bool_nodes;

    std::unordered_map<uint64_t, uint32_t> d_topo_dict;
    std::vector<FeasibleDomain> d_domains;
    // std::vector<uint32_t> d_dag_size;
    // final assertions
    std::vector<uint32_t> d_roots;
    std::vector<uint32_t> d_constants;
    std::vector<std::vector<uint32_t>> d_parents;
    std::vector<std::vector<uint32_t>> d_children;
    // boolean propagated frontier
    // std::vector<uint32_t> d_propagated_frontier;
    std::vector<std::pair<uint32_t, bool>> d_propagated_frontier;
    std::vector<std::pair<uint32_t, bool>> d_reduced_frontier;

    std::unique_ptr<FdPrefixArray> d_fd_prefix;

    std::unique_ptr<bool[]> d_enqueued;
    // which is better? TODO: test the performance
    std::priority_queue<uint32_t> d_queue;
    // std::priority_queue<uint32_t, std::vector<uint32_t>, std::greater<uint32_t>> d_queue;

    // changed nodes in current propagation
    std::unique_ptr<bool[]> d_changed;
    std::vector<uint32_t> d_changed_nodes;

    uint32_t d_propagate_cnt{0};
    // const and marked nodes only
    uint32_t d_base_propagate_cnt;
    // full propagation count
    uint32_t d_frontier_propagate_cnt;

    std::vector<EncodingMark> d_marks;
    bool d_conflict{false};
    bool d_partition_unsat{false};
    uint64_t d_partition_seed{0};

    std::vector<Node> d_encoded_nodes;
    std::vector<Node> d_final_nodes;
    std::vector<uint32_t> d_final_ids;
};

class PassFeasibleDomainPropagator : public PreprocessingPass {
  public:
    PassFeasibleDomainPropagator(Env& env,
                                 backtrack::BacktrackManager* backtrack_mgr);

    void apply(AssertionVector& assertions) override;

    bool has_conflict() const { return d_conflict; }
    bool has_rolled_back() const { return d_rolled_back; }

    void set_substitutions(const std::unordered_map<Node, Node>* subs) {
        d_substitutions = subs;
    }

  private:
    bool d_conflict{false};
    bool d_rolled_back{false};
    const std::unordered_map<Node, Node>* d_substitutions{nullptr};

    OperatorStatistics d_stats;
};

}  // namespace bzla::preprocess::pass::fdp

#ifndef CHECK_ROOTS_FIXED_TRUE
#define CHECK_ROOTS_FIXED_TRUE() \
    check_roots_fixed_true_impl(__FILE__, __LINE__, __func__)
#endif

// #define CHECK_ROOTS_FIXED_TRUE() {}
// #endif

namespace bzla::preprocess::pass {
using PassFeasibleDomainPropagator = fdp::PassFeasibleDomainPropagator;
}  // namespace bzla::preprocess::pass
