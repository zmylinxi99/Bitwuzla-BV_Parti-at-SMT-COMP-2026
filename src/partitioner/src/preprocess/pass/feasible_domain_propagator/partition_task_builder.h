#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace bzla {
class BitVector;
class Node;
class Type;
}

namespace bzla::preprocess::pass::fdp {

class FeasibleDomain;
class FdPrefixArray;
class Propagator;
enum class Result : uint8_t;

class PartitionTaskBuilder {
  public:
    explicit PartitionTaskBuilder(Propagator& propagator,
                                  uint64_t seed = 0);
    bool build(std::vector<Node>& left, std::vector<Node>& right);

  private:
    enum class PartitionKind : uint8_t {
        BOOL,
        BV_BIT,
        BV_INTERVAL,
        BV_ADD_OVERFLOW,
    };
    enum class OverflowSplitKind : uint8_t {
        COMPARE_CHILD,
        OUTPUT_BOUNDARY,
    };

    struct PartitionAction;
    struct BranchEval;

    static int compare_action_key(const PartitionAction& a,
                                  const PartitionAction& b);
    static bool action_key_less(const PartitionAction& a,
                                const PartitionAction& b);

    bool init_history();
    const FeasibleDomain* baseline_domain_ptr(uint32_t id) const;
    const FeasibleDomain& baseline_domain(uint32_t id) const;
    void compute_partition_heuristics();
    void compute_difficulty();

    double interval_log2_size(const FeasibleDomain& dom) const;
    double node_difficulty(const Node& node) const;
    double node_uncertainty(const FeasibleDomain& dom,
                            const Type& type) const;

    uint32_t choose_partition_bit(uint32_t id,
                                  const FeasibleDomain& dom) const;
    bool compute_interval_split(uint32_t node_id,
                                BitVector& l,
                                BitVector& u,
                                BitVector& l2,
                                BitVector& u2) const;
    bool overflow_const_child_info(const Node& add,
                                   uint32_t& const_idx,
                                   uint32_t& var_idx,
                                   BitVector& limit) const;
    bool select_overflow_compare_child(uint32_t add_id,
                                       uint32_t& child_topo) const;

    void normalize_wrapping_interval(const BitVector& l,
                                     const BitVector& u,
                                     BitVector& out_min,
                                     BitVector& out_max,
                                     bool& out_comp) const;
    Result apply_wrapping_interval_constraint(FeasibleDomain& dom,
                                              const BitVector& l,
                                              const BitVector& u) const;
    Node mk_interval_lower_lit(const Node& term,
                               const BitVector& bound) const;
    Node mk_interval_upper_lit(const Node& term,
                               const BitVector& bound) const;
    Node mk_bit_lit(const Node& term, uint32_t bit, bool value) const;
    static const char* overflow_split_kind_to_string(OverflowSplitKind kind);
    void clear_queue();
    void restore_baseline_nodes();
    BranchEval probe_branch(const PartitionAction& action, bool take_left);

    void collect_action_buckets(
        std::vector<std::vector<PartitionAction>>& buckets) const;
    void add_action(std::vector<std::vector<PartitionAction>>& buckets,
                    PartitionAction&& act) const;
    size_t kind_index(PartitionKind kind) const;
    static const char* kind_to_string(PartitionKind kind);

    std::vector<PartitionAction> collect_partition_actions() const;
    bool select_best_action(const std::vector<PartitionAction>& actions,
                            PartitionAction& best,
                            BranchEval& best_left,
                            BranchEval& best_right);
    double score_action_with_probe(const PartitionAction& action,
                                   const BranchEval& left,
                                   const BranchEval& right) const;
    void emit_partition(const PartitionAction& best,
                        const BranchEval& left_eval,
                        const BranchEval& right_eval,
                        std::vector<Node>& left,
                        std::vector<Node>& right);

    Propagator& d_propagator;
    uint64_t d_seed{0};
    FdPrefixArray* d_history{nullptr};
    uint32_t d_baseline_level{0};
    std::vector<double> d_difficulty;
    std::vector<double> d_baseline_uncertainty;
    std::vector<double> d_node_weight;
    double d_baseline_total{0.0};
    std::vector<double> d_node_heuristic;
    std::vector<Node> d_forced_common_guards;
};

}  // namespace bzla::preprocess::pass::fdp
