#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"

namespace bzla::preprocess::pass::fdp {

/**
 * Prefix-style history array for feasible domains.
 *
 * Levels only grow monotonically. Level 0 bookkeeping is separated from the
 * remaining levels so we can drop the initial change set and reuse the
 * `seen` tracking for later updates.
 */
class FdPrefixArray {
  public:
    struct ChangedRange {
        const uint32_t* d_begin_ptr{nullptr};
        const uint32_t* d_end_ptr{nullptr};

        const uint32_t* begin() const { return d_begin_ptr; }
        const uint32_t* end() const { return d_end_ptr; }
        bool empty() const { return d_begin_ptr == d_end_ptr; }
        size_t size() const {
            if (d_begin_ptr == nullptr || d_end_ptr == nullptr) {
                return 0;
            }
            return static_cast<size_t>(d_end_ptr - d_begin_ptr);
        }
    };

    struct NodeSnapshot {
        uint32_t d_level{0};
        FeasibleDomain d_domain;

        NodeSnapshot() = delete;
        NodeSnapshot(uint32_t lvl, const FeasibleDomain& dom)
            : d_level(lvl), d_domain(dom) {}
        NodeSnapshot(const NodeSnapshot&) = default;
        NodeSnapshot(NodeSnapshot&&) noexcept = default;
    };

    FdPrefixArray(size_t num_nodes);
    ~FdPrefixArray() = default;

    FdPrefixArray(FdPrefixArray&&) noexcept = default;
    FdPrefixArray& operator=(FdPrefixArray&&) noexcept = default;
    FdPrefixArray(const FdPrefixArray&) = delete;
    FdPrefixArray& operator=(const FdPrefixArray&) = delete;

    void update(uint32_t level_id,
                uint32_t node_id,
                const FeasibleDomain& domain);
    // Call once after finishing level 0 to snapshot its changed nodes and reset
    // the seen flags for subsequent levels.
    void finalize_level_zero();
    const FeasibleDomain* get_ptr(uint32_t node_id,
                                  uint32_t level_id) const;
    /** @return The highest level id currently stored. */
    uint32_t max_level() const;
    // Returns the nodes (unique ids) changed at level 0.
    ChangedRange level0_changed_nodes() const;
    // Returns the nodes (unique ids) changed from level 1 up to and including
    // level_id (level_id==0 yields an empty range).
    ChangedRange changed_nodes(uint32_t level_id) const;

  private:
    void ensure_level(uint32_t level_id);
    const FeasibleDomain& snapshot_for_level(uint32_t node_id,
                                             uint32_t level_id) const;

    std::vector<std::vector<NodeSnapshot>> d_nodes;
    std::vector<bool> d_seen_changed;
    std::vector<uint32_t> d_level0_changed_nodes;
    size_t d_level0_end{0};
    bool d_level0_sealed{false};
    std::vector<uint32_t> d_changed_nodes;
    // d_level_end[l] stores the exclusive end offset into d_changed_nodes for
    // changes recorded up to and including level l.
    std::vector<size_t> d_level_end;
};

}  // namespace bzla::preprocess::pass::fdp
