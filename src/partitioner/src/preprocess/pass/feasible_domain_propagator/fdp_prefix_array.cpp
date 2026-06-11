#include "preprocess/pass/feasible_domain_propagator/fdp_prefix_array.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>

#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"

namespace bzla::preprocess::pass::fdp {

FdPrefixArray::FdPrefixArray(size_t num_nodes)
    : d_nodes(num_nodes), d_seen_changed(num_nodes, false) {
    d_level_end.push_back(0);
}

void FdPrefixArray::ensure_level(uint32_t level_id) {
    // Levels only increase; callers should not request rollback semantics.
    if (!d_level_end.empty()) {
        if (level_id + 1 < d_level_end.size()) {
            throw std::logic_error(
                "FdPrefixArray does not support decreasing levels");
        }
    }
    else {
        d_level_end.push_back(0);
    }

    while (d_level_end.size() <= level_id) {
        d_level_end.push_back(d_level_end.back());
    }
}

void FdPrefixArray::update(uint32_t level_id,
                           uint32_t node_id,
                           const FeasibleDomain& domain) {
    if (node_id >= d_nodes.size()) {
        return;
    }
    ensure_level(level_id);

    auto& history = d_nodes[node_id];
    if (history.empty()) {
        history.emplace_back(level_id, domain);
    }
    else {
        assert(history.back().d_level < level_id);
        history.emplace_back(level_id, domain);
    }

    if (level_id == 0) {
        assert(!d_level0_sealed && "Level 0 already finalized");
        if (!d_seen_changed[node_id]) {
            d_level0_changed_nodes.push_back(node_id);
            d_seen_changed[node_id] = true;
        }
        d_level0_end = d_level0_changed_nodes.size();
    }
    else {
        assert(d_level0_sealed && "Call finalize_level_zero() before level > 0");
        if (!d_seen_changed[node_id]) {
            d_changed_nodes.push_back(node_id);
            d_seen_changed[node_id] = true;
        }
        d_level_end[level_id] = d_changed_nodes.size();
    }
}

const FeasibleDomain& FdPrefixArray::snapshot_for_level(uint32_t node_id,
                                                        uint32_t level_id) const {
    const auto& history = d_nodes[node_id];
    auto it = std::upper_bound(
        history.begin(),
        history.end(),
        level_id,
        [](uint32_t value, const NodeSnapshot& snap) { return value < snap.d_level; });
    assert(it != history.begin());
    --it;
    return it->d_domain;
}

const FeasibleDomain* FdPrefixArray::get_ptr(uint32_t node_id,
                                             uint32_t level_id) const {
    assert(node_id < d_nodes.size());
    const auto& history = d_nodes[node_id];
    if (history.empty()) {
        return nullptr;
    }
    // Do not return snapshots from future levels: if the earliest snapshot
    // for this node was taken after the requested level, treat it as missing.
    if (level_id < history.front().d_level) {
        return nullptr;
    }
    return &snapshot_for_level(node_id, level_id);
}

uint32_t FdPrefixArray::max_level() const {
    if (d_level_end.empty()) {
        return 0;
    }
    return static_cast<uint32_t>(d_level_end.size() - 1);
}

void FdPrefixArray::finalize_level_zero() {
    if (d_level0_sealed) {
        return;
    }
    d_level0_sealed = true;
    std::fill(d_seen_changed.begin(), d_seen_changed.end(), false);
    d_changed_nodes.clear();
    d_level_end.clear();
    d_level_end.push_back(0);
}

FdPrefixArray::ChangedRange
FdPrefixArray::level0_changed_nodes() const {
    if (d_level0_end == 0) {
        return {};
    }
    const uint32_t* begin_ptr = d_level0_changed_nodes.data();
    return ChangedRange{begin_ptr, begin_ptr + d_level0_end};
}

FdPrefixArray::ChangedRange
FdPrefixArray::changed_nodes(uint32_t level_id) const {
    if (level_id == 0 || d_level_end.empty()) {
        return {};
    }

    if (level_id >= d_level_end.size()) {
        level_id = d_level_end.size() - 1;
    }

    const size_t end_idx = d_level_end[level_id];
    if (end_idx == 0) {
        return {};
    }
    const uint32_t* begin_ptr = d_changed_nodes.data();
    return ChangedRange{begin_ptr, begin_ptr + end_idx};
}

}  // namespace bzla::preprocess::pass::fdp
