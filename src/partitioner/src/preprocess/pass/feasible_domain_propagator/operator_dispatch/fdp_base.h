#pragma once

#include <cstdint>
#include <vector>

#include "node/node_kind.h"

namespace bzla {
class Node;
}  // namespace bzla

namespace bzla::preprocess::pass::fdp {

class FeasibleDomain;

enum class Result : uint8_t {
    UNCHANGED = 0,
    CHANGED = 1,
    UPDATED = 2,
    CONFLICT = 3,
};

// enum class Status : uint8_t {
//     UNUPDATED = 0,
//     CHANGED = 1,
//     UPDATED = 2,
//     CONFLICT = 3
// };

constexpr Result
operator|(Result a, Result b) {
    if (a == Result::CONFLICT || b == Result::CONFLICT) {
        return Result::CONFLICT;
    }
    if (a == Result::UPDATED || b == Result::UPDATED) {
        return Result::UPDATED;
    }
    if (a == Result::CHANGED || b == Result::CHANGED) {
        return Result::CHANGED;
    }
    return Result::UNCHANGED;
}

constexpr Result
operator|=(Result& a, Result b) {
    a = a | b;
    return a;
}

constexpr bool
is_unchanged(Result r) {
    return r == Result::UNCHANGED;
}

constexpr bool
is_changed(Result r) {
    return r == Result::CHANGED || r == Result::UPDATED;
}

constexpr bool
is_updated(Result r) {
    return r == Result::UPDATED;
}

constexpr bool
is_conflict(Result r) {
    return r == Result::CONFLICT;
}

class FdpOperator {
  public:
    using DomainVector = std::vector<FeasibleDomain*>;

    FdpOperator(node::Kind kind,
                const char* debug_name,
                DomainVector& children,
                FeasibleDomain* self);
    virtual ~FdpOperator() = default;

    node::Kind kind() const { return d_kind; }
    const char* name() const { return d_name; }

    virtual Result apply() = 0;

  protected:
    DomainVector& d_children;
    FeasibleDomain* d_self;

  private:
    node::Kind d_kind;
    const char* d_name;
};

}  // namespace bzla::preprocess::pass::fdp
