#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_base.h"

namespace bzla::preprocess::pass::fdp {

FdpOperator::FdpOperator(node::Kind kind,
                         const char* debug_name,
                         DomainVector& children,
                         FeasibleDomain* self)
    : d_children(children), d_self(self), d_kind(kind), d_name(debug_name) {
}

}  // namespace bzla::preprocess::pass::fdp
