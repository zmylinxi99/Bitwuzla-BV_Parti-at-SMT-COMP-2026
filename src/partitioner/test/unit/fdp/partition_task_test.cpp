#include <gtest/gtest.h>

#include <vector>

#include "bv/bitvector.h"
#include "fdp_test_utils.h"
#include "option/option.h"
#include "preprocess/pass/feasible_domain_propagator/propagator.h"
#include "solving_context.h"
#include "test_preprocess_pass.h"

namespace bzla::preprocess::pass::fdp {
namespace {

using namespace node;
using ::bzla::BitVector;
using ::bzla::Node;
using ::bzla::Result;
using ::bzla::Type;

class TestFdpPartitionTasks : public bzla::test::TestPreprocessingPass {
  protected:
    Env d_env;

    TestFdpPartitionTasks() : d_env(d_nm) {}

    Result SolveNoFdp(const std::vector<Node>& assertions) {
        option::Options opts = d_options;
        opts.set(option::Option::PP_FDP_ENABLE, false);
        SolvingContext ctx(d_nm, opts);
        for (const Node& a : assertions) {
            ctx.assert_formula(a);
        }
        return ctx.solve();
    }
};

TEST_F(TestFdpPartitionTasks, SimplifiedEquivalentAndPartitionSat) {
    Type bv4 = d_nm.mk_bv_type(4);
    Node x = d_nm.mk_const(bv4, "x");
    Node y = d_nm.mk_const(bv4, "y");

    Node x_mask = d_nm.mk_value(BitVector::from_ui(4, 0b1110));
    Node x_pat = d_nm.mk_value(BitVector::from_ui(4, 0b0010));
    Node y_mask = d_nm.mk_value(BitVector::from_ui(4, 0b1100));
    Node y_pat = d_nm.mk_value(BitVector::from_ui(4, 0b0100));

    Node x_masked = d_nm.mk_node(Kind::BV_AND, {x, x_mask});
    Node y_masked = d_nm.mk_node(Kind::BV_AND, {y, y_mask});
    Node x_hi_eq = d_nm.mk_node(Kind::EQUAL, {x_masked, x_pat});
    Node y_hi_eq = d_nm.mk_node(Kind::EQUAL, {y_masked, y_pat});
    Node root = d_nm.mk_node(Kind::AND, {x_hi_eq, y_hi_eq});

    std::vector<Node> original{root};

    Propagator propagator(d_env, test::TestOperatorStats());
    propagator.run(original);

    ASSERT_FALSE(propagator.d_final_roots.empty());

    const Result orig_res = SolveNoFdp(original);
    const Result simp_res = SolveNoFdp(propagator.d_final_roots);
    ASSERT_EQ(orig_res, Result::SAT);
    ASSERT_EQ(simp_res, Result::SAT);

    Propagator partitioner(d_env, test::TestOperatorStats());
    partitioner.run_propagation_only(propagator.d_final_roots);

    option::Options simp_opts = d_options;
    simp_opts.set(option::Option::PP_FDP_ENABLE, false);
    SolvingContext simp_ctx(d_nm, simp_opts);
    for (const Node& a : propagator.d_final_roots) {
        simp_ctx.assert_formula(a);
    }
    ASSERT_EQ(simp_ctx.solve(), Result::SAT);
    Node x_simp = simp_ctx.get_value(x);
    Node y_simp = simp_ctx.get_value(y);

    option::Options simp_check_opts = d_options;
    simp_check_opts.set(option::Option::PP_FDP_ENABLE, false);
    SolvingContext simp_check_ctx(d_nm, simp_check_opts);
    for (const Node& a : original) {
        simp_check_ctx.assert_formula(a);
    }
    simp_check_ctx.assert_formula(d_nm.mk_node(Kind::EQUAL, {x, x_simp}));
    simp_check_ctx.assert_formula(d_nm.mk_node(Kind::EQUAL, {y, y_simp}));
    ASSERT_EQ(simp_check_ctx.solve(), Result::SAT);

    std::vector<Node> left, right;
    ASSERT_TRUE(partitioner.build_partition_tasks(left, right));

    const Result left_res = SolveNoFdp(left);
    const Result right_res = SolveNoFdp(right);
    ASSERT_TRUE(left_res == Result::SAT || right_res == Result::SAT);

    // Pick a satisfiable branch and validate its model satisfies the original task.
    std::vector<Node>* sat_branch = nullptr;
    if (left_res == Result::SAT) {
        sat_branch = &left;
    }
    else if (right_res == Result::SAT) {
        sat_branch = &right;
    }
    ASSERT_NE(sat_branch, nullptr);

    option::Options sat_opts = d_options;
    sat_opts.set(option::Option::PP_FDP_ENABLE, false);
    SolvingContext sat_ctx(d_nm, sat_opts);
    for (const Node& a : *sat_branch) {
        sat_ctx.assert_formula(a);
    }
    ASSERT_EQ(sat_ctx.solve(), Result::SAT);

    Node x_val = sat_ctx.get_value(x);
    Node y_val = sat_ctx.get_value(y);

    option::Options check_opts = d_options;
    check_opts.set(option::Option::PP_FDP_ENABLE, false);
    SolvingContext check_ctx(d_nm, check_opts);
    for (const Node& a : original) {
        check_ctx.assert_formula(a);
    }
    check_ctx.assert_formula(d_nm.mk_node(Kind::EQUAL, {x, x_val}));
    check_ctx.assert_formula(d_nm.mk_node(Kind::EQUAL, {y, y_val}));
    ASSERT_EQ(check_ctx.solve(), Result::SAT);
}

TEST_F(TestFdpPartitionTasks, SingleBranchProbeConflictDoesNotMarkGlobalUnsat) {
    Type bool_ty = d_nm.mk_bool_type();
    Node x = d_nm.mk_const(bool_ty, "x");
    Node a = d_nm.mk_const(bool_ty, "a");

    Node not_x = d_nm.mk_node(Kind::NOT, {x});
    Node clause1 = d_nm.mk_node(Kind::OR, {not_x, a});
    Node clause2 = d_nm.mk_node(
        Kind::OR, {not_x, d_nm.mk_node(Kind::NOT, {a})});
    Node root = d_nm.mk_node(Kind::AND, {clause1, clause2});

    std::vector<Node> assertions{root};
    ASSERT_EQ(SolveNoFdp(assertions), Result::SAT);

    Propagator partitioner(d_env, test::TestOperatorStats());
    partitioner.run_propagation_only(assertions);
    ASSERT_FALSE(partitioner.has_conflict());

    std::vector<Node> left, right;
    ASSERT_TRUE(partitioner.build_partition_tasks(left, right));
    ASSERT_FALSE(partitioner.has_conflict());
    ASSERT_FALSE(partitioner.has_partition_unsat());

    const Result left_res = SolveNoFdp(left);
    const Result right_res = SolveNoFdp(right);
    ASSERT_TRUE(left_res == Result::SAT || right_res == Result::SAT);
}

TEST_F(TestFdpPartitionTasks, SimplifiedEquivalentAndPartitionUnsat) {
    Type bv4 = d_nm.mk_bv_type(4);
    Node x = d_nm.mk_const(bv4, "x");
    Node one = d_nm.mk_value(BitVector::mk_one(4));
    Node x_plus_one = d_nm.mk_node(Kind::BV_ADD, {x, one});
    Node root = d_nm.mk_node(Kind::EQUAL, {x, x_plus_one});

    std::vector<Node> original{root};
    Propagator propagator(d_env, test::TestOperatorStats());
    propagator.run(original);

    const Result orig_res = SolveNoFdp(original);
    const Result simp_res = SolveNoFdp(propagator.d_final_roots);
    ASSERT_EQ(orig_res, Result::UNSAT);
    ASSERT_EQ(simp_res, Result::UNSAT);

    Propagator partitioner(d_env, test::TestOperatorStats());
    partitioner.run_propagation_only(propagator.d_final_roots);

    std::vector<Node> left, right;
    ASSERT_TRUE(partitioner.build_partition_tasks(left, right));
    ASSERT_EQ(SolveNoFdp(left), Result::UNSAT);
    ASSERT_EQ(SolveNoFdp(right), Result::UNSAT);
}

}  // namespace
}  // namespace bzla::preprocess::pass::fdp

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
