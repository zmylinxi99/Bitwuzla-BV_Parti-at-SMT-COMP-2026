#include <gtest/gtest.h>

#include "bv/bitvector.h"
#include "preprocess/assertion_vector.h"
#include "preprocess/pass/feasible_domain_propagator/propagator.h"
#include "test_preprocess_pass.h"

namespace bzla::preprocess::pass::fdp {
namespace {

using namespace node;
using ::bzla::BitVector;

class TestPassFeasibleDomainPropagator : public bzla::test::TestPreprocessingPass {
  public:
    TestPassFeasibleDomainPropagator() : d_env(d_nm), d_pass(d_env, &d_bm) {}

  protected:
    Env d_env;
    PassFeasibleDomainPropagator d_pass;
};

TEST_F(TestPassFeasibleDomainPropagator, PropagatesSimpleEquality) {
    Type bv4 = d_nm.mk_bv_type(4);
    Node x = d_nm.mk_const(bv4, "x");
    Node c = d_nm.mk_value(BitVector::from_ui(4, 1));
    Node eq = d_nm.mk_node(Kind::EQUAL, {x, c});

    d_as.push_back(eq);
    preprocess::AssertionVector assertions(d_as.view());
    d_pass.apply(assertions);

    ASSERT_EQ(assertions.size(), 1);
    EXPECT_EQ(assertions[0], eq);
    EXPECT_FALSE(d_pass.has_conflict());
}

TEST_F(TestPassFeasibleDomainPropagator, DetectsConflictingAssignments) {
    Type bv4 = d_nm.mk_bv_type(4);
    Node x = d_nm.mk_const(bv4, "x");
    Node zero = d_nm.mk_value(BitVector::mk_zero(4));
    Node one = d_nm.mk_value(BitVector::mk_one(4));
    Node eq_zero = d_nm.mk_node(Kind::EQUAL, {x, zero});
    Node eq_one = d_nm.mk_node(Kind::EQUAL, {x, one});

    d_as.push_back(eq_zero);
    d_as.push_back(eq_one);
    preprocess::AssertionVector assertions(d_as.view());

    d_pass.apply(assertions);

    ASSERT_TRUE(d_pass.has_conflict());
    ASSERT_EQ(assertions.size(), 3);
    EXPECT_EQ(assertions[0], eq_zero);
    EXPECT_EQ(assertions[1], eq_one);
    EXPECT_EQ(assertions[2], d_nm.mk_value(false));
}

}  // namespace
}  // namespace bzla::preprocess::pass::fdp

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
