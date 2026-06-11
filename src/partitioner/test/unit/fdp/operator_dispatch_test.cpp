#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <sstream>

#include "fdp_test_utils.h"
#include "preprocess/pass/feasible_domain_propagator/feasible_domain.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_arithmetic.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_boolean.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_comparison.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_composition.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_division.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_dummy.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_ite.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_multiplication.h"
#include "preprocess/pass/feasible_domain_propagator/operator_dispatch/fdp_shifting.h"

namespace bzla::preprocess::pass::fdp {
namespace {

TEST(FdpBooleanOperatorsTest, NotPropagatesConstant) {
    FeasibleDomain child(1);
    FeasibleDomain output(1);
    child.set_constant(true);

    FdpOperator::DomainVector children{&child};
    FdpNotOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_TRUE(is_changed(res));
    test::ExpectAllInvariant({&child, &output});
    EXPECT_TRUE(output.is_fixed_zero(0));
}

TEST(FdpBooleanOperatorsTest, XorPropagatesSingleUnknown) {
    FeasibleDomain a(1), b(1), output(1);
    a.set_constant(true);
    b.set_constant(false);

    FdpOperator::DomainVector children{&a, &b};
    FdpXorOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_TRUE(is_changed(res));
    test::ExpectAllInvariant({&a, &b, &output});
    EXPECT_TRUE(output.is_fixed_one(0));
}

TEST(FdpBooleanOperatorsTest, AndPropagatesFalse) {
    FeasibleDomain a(1), b(1), output(1);
    a.set_constant(true);
    b.set_constant(false);

    FdpOperator::DomainVector children{&a, &b};
    FdpAndOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_TRUE(is_changed(res));
    test::ExpectAllInvariant({&a, &b, &output});
    EXPECT_TRUE(output.is_fixed_zero(0));
}

TEST(FdpArithmeticOperatorsTest, AddFixesOutputBits) {
    FeasibleDomain lhs(4), rhs(4), output(4);
    test::SetUnsigned(&lhs, 1);
    test::SetUnsigned(&rhs, 2);

    FdpOperator::DomainVector children{&lhs, &rhs};
    FdpAddOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();

    EXPECT_TRUE(is_changed(res));
    test::ExpectAllInvariant({&lhs, &rhs, &output});
    EXPECT_EQ(test::DomainValue(output), 3u);
}

TEST(FdpArithmeticOperatorsTest, MulAcceptsConsistentAssignment) {
    FeasibleDomain lhs(4), rhs(4), output(4);
    test::SetUnsigned(&lhs, 3);
    test::SetUnsigned(&rhs, 2);
    test::SetUnsigned(&output, 6);

    FdpOperator::DomainVector children{&lhs, &rhs};
    FdpMulOperator op(test::TestOperatorStats(), children, &output);
    Result res = op.apply();

    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&lhs, &rhs, &output});
}

TEST(FdpArithmeticOperatorsTest, DivAndRemStayConsistentWithConstants) {
    FeasibleDomain dividend(4), divisor(4), quotient(4), remainder(4);
    test::SetUnsigned(&dividend, 6);
    test::SetUnsigned(&divisor, 2);
    test::SetUnsigned(&quotient, 3);
    test::SetUnsigned(&remainder, 0);

    {
        FdpOperator::DomainVector children{&dividend, &divisor};
        FdpDivOperator div(test::TestOperatorStats(), children, &quotient);
        Result res = div.apply();
        EXPECT_FALSE(is_conflict(res));
    }
    test::ExpectAllInvariant({&dividend, &divisor, &quotient});

    {
        FdpOperator::DomainVector children{&dividend, &divisor};
        FdpRemOperator rem(test::TestOperatorStats(), children, &remainder);
        Result res = rem.apply();
        EXPECT_FALSE(is_conflict(res));
    }
    test::ExpectAllInvariant({&dividend, &divisor, &remainder});
}

TEST(FdpComparisonOperatorsTest, LessThanSetsBooleanOutput) {
    FeasibleDomain lhs(4), rhs(4), output(1);
    test::SetUnsigned(&lhs, 1);
    test::SetUnsigned(&rhs, 2);

    FdpOperator::DomainVector children{&lhs, &rhs};
    FdpLessThanOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&lhs, &rhs, &output});
    EXPECT_TRUE(output.is_fixed_one(0));
}

TEST(FdpComparisonOperatorsTest, SignedLessThanRecognizesNegative) {
    FeasibleDomain lhs(4), rhs(4), output(1);
    test::SetUnsigned(&lhs, 0b1111);  // -1 in 4-bit signed space
    test::SetUnsigned(&rhs, 1);

    FdpOperator::DomainVector children{&lhs, &rhs};
    FdpSignedLessThanOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&lhs, &rhs, &output});
    EXPECT_TRUE(output.is_fixed_one(0));
}

TEST(FdpComparisonOperatorsTest, EqKeepsOutputsConsistent) {
    FeasibleDomain lhs(4), rhs(4), output(1);
    test::SetUnsigned(&lhs, 5);
    test::SetUnsigned(&rhs, 5);

    FdpOperator::DomainVector children{&lhs, &rhs};
    FdpEqOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&lhs, &rhs, &output});
    EXPECT_TRUE(output.is_fixed_one(0));
}

TEST(FdpCompositionOperatorsTest, ExtractCopiesBits) {
    FeasibleDomain input(8);
    test::SetUnsigned(&input, 0b10110110);
    const uint64_t high = 5;
    const uint64_t low = 2;
    const uint32_t extract_width = high - low + 1;
    FeasibleDomain output(extract_width);

    FdpOperator::DomainVector children{&input};
    FdpExtractOperator op(test::TestOperatorStats(), children, &output, low, high);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&input, &output});
    EXPECT_EQ(test::DomainValue(output),
              (0b10110110 >> 2) & test::MaskForWidth(extract_width));
}

TEST(FdpCompositionOperatorsTest, ConcatBuildsOutput) {
    FeasibleDomain high_part(4), low_part(4), output(8);
    test::SetUnsigned(&high_part, 0b1100);
    test::SetUnsigned(&low_part, 0b0011);

    FdpOperator::DomainVector children{&high_part, &low_part};
    FdpConcatOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&high_part, &low_part, &output});
    EXPECT_EQ(test::DomainValue(output), 0b11000011u);
}

TEST(FdpIteOperatorTest, GuardFixesBranch) {
    FeasibleDomain guard(1), t_branch(4), f_branch(4), output(4);
    guard.set_constant(true);
    test::SetUnsigned(&t_branch, 7);
    test::SetUnsigned(&f_branch, 1);

    FdpOperator::DomainVector children{&guard, &t_branch, &f_branch};
    FdpIteOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&guard, &t_branch, &f_branch, &output});
    EXPECT_EQ(test::DomainValue(output), 7u);
}

TEST(FdpShiftingOperatorsTest, LeftShiftPropagates) {
    FeasibleDomain value(4), shift(4), output(4);
    test::SetUnsigned(&value, 0b0011);
    test::SetUnsigned(&shift, 1);
    test::SetUnsigned(&output, 0b0110);

    FdpOperator::DomainVector children{&value, &shift};
    FdpLeftShiftOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&value, &shift, &output});
}

TEST(FdpShiftingOperatorsTest, RightShiftPropagates) {
    FeasibleDomain value(4), shift(4), output(4);
    test::SetUnsigned(&value, 0b1000);
    test::SetUnsigned(&shift, 1);
    test::SetUnsigned(&output, 0b0100);

    FdpOperator::DomainVector children{&value, &shift};
    FdpRightShiftOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&value, &shift, &output});
}

TEST(FdpShiftingOperatorsTest, ArithRightShiftMaintainsSign) {
    FeasibleDomain value(4), shift(4), output(4);
    test::SetUnsigned(&value, 0b1100);
    test::SetUnsigned(&shift, 1);
    test::SetUnsigned(&output, 0b1110);

    FdpOperator::DomainVector children{&value, &shift};
    FdpArithRightShiftOperator op(test::TestOperatorStats(), children, &output);

    Result res = op.apply();
    EXPECT_FALSE(is_conflict(res));
    test::ExpectAllInvariant({&value, &shift, &output});
}

TEST(FdpDummyOperatorTest, ApplyIsNoOp) {
    FeasibleDomain node(1);
    FdpOperator::DomainVector empty_children;
    FdpDummyOperator op(test::TestOperatorStats(), empty_children, &node);
    Result res = op.apply();
    EXPECT_TRUE(is_unchanged(res));
    test::ExpectInvariant(node);
}

}  // namespace
}  // namespace bzla::preprocess::pass::fdp

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
