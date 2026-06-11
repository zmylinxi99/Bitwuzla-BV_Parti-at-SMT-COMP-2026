#pragma once

#include <gtest/gtest.h>

#include "backtrack/assertion_stack.h"
#include "backtrack/backtrackable.h"
#include "env.h"
#include "node/node_manager.h"
#include "option/option.h"

namespace bzla::test {

class TestPreprocessingPass : public ::testing::Test {
  public:
    TestPreprocessingPass() = default;

  protected:
    NodeManager d_nm;
    option::Options d_options;
    backtrack::BacktrackManager d_bm;
    backtrack::AssertionStack d_as;
};

}  // namespace bzla::test
