fdp_test_inc = include_directories('../../../src')

fdp_feasible_domain_test = executable(
  'fdp_feasible_domain_test',
  ['feasible_domain_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_feasible_domain', fdp_feasible_domain_test)

fdp_operator_dispatch_test = executable(
  'fdp_operator_dispatch_test',
  ['operator_dispatch_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_operator_dispatch', fdp_operator_dispatch_test)

fdp_operator_soundness_test = executable(
  'fdp_operator_soundness_test',
  ['operator_soundness_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_operator_soundness', fdp_operator_soundness_test)

fdp_pass_test = executable(
  'fdp_pass_feasible_domain_propagator_test',
  ['pass_feasible_domain_propagator_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_pass_feasible_domain_propagator', fdp_pass_test)

fdp_partition_task_test = executable(
  'fdp_partition_task_test',
  ['partition_task_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_partition_task', fdp_partition_task_test)

fdp_manual_test = executable(
  'fdp_manual_test',
  ['fdp_manual_test.cpp'],
  include_directories: fdp_test_inc,
  dependencies: [bitwuzla_dep, gtest_dep]
)

test('fdp_manual_test', fdp_manual_test)
