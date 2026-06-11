project('gtest', 'cpp',
  version: '1.17.0',
  license: 'BSD-3-Clause',
  default_options: [
    'cpp_std=c++17',
    'warning_level=0',
    'default_library=static',
  ]
)

thread_dep = dependency('threads')

installed_headers = include_directories('googletest/include', 'googletest')
gmock_headers = include_directories('googlemock/include', 'googlemock')

# gtest libraries

sources_gtest = files('googletest/src/gtest-all.cc')
gtest_lib = static_library('gtest', sources_gtest,
  include_directories: installed_headers,
  dependencies: thread_dep,
  install: false,
)

gtest_dep = declare_dependency(
  link_with: gtest_lib,
  include_directories: installed_headers,
  dependencies: thread_dep,
)

gtest_main_lib = static_library('gtest_main', 'googletest/src/gtest_main.cc',
  include_directories: installed_headers,
  link_whole: gtest_lib,
  dependencies: thread_dep,
  install: false,
)

gtest_main_dep = declare_dependency(
  link_with: gtest_main_lib,
  include_directories: installed_headers,
  dependencies: thread_dep,
)

# gmock libraries

gmock_lib = static_library('gmock', 'googlemock/src/gmock-all.cc',
  include_directories: [gmock_headers, installed_headers],
  link_whole: gtest_lib,
  dependencies: thread_dep,
  install: false,
)

gmock_dep = declare_dependency(
  link_with: gmock_lib,
  include_directories: [gmock_headers, installed_headers],
  dependencies: thread_dep,
)

gmock_main_lib = static_library('gmock_main', 'googlemock/src/gmock_main.cc',
  include_directories: [gmock_headers, installed_headers],
  link_whole: [gmock_lib, gtest_lib],
  dependencies: thread_dep,
  install: false,
)

gmock_main_dep = declare_dependency(
  link_with: gmock_main_lib,
  include_directories: [gmock_headers, installed_headers],
  dependencies: thread_dep,
)

meson.override_dependency('gtest', gtest_dep)
meson.override_dependency('gtest_main', gtest_main_dep)
meson.override_dependency('gmock', gmock_dep)
meson.override_dependency('gmock_main', gmock_main_dep)
