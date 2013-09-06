rock_library(simple_path_planner
    SOURCES SimplePathPlanner.cpp
    HEADERS SimplePathPlanner.hpp
    DEPS_PKGCONFIG base-lib nav_graph_search
    )

find_package(Boost REQUIRED system)
rock_executable(simple_path_planner_test_bin Main.cpp
    DEPS simple_path_planner
    DEPS_PLAIN Boost_SYSTEM)

