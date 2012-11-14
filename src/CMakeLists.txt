rock_library(path_planner
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(path_planner_bin Main.cpp
    DEPS path_planner)

