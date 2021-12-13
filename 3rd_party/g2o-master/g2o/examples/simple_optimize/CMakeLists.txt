include_directories(${CSPARSE_INCLUDE_DIR})

include_directories("/usr/include/eigen3")

# 增加对C++11新特性的支持
add_definitions(-std=c++11)

add_executable(simple_optimize
  simple_optimize.cpp
)

set_target_properties(simple_optimize PROPERTIES OUTPUT_NAME simple_optimize${EXE_POSTFIX})

target_link_libraries(simple_optimize core solver_csparse)
target_link_libraries(simple_optimize types_slam2d)
target_link_libraries(simple_optimize types_slam3d)
