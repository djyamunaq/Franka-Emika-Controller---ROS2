cmake_minimum_required(VERSION 2.8.12)
project (MotionPlanningLib)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package                (Eigen3 3.4 REQUIRED)

add_library                (trajectory_generation STATIC ${CMAKE_SOURCE_DIR}/lib/src/trajectory_generation/trajectory_generation.cpp)
target_link_libraries      (trajectory_generation Eigen3::Eigen)
target_include_directories (trajectory_generation PUBLIC ${CMAKE_SOURCE_DIR}/lib/include/trajectory_generation)

add_library                (kinematic_model STATIC ${CMAKE_SOURCE_DIR}/lib/src/kinematic_model/kinematic_model.cpp)
target_link_libraries      (kinematic_model Eigen3::Eigen)
target_include_directories (kinematic_model PUBLIC ${CMAKE_SOURCE_DIR}/lib/include/kinematic_model)

add_library                (control STATIC ${CMAKE_SOURCE_DIR}/lib/src/control/control.cpp)
target_link_libraries      (control Eigen3::Eigen  kinematic_model)
target_include_directories (control PUBLIC ${CMAKE_SOURCE_DIR}/lib/include/control)

install(TARGETS
    trajectory_generation
    kinematic_model
    control
    DESTINATION lib/${PROJECT_NAME})