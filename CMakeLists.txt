project(dn)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-g -Wno-deprecated-declarations -Wno-ignored-attributes -Wno-unused-result ${OTHERS_DEFINE}")

add_subdirectory(
    submodules/roamfree/roamfree
)

include_directories(include                     
    ${EIGEN3_INCLUDE_DIR}    
    submodules/roamfree/roamfree/ROAMestimation/include
    submodules/roamfree/roamfree/ROAMimu/include
)

add_executable(
    dn
    src/main.cpp
    src/stamped_csv.cpp
    src/initial_guess_generator.cpp
)

target_link_libraries(
    dn
    ROAMestimation
    ROAMimu
)