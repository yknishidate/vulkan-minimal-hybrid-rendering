cmake_minimum_required(VERSION 3.16)

include($ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

project(vulkan_minimal_hybrid_rendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE sources src/*)
add_executable(${PROJECT_NAME} ${sources})

find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    glfw
    glm::glm
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $ENV{VULKAN_SDK}/Include
)
