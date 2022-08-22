cmake_minimum_required(VERSION 3.16)

project(vulkan_minimal_hybrid_rendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

file(GLOB_RECURSE sources src/*)
add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME} PUBLIC glfw)

target_include_directories(${PROJECT_NAME} PUBLIC
    $ENV{VULKAN_SDK}/Include
    ${PROJECT_SOURCE_DIR}/external/glm
)
