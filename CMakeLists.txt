cmake_minimum_required(VERSION 3.16)

include($ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

project(vulkan_minimal_hybrid_rendering LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME} 
    src/main.cpp 
    src/context.hpp src/window.hpp src/swapchain.hpp
)

find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    glfw
    glm::glm
    tinyobjloader::tinyobjloader
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $ENV{VULKAN_SDK}/Include
)
