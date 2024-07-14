cmake_minimum_required(VERSION 3.10)

project(Pathtracer
    VERSION 0.3.0
    DESCRIPTION "Physically-based Path Tracer capable of rendering various geometrical shapes, including triangles."
    LANGUAGES CXX)

set(ProjectName "Pathtracer")

set(ProjectExplicitResOutputDir "res")
set(ProjectRoot "${CMAKE_CURRENT_SOURCE_DIR}")
set(ProjectBackend "CPU_BACKEND")
set(ProjectDisableImGui ON)

add_subdirectory(Automata-Engine)

set_property(TARGET Pathtracer PROPERTY CXX_STANDARD 20)