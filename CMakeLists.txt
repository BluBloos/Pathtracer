cmake_minimum_required(VERSION 3.10)

set(ProjectName "Raytracer")

project(${ProjectName}
    VERSION 0.3.0
    DESCRIPTION "Dead simple raytracer."
    LANGUAGES CXX)

# set(ProjectResourcesExtraPath "${CMAKE_CURRENT_SOURCE_DIR}/assets/")
# message( STATUS "ProjectResourcesExtraPath = ${ProjectResourcesExtraPath}" )

set(ProjectExplicitResOutputDir "res")
set(ProjectRoot "${CMAKE_CURRENT_SOURCE_DIR}")
set(ProjectBackend "DX12_BACKEND CPU_BACKEND")
#set(ProjectDisableEngineIntro ON)
set(ProjectDisableImGui ON)

add_subdirectory(Automata-Engine)
