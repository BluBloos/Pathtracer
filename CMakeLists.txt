cmake_minimum_required(VERSION 3.10)

option(RayEnableDX "use DXR" OFF)
option(RayEnableVK "use Vulkan" OFF)

set(ProjectName "Raytracer")

project(${ProjectName}
    VERSION 0.3.0
    DESCRIPTION "Dead simple raytracer."
    LANGUAGES CXX)

# set(ProjectResourcesExtraPath "${CMAKE_CURRENT_SOURCE_DIR}/assets/")
# message( STATUS "ProjectResourcesExtraPath = ${ProjectResourcesExtraPath}" )

set(ProjectExplicitResOutputDir "res")
set(ProjectRoot "${CMAKE_CURRENT_SOURCE_DIR}")

if (RayEnableVK)
    set(ProjectBackend "VK_BACKEND CPU_BACKEND")
else()
    set(ProjectBackend "DX12_BACKEND CPU_BACKEND")
endif()

#set(ProjectDisableEngineIntro ON)
set(ProjectDisableImGui ON)

add_subdirectory(Automata-Engine)
