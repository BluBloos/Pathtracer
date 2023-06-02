/*

for simplicity, we are going to target Vulkan v1.2

SPV_KHR_ray_tracing
SPV_KHR_ray_query

require spirv v1.4?

VK_KHR_acceleration_structure
  has dependencies of :
    VK_KHR_deferred_host_operations
  we also have dependecy features of :
    accelerationStructure
    descriptorBindingAccelerationStructureUpdateAfterBind
    bufferDeviceAddress
    descriptorIndexing
      and all the features required by this feature

VK_KHR_ray_tracing_pipeline
  has dependencies of : 
    rayTracingPipeline
    VK_KHR_acceleration_structure
    rayTracingPipelineTraceRaysIndirect
    rayTraversalPrimitiveCulling, if VK_KHR_ray_query is also supported
    VK_KHR_pipeline_library (soft requirement, needed only if used)

VK_KHR_ray_query
  has depends of : 
    VK_KHR_acceleration_structure
    rayQuery

VK_KHR_pipeline_library

TODO: VkAccelerationStructureKHR and VkAccelerationStructureNV handles are no longer aliases
^ so we want to look into that. and in general, take a look at VK_NV_ray_tracing.


BEGIN
===
===
=== begin what? the writeup of how rt works on vk, of course.


ACCEL STRUCTURE ::
==================

just like DXR, we create an acceleration structure. this is done within a VkBuffer.
there are the same kinds,
    
    VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR
    VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR

seems another part is also similar, the querying of the prebuild info -> 

    vkGetAccelerationStructureBuildSizesKHR.

the prebuild info comes back in the form of :

    VkAccelerationStructureBuildSizesInfoKHR

like as with the DXR side of things, we need not specify much things up front. i.e., we do not
require the data, just a description of the data that will be used to build the acceleration
structure.

    VkAccelerationStructureBuildGeometryInfoKHR

we put such description in the structure above.

now, there seems to be this idea of a compacting copy. not sure yet what that is, but there is
what seems to be a similar call where we first query required sizes.

    vkCmdWriteAccelerationStructuresPropertiesKHR

building
=======

so we create a Vulkan object of kind

    VkAccelerationStructureKHR

with the command

    vkCmdBuildAccelerationStructuresKHR

this doesn't actually init the data of the acceleration structure.
this is following the common VK pattern where you create the "object" of the thing.
the "object" isn't the data, just a ref to it (with metadata, shape).
once we create the ref we need to have it do the pointer, so we "bind" some backing mem.

however, in this case there is a slight deviation from the common pattern, where
we provide the backing buffer info upfront, baked into the object creation call.
this is via the 

    VkAccelerationStructureCreateInfoKHR

structure.

so, to actually create the acceleration structure, this is later done via:

    vkBuildAccelerationStructuresKHR            (CPU side build)
    vkCmdBuildAccelerationStructuresKHR         (GPU side build)
    vkCmdBuildAccelerationStructuresIndirectKHR (something suspicious)

once we have the build complete, the data used to make it (the instance data, e.g.)
is no longer needed. the acceleration structure is completely self-contained ...
something that the VK docs do a much better job of explaining than the DX docs!!!!

blas
====

same thing, contains the triangles or AABB for custom procedural geometry.

tlas
====

contains instances of blas, each ref with tranform and shading properties.


DEFERRED HOST(cpu) OPS ::
====================

there seems to be this thing of "deferred host operations". this is where I can use the CPU to build
the acceleration structure as opposed to the GPU. and I can have that be a multithreaded thing. 
and it goes as where I can be rendering a frame and building an acceleration structure at the same time,
presumably where I am building the acceleration structure one full frame in advance.
the exact API calls go as: 

   vkBuildAccelerationStructuresKHR
   vkCreateRayTracingPipelinesKHR

where we notice that we can also do the parallel pipeline creation as well.


RESOURCE USAGE AND SYNC ::
==========================

the resources need the following usage bits,

accel structures                : VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR
scratch space                   : VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
inputs (such as verts e.g.)     : VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR
shading binding table           : VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR


accel build stage               : VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR
access as accel structure       : VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR
                                  \  VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR
access to inputs for the build  : VK_ACCESS_SHADER_READ_BIT
copy accel structure stage      : VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR
access buffer via device addr   : VK_ACCESS_TRANSFER_READ_BIT
                                  \  VK_ACCESS_TRANSFER_WRITE_BIT
trace ray cmd stage             : VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR
access shading binding talbe    : VK_ACCESS_SHADER_READ_BIT


TODO: when copy an accel structure, can I not just do a regular buffer copy?
apparently we need to use 

    vkCmdCopyAccelerationStructureKHR


shader binding table ::
====

this is a shader group handle (that we get back from the impl) for a given shader group.
each entry is also plus an additional shader buffer record which contains instance specific data
(some constants or something, for example).

the address into this table is sort of like DXR, where you have many different contributions.
contrib from cmd list call, contrib from the shader call, and contrib from the specific instance.

==================================== other stuff.

when creating the raytracing pipeline, we can specify the max stack size. interestingly, a stack is used
because we have shaders calling other shaders and so forth.

looks maybe that on the shader end that we have some 

    OpTraceRayKHR \ traceRaysEXT

for doing the tracing within the shader. this like DXR takes the acceleration structure to trace into.
it also takes a Payload parameter.

apparently, we can use 

    vkGetAccelerationStructureDeviceAddressKHR

and so instead of a descriptor, we could literally use the address direct from within the shader?

when it comes to the actual shader side of things, there does indeed seem to be
an any hit shader, an intersection shader, a closest hit shader, and prob a miss shader + raygen shader as well.

interestingly, there seems to be the same sort of

    ShaderRecordBufferKHR

idea that is going on. so I suspect this is where we put the info regarding our shader
stuff that constitute the pipeline.

and it seems like when making the call to trace the rays you provide the "shader binding tables"
via their gpuva address, and this goes as :

    VkStridedDeviceAddressRegionKHR





TODO: look into compacting the acceleration structure.

TODO: what is going on with the nullDescriptor support for AS?

TODO: look into the ray query stuff. for now, I think this is similar to the DXR 1.1. stuff.

TODO: there is lots going on with indirect things, look into that.

TODO: look into the save & restore of raytracing acceleration structures.

*/
