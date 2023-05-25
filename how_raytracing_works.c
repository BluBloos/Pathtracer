/*

we begin by noting that the raytracing pipeline is a seperate one to the rasterization pipeline.

there are some fixed function elements to this pipeline:
- sorting of cast rays for coherency.
- ray tri intersection test.
- traversal of the acceleration structure, which is based on the scene geometry.

the shaders in this pipeline can:
- generate rays.
- do intersection testing that is not ray tri (procedural primitives).
  - intersection shaders.
- process ray intersection (shading or miss).

api stuff:
- bind raytracing pipeline w/ raytracing shaders.
- dispatchrays call on the command list.

the ray gen shader :
- bunch of these launched per pixel in threads. exec order is undefined.
- can generate a ray within HLSL via traceray.
- traceray returns once the ray has done all the bounces and w/ a user payload.
this payload gets modified as the ray goes.
- the output of the shader is via writing to UAV texture.
- as a thread, we know which pixel we are (kind of like a compute shader), so can easily write to UAV.
- we can bind many TLASs (to diff res slots), then pick which to use for tracing rays.

intersection shader:
- this shader is run when ray hits AABB to determine the attributes of the ray hit,
  thus defining the procedural geometry.
- if not using D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION,
the intersection shaders may run more than once/ray. apps should consider making these shaders
functional and stable.

any hit shader :
- run for any hit along the ray length(extent), for any ray.
- we get access to the ray hit attrs.
- we get to modify the payload.
- we can call ignorehit if we want (doesn't update tmax).
- we can accept the hit (this becomes tmax).
- we can continue (look at more prims).
- there is no order for a single ray and the set of intersections.
- for a single ray, only one any hit can be going. this solves the sync problem.
- this shader kind is optional and otherwise we just accept the new tmax.
- any hit shaders cannot trace new rays.

closest hit shader :
- run for the closest hit. also the terminal shader if we do not spawn rays.
- get access to the ray hit attrs.
- we can contrib to payload.
- we can launch new rays.
- can even write to UAV if we desire.
- all any hit shaders will run before the closest hit shader.
  - and the any hit can run even for the same T as closest hit.
- optional.

miss shader :
- if we hit no geometry.
- can modfiy the payload + generate more rays.

ray :
- origin, dir, len.
- rays can have multiple hit points.
- tmax will be modified to the closest hit (maybe not closest but for some other reason we terminated).
- when we use ray tri hit, we get tri barycentric coords as the hit point kind.

tlas :
- top level accel structure.
- contains set of instances of blas.
  - with metadata of world matrix + instance ID.

blas :
- bottom level accel structure.
- contains a set of triangle or procedural meshes, each described by an AABB.
  - the set can only contain one kind of element, so either tri/procedural.

accel structure general :
- an app can choose to always build from scratch or to make an AS that is "updatable".
- static structures are faster to raytrace, but updatable ones are faster to update.

shader table :
- contains shader records as elements.
- each shader record contains:
  - references to resources, etc.
  - a hit group: any hit shader, closest hit shader, intersection shader. the group can be null.
- geo in the blas point to a shader record, indicating the shaders to be run when this geo is hit.

control flow :=
- any hit isn't guaranteed to run on all points. it only runs for cloest hits thus far.
- there is a backpath to the intersection shader where it goes if one calls ReportHit.

cool things :=
- accel structure can be serialized to file.
- there are cool ray flags that support some lovely optimizations.
- from within a raytracing shader, you can literally call another shader! what!? that's advanced.

background :=

seems there is this notion of coherent versus incoherent information. where the ray before bounce is 
coherent, but after bounce is incoherent.
  this sort of makes sense, where the information maps from a coherent space to an incoherent space.
  and the coherency is a metric on the spatial locality if the data.

so it would be interesting to understand precisely what is being sorted and by what specific metric.


TODO:
- complete walkthrough reading.
- look at API.
- look at HLSL.
- maybe read sections on limitations.
- read general tips for building accel structures.

- read potential future features.






 */




/*
DXR 1.1 

- can dispatch rays from an execute indirect.
- inline raytracing support.

 */
