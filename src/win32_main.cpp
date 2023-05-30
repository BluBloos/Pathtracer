#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
#include "ray_dxr.h"
#include <automata_engine.hpp>

#include <synchapi.h>

#include <comdef.h>
#include <windows.h>


#include <D3d12.h>
#include <dxgi1_6.h>
#include <D3dx12.h>

#define COM_RELEASE(comPtr) (comPtr != nullptr) ? comPtr->Release() : 0; 

/*
Next steps for the software overall:

1. A greater degree in the sophistication of the simulation.

    - add light sources and shadows
    - "do the raytrace proper" = have your simulation conform more to the underlying Physics
    - ray propagation in the medium = volumetric light transport = fog

2. "Do the ratracer proper":

    - Good resource:
        - http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/22mcrt.pdf
    - Cast rays from our pixels. Hit some point.
    - consider the light transport equation
        - what we see for the ray intersection point is
            - emitted radiance from that point by object hit + light contrib to that point from rest of scene
    - How much light is arriving at this point?
        - Direct illumination:
            - Cast a ray for each light (shadow cast). Can, from this, determine how much
            light is contributing to this hit point.
                - The amount of light contrib is based on irradiance amount (light "intensity")
                and the BSDF func (defining object material).
            - point lights cast hard shadows
            - area lights case soft shadows
        - Global illumination:
            - Ambient light of the scene
            - light transport = bounces of light through the scene
            - energy decay across space due to scattering = attenuation
            - absorption of energy into volumes = volumetric lighting stuff = fog
            - refraction rays too!
        - depth of field
            - use a lens (with real width) over a pinhole camera (aperature is infinitely small)
        - monte-carlo integration:
            - support area light integration for soft shadows by casting many shadow rays 
            - support rough surfaces that create many normals = jitter reflected rays
            - shoot random rays through the lens
            - the 101 for monte-carlo integration:
                - Plz read here: https://graphics.stanford.edu/courses/cs348b-01/course29.hanrahan.pdf 
    
3. Ultimate desired demo = Sponza scene.
*/


/*

  changes required for the DXR version:

  - for starters, we know that all the geometry in our scene is opaque, so we will not need the any hits.
  - we are using procedural shapes, so we'll need to use the intersection shader.
  - our closest hit shader is going to write to UAV if it is the 8th bounce. this shader will spawn shaders,
  unless we have hit the bounce count. that's the terminal condition.
  this is gonna be the meat and is going to replace the bulk of what we have below.
  - the miss shader should just give the sky color.
  - we're gonna actually keep trying to do the "texel" thing. this can be done by doing the DispatchRays call
  per texel.
  - the ray gen shader is where we do the 256 ray_pp idea. I think this is literally going to do nothing where
  it just calls TraceRay.

  - we'll use two blas and one tlas. the blas kinds are inf plane and sphere, each procedural geometry.
  the tlas will be the five instances of those that are currently in the scene.  

  - so without looking into the details of DXR; I see two options for how we can have two different materials.
  1. we can use wholly different shaders.
  2. if the metadata of instances is permissive enough, we can use this to encode our material parameters.
  most importantly, the scatter factor; which controls how random of a bounce to do.

 */ 



#define DXR_MODE 1

// number of render threads to spawn.
#define THREAD_COUNT 7

// TODO: we need to free many of the resources in this structure.
struct game_data {

  // some stupid shit.
  ID3D12Device *d3dDevice = NULL;
  ID3D12Debug *debugController = NULL;

  // pipeline stuff.
  ID3D12PipelineState *computePipelineState = NULL;
  ID3D12RootSignature *rootSig = NULL;
  ID3D12StateObject *rayStateObject = NULL;

  // resources.
  ID3D12Resource *gpuTex = NULL;
  ID3D12Resource *cpuTex = NULL;

  // stuff for raytracing.
  ID3D12Resource *sceneBuffer = NULL;
  ID3D12Resource *rayShaderTable = NULL;
  ID3D12Resource *tlas = NULL;
  ID3D12Resource *BLASes[2] = {}; // TODO: 2 constant is hacky.

  ID3D12Resource *tlasScratch = NULL;
  ID3D12Resource *blasScratches[2] = {}; // TODO: 2 constant is hacky.

  ID3D12Resource *AABBs = NULL;
  ID3D12Resource *tlasInstances = NULL;

  ID3D12DescriptorHeap *descHeap = NULL;

  // for submitting shit.
  ID3D12CommandQueue *commandQueue = NULL;

  // main command list shit for doing some of the setup work.
  // after the setup work we also use this for the UAV copy to game backbuffer.
  ID3D12CommandAllocator *commandAllocator = NULL;
  ID3D12GraphicsCommandList *commandList = NULL;
  ID3D12Fence *fence = NULL;
  int fenceValue;
  HANDLE fenceEvent;

  // we keep around cmd lists and fences for each ray render thread
  // so that we can do the dispatching.
  ID3D12CommandAllocator *rayCmdAllocators[THREAD_COUNT] = {};
  ID3D12GraphicsCommandList *rayCmdLists[THREAD_COUNT] = {};
  ID3D12Fence *rayFences[THREAD_COUNT] = {};
  int rayFenceValues[THREAD_COUNT] = {};
  HANDLE rayFenceEvents[THREAD_COUNT] = {};
};

static game_data *getGameData(ae::game_memory_t *gameMemory) {
  return (game_data *)gameMemory->data;
}

static unsigned int GetTotalPixelSize(image_32_t image) {
  return image.height * image.width * sizeof(unsigned int);
}

static image_32_t AllocateImage(unsigned int width, unsigned int height) {
    image_32_t newImage = {};
    newImage.width = width;
    newImage.height = height;
    unsigned int totalPixelSize = GetTotalPixelSize(newImage);
    newImage.pixelPointer = (unsigned int *)malloc(totalPixelSize);
    return newImage;
}

static void WriteImage(image_32_t image, char *fileName) {
    unsigned int outputPixelSize = GetTotalPixelSize(image);
    bitmap_header_t header = {};
    header.FileType = 0x4D42;   
    header.FileSize = sizeof(bitmap_header_t) + outputPixelSize;
    header.BitmapOffset = sizeof(header); 
    header.size = 40;    
    header.Width = image.width;  
    header.Height = image.height;
    header.Planes = 1;          
    header.BitsPerPixel = 32;    
    FILE *fileHandle = fopen("test.bmp", "wb");
    if(fileHandle) {
        fwrite(&header, sizeof(header), 1, fileHandle);
        fwrite(image.pixelPointer, outputPixelSize, 1, fileHandle);
        fclose(fileHandle);
    }
    else {
        fprintf(stderr, "[ERROR] Unable to write output file\n");
    }
}

static v3 RayCast(world_t *world, v3 rayOrigin, v3 rayDirection) {
    v3 result = {};
    float tolerance = 0.0001f;
    float minHitDistance = 0.001f;
    v3 attenuation = V3(1, 1, 1);
    for (unsigned int bounceCount = 0; bounceCount < 8; ++bounceCount) {
        float hitDistance = FLT_MAX;
        unsigned int hitMatIndex = 0;
        v3 nextNormal = {};
        // sphere intersection test.
        for (
            unsigned int sphereIndex= 0;
            sphereIndex < world->sphereCount;
            sphereIndex++
        ) {
            sphere_t sphere = world->spheres[sphereIndex];  
            v3 sphereRelativeRayOrigin = rayOrigin - sphere.p;
            float a = Dot(rayDirection, rayDirection);
            float b = 2.0f * Dot(sphereRelativeRayOrigin, rayDirection);
            float c = Dot(sphereRelativeRayOrigin, sphereRelativeRayOrigin) 
                - sphere.r * sphere.r;
            float denom = 2.0f * a;
            float rootTerm = SquareRoot(b * b - 4.0f * a * c);
            if (rootTerm > tolerance){
                // NOTE(Noah): The denominator can never be zero
                float tp = (-b + rootTerm) / denom;
                float tn = (-b - rootTerm) / denom;   
                float t = tp;
                if ((tn > minHitDistance) && (tn < tp)){
                    t = tn;
                }
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = sphere.matIndex;
                    nextNormal = Normalize(t*rayDirection + sphereRelativeRayOrigin);
                }
            }
        }
        // floor intersection test
        for (
            unsigned int planeIndex = 0;
            planeIndex < world->planeCount;
            planeIndex++
        ) {
            plane_t plane = world->planes[planeIndex];  
            float denom = Dot(plane.n, rayDirection);
            if ((denom < -tolerance) || (denom > tolerance)) {
                float t = (-plane.d - Dot(plane.n, rayOrigin)) / denom;
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = plane.matIndex;
                    nextNormal = plane.n;
                }
            } 
        }
        if (hitMatIndex) {
            material_t mat = world->materials[hitMatIndex];
            //TODO(Noah): Do real reflectance stuff
            result = result + Hadamard(attenuation, mat.emitColor);
            attenuation = Hadamard(attenuation, mat.refColor);            
            rayOrigin = rayOrigin + hitDistance * rayDirection;
            //NOTE(Noah): this does a reflection thing
            //TODO(Noah): these are not accurate permutations
            v3 pureBounce = 
                rayDirection - 2.0f * Dot(nextNormal, rayDirection) * nextNormal;
            v3 randomBounce = Normalize(
                nextNormal + V3(
                    RandomBilateral(),
                    RandomBilateral(),
                    RandomBilateral()
                )
            );
            rayDirection = Normalize(Lerp(randomBounce, pureBounce, mat.scatter));
        }
        else {
          // sky contrib and terminate all future bounces;
            material_t mat = world->materials[hitMatIndex];
            result = result + Hadamard(attenuation, mat.emitColor);
            break;
        }
    }
    return result;   
}

static world_t world = {};
// populate word with floor and spheres.
static material_t materials[4] = {};
static plane_t planes[1] = {};
static sphere_t spheres[3] = {};
static v3 cameraP = {};
static v3 cameraZ = {};
static v3 cameraX = {};
static v3 cameraY = {};
static float filmDist = 1.0f;
static float filmW = 1.0f;
static float filmH = 1.0f;
static float halfFilmW;
static float halfFilmH;
static v3 filmCenter;
float halfPixW;
float halfPixH;
unsigned int g_raysPerPixel = 256;
image_32_t image = {};

std::mutex g_commandQueueMutex;

void blitToGameBackbuffer(ae::game_memory_t *gameMemory) {
#define UPDATE_FAIL_CHECK()                                                    \
  if (hr != S_OK)                                                              \
    return;

    HRESULT hr;

    auto gd = getGameData(gameMemory);

    {
        std::lock_guard<std::mutex> lock(g_commandQueueMutex);

        // Execute the command list.
        ID3D12CommandList* ppCommandLists[] = { gd->commandList };
        gd->commandQueue->ExecuteCommandLists(1, ppCommandLists);

        // Schedule a Signal command in the queue.
        hr = (gd->commandQueue->Signal(gd->fence, ++gd->fenceValue));
        UPDATE_FAIL_CHECK();
    }
    // wait for that work to complete:
    {
        hr = (gd->fence->SetEventOnCompletion(gd->fenceValue, gd->fenceEvent));
        UPDATE_FAIL_CHECK();
        WaitForSingleObjectEx(gd->fenceEvent, INFINITE, FALSE);
    }

    // NOTE: the tex that we get from read subresource has been swizzled in the
    // shader to match our surface format.

    D3D12_BOX box = {};
    box.right = gameMemory->backbufferWidth;
    box.bottom = gameMemory->backbufferHeight;
    box.back = 1; // depth of one.

    UINT rowPitch = sizeof(uint32_t) * gameMemory->backbufferWidth;
    hr = gd->cpuTex->ReadFromSubresource(gameMemory->backbufferPixels, rowPitch,
                                        rowPitch * gameMemory->backbufferHeight,
                                        0, // src subresource.
                                        &box);

#undef UPDATE_FAIL_CHECK
}

void visualizer(ae::game_memory_t *gameMemory) {

#if DXR_MODE

    blitToGameBackbuffer(gameMemory);

#else
    // TODO(Noah): Right now, the image is upside-down. Do we fix this on the
    // application side or is this something that we can fix on the engine side?
    memcpy((void *)gameMemory->backbufferPixels, image.pixelPointer,
           sizeof(uint32_t) * gameMemory->backbufferWidth *
               gameMemory->backbufferHeight);

#endif // DXR_MODE
}

void automata_engine::HandleWindowResize(game_memory_t *gameMemory, int nw,
                                         int nh) {}

void automata_engine::PreInit(game_memory_t *gameMemory) {
    ae::defaultWinProfile = AUTOMATA_ENGINE_WINPROFILE_NORESIZE;
    ae::defaultWindowName = "Raytracer";
    ae::defaultWidth  = 1280;
    ae::defaultHeight = 720;
}

typedef struct texel {
    int width;
    int height;
    uint32_t xPos;
    uint32_t yPos;
} texel_t;

struct render_thread_data {
    std::tuple<texel_t *, uint32_t> *pTexelTuple;
    unsigned int threadIdx;
    game_data *gd;
};

// TODO: for therse two functions below, do we need to schedule a release???
// pretty sure we do, and query interface increases the reference count of the
// thing.
ID3D12GraphicsCommandList4 *getRayCmdList(ID3D12GraphicsCommandList *cmd) {
    ID3D12GraphicsCommandList4 *r = nullptr;
    cmd->QueryInterface(&r);
    assert(r);
    return r;
}
ID3D12Device5 *getRayDevice(ID3D12Device *device) {
    ID3D12Device5 *r = nullptr;
    device->QueryInterface(&r);
    assert(r);
    return r;
}

// NOTE(Noah): 
// Here's some documentation on some of the first multithreading bugs I have ever encountered!
// 
// It seems like the threads are overlapping (drawing to same texels).
// And the stagger pattern at the beginning (looked like staircase going down from left -> right)
// is the overlapping threads having their ptr_to_texels array updated. stagger pattern ends with
// texels being draw in what appear to be those belonging to the very last thread group.
//
// In fact, this is exactly what is happening. we pass a pointer, which is going to be the same one
// every time and the threads are all in the same virtual memory space. So next time thru the loop,
// the texels arr is updated for all threads.
DWORD WINAPI render_thread(_In_ LPVOID lpParameter) {

    auto threadData = (struct render_thread_data *)lpParameter;
    auto gd = threadData->gd;

    std::tuple<texel_t *, uint32_t> *ptr_to_tuple = threadData->pTexelTuple;

    texel_t *texels = std::get<0>(*ptr_to_tuple);

    for (uint32_t i = 0; i < std::get<1>(*ptr_to_tuple); i++) {
        texel_t texel = texels[i];

#if DXR_MODE
        HRESULT hr;
        // record command list.
        auto tIdx = threadData->threadIdx;
        auto cmd = getRayCmdList(gd->rayCmdLists[tIdx]);
        defer(COM_RELEASE(cmd));

        // dispatch rays for texel.
        cmd->SetPipelineState1(gd->rayStateObject);

        D3D12_DISPATCH_RAYS_DESC rayDesc = {};

        //   D3D12_GPU_VIRTUAL_ADDRESS_RANGE
        rayDesc.RayGenerationShaderRecord = {
            gd->rayShaderTable->GetGPUVirtualAddress(),
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT // size
        };

        // D3D12_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE
        rayDesc.MissShaderTable = {
            gd->rayShaderTable->GetGPUVirtualAddress() +
                D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT, // size
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT  // stride
        };

        // D3D12_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE
        rayDesc.HitGroupTable = {
            gd->rayShaderTable->GetGPUVirtualAddress() +
                2 * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
            //TODO: this constant of 2 here is hacky.
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT * 2, // size
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT  // stride
        };

        rayDesc.Width = texel.width;
        rayDesc.Height = texel.height;
        rayDesc.Depth = 1;

        // bind the other crap so the shaders actually know where to get the
        // resources, etc. bind tlas too.
        {
            cmd->SetComputeRootSignature(gd->rootSig);
            cmd->SetDescriptorHeaps(1, &gd->descHeap);
            cmd->SetComputeRootDescriptorTable(
                0, gd->descHeap->GetGPUDescriptorHandleForHeapStart());
            struct {
              unsigned int xPos;
              unsigned int yPos;
              unsigned int seed;
              unsigned int width;
              unsigned int height;
            } constants = {texel.xPos, texel.yPos,
                           (unsigned int)(ae::timing::epoch()),
                           image.width, image.height
            };
            cmd->SetComputeRoot32BitConstants(
                                             1, //root param.
                                             5, //num 32 bit vals to set.
                                             &constants,
                                             0  //offset.
                                             );
            cmd->SetComputeRootShaderResourceView(2, gd->tlas->GetGPUVirtualAddress());
        }

        // TODO: even if my thing is working right now, it's not working. I need
        // to dispatch these with a viewport or something. otherwise, all texels
        // will write to the top-left one and overwrite each other.
        cmd->DispatchRays(&rayDesc);

        cmd->Close();

        // only one thread can submit to queue at a time.
        {
            std::lock_guard<std::mutex> lock(g_commandQueueMutex);
            ID3D12CommandList *ppCommandLists[] = {gd->rayCmdLists[tIdx]};
            gd->commandQueue->ExecuteCommandLists(1, ppCommandLists);
            // Schedule a Signal command in the queue.
            hr = (gd->commandQueue->Signal(gd->rayFences[tIdx],
                                           ++gd->rayFenceValues[tIdx]));
            if (hr != S_OK)
                throw;
        }

        // use a render thread local fence to wait on that work.
        {
            hr = (gd->fence->SetEventOnCompletion(gd->rayFenceValues[tIdx],
                                                  gd->rayFenceEvents[tIdx]));
            if (hr != S_OK)
                throw;
            WaitForSingleObjectEx(gd->rayFenceEvents[tIdx], INFINITE, FALSE);

            // TODO: this solves some "cmd list allocator reset" ideas.
            // but also, if I just let those happen, the app still goes to completion and I get
            // the raytraced image.
            //
            // so, there seems to be some funny business going on. we should definitely investigate
            // that!
            //
            //Sleep(500); 
        }

        // reset list and allocators.
        // Command list allocators can only be reset when the associated
        // command lists have finished execution on the GPU.
#if 0
        char buf[4096];
        sprintf(buf, "now reset allocator with tIdx: %d, fenceVal: %d\n", tIdx, gd->rayFenceValues[tIdx]);
        ::OutputDebugStringA(buf);
#endif
        (gd->rayCmdAllocators[tIdx]->Reset());
        (gd->rayCmdLists[tIdx]->Reset(gd->rayCmdAllocators[tIdx],
                                      nullptr // initial pipeline state.
                                      ));
#else
        unsigned int *out =
            image.pixelPointer + texel.yPos * image.width + texel.xPos;
        // Raytracer works by averaging all colors from all rays shot from this
        // pixel.
        for (unsigned int y = texel.yPos; y < (texel.height + texel.yPos);
             y++) {
            float filmY = -1.0f + 2.0f * (float)y / (float)image.height;
            for (unsigned int x = texel.xPos; x < (texel.width + texel.xPos);
                 x++) {
                float filmX = -1.0f + 2.0f * (float)x / (float)image.width;
                v3 color = {};
                float contrib = 1.0f / (float)g_raysPerPixel;
                for (unsigned int rayIndex = 0; rayIndex < g_raysPerPixel;
                     rayIndex++) {
                    float offX = filmX + (RandomBilateral() * halfPixW);
                    float offY = filmY + (RandomBilateral() * halfPixH);
                    v3 filmP = filmCenter + (offX * halfFilmW * cameraX) +
                               (offY * halfFilmH * cameraY);
                    v3 rayOrigin = cameraP;
                    v3 rayDirection = Normalize(filmP - cameraP);
                    color = color +
                            contrib * RayCast(&world, rayOrigin, rayDirection);
                }
                v4 BMPColor = {255.0f * LinearToSRGB(color.r),
                               255.0f * LinearToSRGB(color.g),
                               255.0f * LinearToSRGB(color.b), 255.0f};
                unsigned int BMPValue = BGRAPack4x8(BMPColor);
                *out++ = BMPValue; // ARGB
            }
            out += image.width - texel.width;
        }
#endif

        // TODO(Noah): It seems that the thread continues even after we close
        // the window?? (we had prints that were showing) it could be the
        // terminal doing a buffering thing, and just being slow. OR, it could
        // be that the threads are for reals still alive?? If it is the second
        // one, this is a cause for concern.
    }
    ExitThread(0);
}

// TODO: once a thread completes work it should steal texels from other threads still working.
//
// on a per-thread basis, we could maybe introduce cpu SIMD.

DWORD WINAPI master_thread(_In_ LPVOID lpParameter) {

    auto gameMemory = (ae::game_memory_t *)lpParameter;

#define THREAD_GROUP_SIZE 32
#define PIXELS_PER_TEXEL (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)
    uint32_t maxTexelsPerThread =
        (uint32_t)ceilf((float)(image.width * image.height) /
                        (float)(PIXELS_PER_TEXEL * THREAD_COUNT));
    AELoggerLog("maxTexelsPerThread: %d", maxTexelsPerThread);
    {
        HANDLE threadHandles[THREAD_COUNT];
        uint32_t xPos = 0;
        uint32_t yPos = 0;
        // TODO(Noah): Could do entire image as BSP tree -> assign threads to
        // these regions. then break up these regions into texels.
        texel_t *texels = nullptr;

        std::tuple<texel_t *, uint32_t> texelParams[THREAD_COUNT];
        render_thread_data renderThreadData[THREAD_COUNT];

        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            for (uint32_t j = 0; j < maxTexelsPerThread; j++) {
                texel_t texel;
                texel.width = THREAD_GROUP_SIZE;
                texel.height = THREAD_GROUP_SIZE;
                texel.xPos = xPos;
                texel.yPos = yPos;
                xPos += THREAD_GROUP_SIZE;
                bool isPartialTexel = false;
                if (texel.yPos + texel.height > image.height) {
                    texel.height -= (texel.yPos + texel.height) - image.height;
                    texel.height = ae::math::max(texel.height, 0);
                    isPartialTexel = true;
                }
                if (xPos >= image.width) {
                    if (xPos > image.width) {
                      texel.width -= xPos - image.width;
                      texel.width = ae::math::max(texel.width, 0);
                      isPartialTexel = true;
                    }
                    xPos = 0;
                    yPos += THREAD_GROUP_SIZE;
                }
                if (texel.xPos >= 0 &&
                    (texel.xPos + texel.width <= image.width) &&
                    texel.yPos >= 0 &&
                    (texel.yPos + texel.height <= image.height)) {
                    if (isPartialTexel)
                      j--; // NOTE(Noah): This is hack ...
                    StretchyBufferPush(texels, texel);
                } else {
                    AELoggerWarn("found invalid texel:");
                    AELoggerLog("with xPos: %d", texel.xPos);
                    AELoggerLog("with yPos: %d", texel.yPos);
                    AELoggerLog("with width: %d", texel.width);
                    AELoggerLog("with height: %d", texel.height);
                }
            }
        }
        // NOTE(Noah): The reason we split up the for-loop is because texels
        // base addr is not stable until we have finished pushing (this is due
        // to stretchy buff logic).
        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            texelParams[i] = std::make_tuple(texels + i * maxTexelsPerThread,
                                             (i + 1 < THREAD_COUNT)
                                                 ? maxTexelsPerThread
                                                 : StretchyBufferCount(texels) -
                                                       (THREAD_COUNT - 1) *
                                                           maxTexelsPerThread);

            auto &rtd = renderThreadData[i];
            rtd.pTexelTuple = &texelParams[i];
            rtd.threadIdx = i;
            rtd.gd = getGameData(gameMemory);

            threadHandles[i] =
                CreateThread(nullptr,
                             0, // default stack size.
                             render_thread, (LPVOID)&rtd,
                             0, // thread runs immediately after creation.
                             nullptr);
        }
        // wait for all threads to complete.
        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            WaitForSingleObject(threadHandles[i], INFINITE);
        }
        StretchyBufferFree(texels);
    }

    image_32_t img = {};
#if DXR_MODE
    // NOTE: this is a little hacky but I don't think its TOO bad. it could of
    // course
    //  be better. but anyways, we're going to sleep this thread for just a
    //  little bit. in doing so, we allow the render thread to "catch up" and
    //  fetch to the game backbuffer the very last image. the final image is
    //  const, so doesn't matter if the render thread fetches many time. just
    //  need it to have fetched the final at least once.
    Sleep(500 // ms
    );
    img.width = gameMemory->backbufferWidth;
    img.height = gameMemory->backbufferHeight;
    img.pixelPointer = gameMemory->backbufferPixels;
#else
    img = image;
#endif
    WriteImage(img, "test.bmp");
    AELoggerLog("Done. Image written to test.bmp\n");
    ExitThread(0);
}

struct upload_buffer {
    ID3D12Resource *src;
    ID3D12Resource *dst;
    UINT64 size;
    unsigned int initState;
};

struct upload_buffer_helper {
    static constexpr unsigned int capacity = 10;
    upload_buffer uBuffers[capacity];
    unsigned int ubCount = 0;

    upload_buffer &operator[](size_t index) {
        assert(index < capacity);
        return uBuffers[index];
    }

    void flush(game_data *gd) {
        auto &cmd = gd->commandList;

        for (int i = 0; i < ubCount; i++) {
            auto &ub = uBuffers[i];
            if (ub.src && ub.dst) {
                // NOTE: we assume state before of COPY_DEST.
                cmd->CopyBufferRegion(ub.dst, 0, ub.src, 0, ub.size);
                cmd->ResourceBarrier(1,
                                     &CD3DX12_RESOURCE_BARRIER::Transition(
                                         ub.dst, D3D12_RESOURCE_STATE_COPY_DEST,
                                         (D3D12_RESOURCE_STATES)ub.initState));
            }
        }

        // TODO: filling the upload buffer after flush overwrites the .src,
        // resulting in being unable to Release the resource.
        //
        // so, iter over the upload buffers is not a realiable way to free em'
        ubCount = 0;
    }

    upload_buffer &curr() { return uBuffers[ubCount]; }

    void iter() { ubCount++; }
};

void automata_engine::Init(game_memory_t *gameMemory) {
    AELoggerLog("Doing stuff...\n");

    // early init of structures so that we can also copy into CBV,
    // among other initialization needs.
    materials[0].emitColor = V3(0.1f, 0.0f, 0.1f);
    //    materials[0].emitColor = V3(0.3f, 0.4f, 0.5f);
    materials[1].refColor = V3(0.5f, 0.5f, 0.5f);
    //materials[1].emitColor = V3(0.5f, 0.5f, 0.5f);
    materials[1].scatter=1;
    materials[2].refColor = V3(0.7f, 0.25f, 0.3f);
    //materials[2].emitColor = V3(0.7f, 0.25f, 0.3f);
    materials[2].scatter=1;
    materials[3].refColor = V3(0.0f, 0.8f, 0.0f);
    //materials[3].emitColor = V3(0.0f, 0.8f, 0.0f);
    materials[3].scatter = 1.0f;
    planes[0].n = V3(0, 0, 1);
    planes[0].d = 0; // plane on origin
    planes[0].matIndex = 1;
    spheres[0].p = V3(0,0,0);
    spheres[0].r = 1.0f;
    spheres[0].matIndex = 2;
    spheres[1].p = V3(-3,0,-2);
    spheres[1].r = 1.0f;
    spheres[1].matIndex = 2;
    spheres[2].p = V3(-2,2,0);
    spheres[2].r = 1.0f;
    spheres[2].matIndex = 3;
    world.materialCount = ARRAY_COUNT(materials);
    world.materials = materials;
    world.planeCount = ARRAY_COUNT(planes);
    world.planes = planes;
    world.sphereCount = ARRAY_COUNT(spheres);
    world.spheres = spheres;

    assert(world.sphereCount <= DXR_WORLD_LIMIT);
    assert(world.planeCount <= DXR_WORLD_LIMIT);
    assert(world.materialCount <= DXR_WORLD_LIMIT);
    
    auto gd = getGameData(gameMemory);

    HRESULT hr;
#define INIT_FAIL_CHECK()                                                      \
  if (hr != S_OK) {                                                            \
    AELoggerError("INIT_FAIL_CHECK failed at line=%d with hr=%x", __LINE__,    \
                  hr);                                                         \
    return;                                                                    \
  }

    // init the d3d device (and debug stuff)
    {

        UINT dxgiFactoryFlags = 0;

#if defined(_DEBUG)
        {
            if (SUCCEEDED(D3D12GetDebugInterface(
                    IID_PPV_ARGS(&gd->debugController)))) {
                gd->debugController->EnableDebugLayer();
                dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
            }
        }
#endif

        IDXGIFactory2 *dxgiFactory = NULL;
        hr = CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&dxgiFactory));
        INIT_FAIL_CHECK();
        defer(COM_RELEASE(dxgiFactory));

        IDXGIAdapter1 *hardwareAdapter = nullptr;
        ae::DX::findHardwareAdapter(dxgiFactory, &hardwareAdapter);
        if (!hardwareAdapter)
          return;
        defer(COM_RELEASE(hardwareAdapter));

        (hr = D3D12CreateDevice(hardwareAdapter,
                                D3D_FEATURE_LEVEL_12_0, // minimum feature level
                                IID_PPV_ARGS(&gd->d3dDevice)));
        INIT_FAIL_CHECK();
    }

    // create the root sig.
    {

        // Determine supported root sig version.
        D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData = {};
        featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1;
        if (FAILED(gd->d3dDevice->CheckFeatureSupport(
                D3D12_FEATURE_ROOT_SIGNATURE, &featureData,
                sizeof(featureData)))) {
          featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;
        }

        CD3DX12_DESCRIPTOR_RANGE1 ranges[2];
        CD3DX12_ROOT_PARAMETER1 rootParameters[3];

        ranges[0].Init(
            D3D12_DESCRIPTOR_RANGE_TYPE_UAV,
            2, // num descriptors.
            0, // BaseShaderRegister: map to register(u0) in HLSL.
            0, // RegisterSpace: map to register(u0, space0) in HLSL.
            D3D12_DESCRIPTOR_RANGE_FLAG_DATA_VOLATILE // descriptor static,
                                                      // data pointed to is
                                                      // not.
        );

        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV,
                       1, // num descriptors.
                       0, // b0
                       0, // space0
                       D3D12_DESCRIPTOR_RANGE_FLAG_DATA_STATIC);

        rootParameters[0].InitAsDescriptorTable(_countof(ranges), ranges,
                                                D3D12_SHADER_VISIBILITY_ALL);

        rootParameters[1].InitAsConstants(
                                          5,//num constants.
                                          0,//register.
                                          1//space.
                                          );

        rootParameters[2].InitAsShaderResourceView(
                                                   0,//register
                                                   0 //space.
                                                   );
                
        CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
        rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters);

        // TODO: could make a func called "serialize root sig".
        ID3DBlob *signature;
        ID3DBlob *error;
        defer(COM_RELEASE(signature));
        defer(COM_RELEASE(error));

        if ((hr = D3DX12SerializeVersionedRootSignature(
                 &rootSignatureDesc, featureData.HighestVersion, &signature,
                 &error)) == S_OK) {
          (hr = gd->d3dDevice->CreateRootSignature(
               0, signature->GetBufferPointer(), signature->GetBufferSize(),
               IID_PPV_ARGS(&gd->rootSig)));
        }
        INIT_FAIL_CHECK();
    }
    // end root sig create.

    // create compute pipeline state for postprocess
    {
        // note that the blob is only NEEDED temporary since it gets compiled
        // into the pipeline obj.
        IDxcBlob *computeShader = nullptr;
        defer(COM_RELEASE(computeShader));

        const char *shaderFilePath = "res\\shader.hlsl";
        bool r = ae::DX::compileShader(shaderFilePath, L"copy_shader",
                                       L"cs_6_0", &computeShader);
        if (!r || !computeShader) {
          // compileshader will print already.
          return;
        }

        D3D12_COMPUTE_PIPELINE_STATE_DESC computePipelineDesc = {};
        computePipelineDesc.pRootSignature = gd->rootSig;
        computePipelineDesc.CS = CD3DX12_SHADER_BYTECODE(
            computeShader->GetBufferPointer(), computeShader->GetBufferSize());

        hr = (gd->d3dDevice->CreateComputePipelineState(
            &computePipelineDesc, IID_PPV_ARGS(&gd->computePipelineState)));
        INIT_FAIL_CHECK();
    }

    LPCWSTR c_raygenShaderName = L"ray_gen_shader";
    LPCWSTR c_missShaderName = L"miss_main";
    LPCWSTR c_planeIntersectShaderName = L"intersection_plane";
    LPCWSTR c_sphereIntersectShaderName = L"intersection_sphere";
    LPCWSTR c_closestHitShaderName = L"closesthit_main";
    
    LPCWSTR c_planeHitgroupName = L"hitgroup_plane";
    LPCWSTR c_sphereHitgroupName = L"hitgroup_sphere";

    // NOTE: you know, I'm sort of convinced that what we have with this DXR API
    // is just really not so good. there just seems like so much crap that I
    // need to do for something quite simple....
    //
    // in fact, the API is so bad, they need a helper API to use the actual API.
    //
    // create the raytracing pipeline.
    {

        auto device = getRayDevice(gd->d3dDevice);

        CD3DX12_STATE_OBJECT_DESC raytracingPipeline{
            D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE};

        IDxcBlob *rayLib = nullptr;
        defer(COM_RELEASE(rayLib));

        // compile the raytracing shaders.
        // TODO: this is dumb because we are compiling the same file twice,
        // plus reading it from disk twice.
        const char *shaderFilePath = "res\\shader.hlsl";
        bool r = ae::DX::compileShader(shaderFilePath,
                                       L"", // TODO:empty entry works??,
                                       L"lib_6_3", &rayLib);
        if (!r || !rayLib) {
          // compileshader will print already.
          return;
        }

        auto lib = raytracingPipeline
                       .CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(
            rayLib->GetBufferPointer(), rayLib->GetBufferSize());

        lib->SetDXILLibrary(&libdxil);
        // NOTE: we do not want to export all shader signatures since
        // copy_shader is not part of our raytracing pipeline.
        lib->DefineExport(c_raygenShaderName);
        lib->DefineExport(c_missShaderName);
        lib->DefineExport(c_planeIntersectShaderName);
        lib->DefineExport(c_sphereIntersectShaderName);
        lib->DefineExport(c_closestHitShaderName);
       
        
        // create hit group plane
        auto hitGroup =
            raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
        hitGroup->SetIntersectionShaderImport(c_planeIntersectShaderName);
        hitGroup->SetHitGroupExport(c_planeHitgroupName);
        hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
        hitGroup->SetClosestHitShaderImport(c_closestHitShaderName);

        // create hit group sphere
        auto hitGroup2 =
            raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
        hitGroup2->SetIntersectionShaderImport(c_sphereIntersectShaderName);
        hitGroup2->SetHitGroupExport(c_sphereHitgroupName);
        hitGroup2->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
        hitGroup2->SetClosestHitShaderImport(c_closestHitShaderName);
        
        // Define the maximum sizes in bytes for the ray payload and attribute
        // structure.
        auto shaderConfig =
            raytracingPipeline
                .CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
        UINT payloadSize = 8 * sizeof(float);   // TODO: this is copy-pasta.
        UINT attributeSize = 8 * sizeof(float); // TODO: this is copy-pasta.
        shaderConfig->Config(payloadSize, attributeSize);

        auto globalRootSignature =
            raytracingPipeline
                .CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
        globalRootSignature->SetRootSignature(
            gd->rootSig); // TODO: we can just reuse the same root sig
        // here, right?

        auto pipelineConfig = raytracingPipeline.CreateSubobject<
            CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
        UINT maxRecursionDepth =
            8; // no more than 7 bounces (+1 to include the primary ray).
        pipelineConfig->Config(maxRecursionDepth);

        hr = device->CreateStateObject(raytracingPipeline,
                                       IID_PPV_ARGS(&gd->rayStateObject));
        INIT_FAIL_CHECK();
    }

    // upload buffers list.
    upload_buffer_helper uBuffer = {};

    // build the shader table which holds the shader records.
    // (for raytracing)
    {
        // some sizes.
        constexpr UINT shaderIdentifierSize =
            D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
        constexpr UINT recordAlign =
            D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT;
        constexpr UINT shaderTableAlign =
            D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;
        static_assert(shaderIdentifierSize <= shaderTableAlign,
                      "we assume shaderIdentifierSize <= shaderTableAlign.");

#define MAX_RECORDS 10
        
        // shader records to put in there.
        struct {
          struct {
            void *ident;
            LPCWSTR name;
          } records[MAX_RECORDS];
          size_t recordCount;
        } shaderTables[] = {
          {
            {{nullptr, c_raygenShaderName}},1},
          {
            {{nullptr, c_missShaderName}},1},
          {
            {{nullptr, c_planeHitgroupName},
             {nullptr, c_sphereHitgroupName}},2}
        };


        
        uint8_t *data = nullptr;
        uBuffer.curr().size = shaderTableAlign * _countof(shaderTables);
        uBuffer.curr().src =
            ae::DX::AllocUploadBuffer(gd->d3dDevice, uBuffer.curr().size,
                                      (void **)&data); // begins mapped.

        // create the backing resource for the shader table.
        (hr = gd->d3dDevice->CreateCommittedResource(
             &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
             D3D12_HEAP_FLAG_NONE,
             &CD3DX12_RESOURCE_DESC::Buffer(uBuffer.curr().size),

             // state needed for things read from a shader that is not a pixel
             // shader.
             D3D12_RESOURCE_STATE_COPY_DEST,

             nullptr, // optimized clear.
             IID_PPV_ARGS(&gd->rayShaderTable)));
        INIT_FAIL_CHECK();

        //TODO: maybe we use diff resources for the shader tables?
        // there is currently some wasted memory due to table start addr requirements.
        uBuffer.curr().dst = gd->rayShaderTable;
        uBuffer.curr().initState =
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;

        ID3D12StateObjectProperties *o;
        defer(COM_RELEASE(o));
        gd->rayStateObject->QueryInterface(&o);

        if (data) {
          auto totalSize = uBuffer.curr().size;
          memset(data, 0, totalSize );
          for (int j=0; j<_countof(shaderTables);j++)
            {
              auto &shaderRecords=shaderTables[j].records;
              uint8_t *pRecord=data;
              auto rc = shaderTables[j].recordCount;
              assert(rc<=MAX_RECORDS);
              for ( int i=0;i < rc;i++ )
                {
                  auto v= shaderRecords[i].ident = o->GetShaderIdentifier(shaderRecords[i].name);
                  assert(v);

                  assert(size_t(pRecord+shaderIdentifierSize-data)<=totalSize);
                  memcpy(pRecord, v, shaderIdentifierSize);
                  pRecord +=  recordAlign;
                  
                }

              data += shaderTableAlign;
            }


        }else {
          AELoggerError("oopsy");
        
        }

#undef MAX_RECORDS

        uBuffer.curr().src->Unmap(0,      // subres
                                  nullptr // entire subresource was modified.
        );

        uBuffer.iter();
    }

    game_window_info_t winInfo = automata_engine::platform::getWindowInfo();
    const size_t sceneBufferSize = ae::math::align_up(sizeof(dxr_world), 256ull);

    auto fnCreateCmdList = [&](ID3D12CommandAllocator **ppAllocator,
                               ID3D12GraphicsCommandList **ppCmdList,
                               ID3D12Fence **ppFence, HANDLE *pEvent) {
      // command allocator.
      (hr = gd->d3dDevice->CreateCommandAllocator(
           D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(ppAllocator)));
      INIT_FAIL_CHECK();

      // command list.
      (hr = gd->d3dDevice->CreateCommandList(
           0, // nodemask, this is for single GPU scenarios.
           D3D12_COMMAND_LIST_TYPE_DIRECT,
           *ppAllocator, // how the device allocates commands for this
                         // list.
           nullptr,      // initial pipeline state.
           IID_PPV_ARGS(ppCmdList)));

      (hr = gd->d3dDevice->CreateFence(0, // init value,
                                       D3D12_FENCE_FLAG_NONE,
                                       IID_PPV_ARGS(ppFence)));
      INIT_FAIL_CHECK();

      // Create an event. We can signal event to have all threads blocked by
      // WaitForSingleEvent now unblocked.
      *pEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
      if (*pEvent == nullptr) {
        (hr = HRESULT_FROM_WIN32(GetLastError()));
        INIT_FAIL_CHECK();
      }
    };

    // create queue.
    {
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        (hr = gd->d3dDevice->CreateCommandQueue(
             &queueDesc, IID_PPV_ARGS(&gd->commandQueue)));
        INIT_FAIL_CHECK();
    }

    // create all the cmd list stuffs.
    fnCreateCmdList(&gd->commandAllocator, &gd->commandList, &gd->fence,
                    &gd->fenceEvent);
    gd->fenceValue = 0;
    for (int i = 0; i < THREAD_COUNT; i++) {
        fnCreateCmdList(&gd->rayCmdAllocators[i], &gd->rayCmdLists[i],
                        &gd->rayFences[i], &gd->rayFenceEvents[i]);
        gd->rayFenceValues[i] = 0;
        // TODO: it's not the end of the world that we don't release these.
        //  the commented out thing below is what we could do. defer until
        //  close.
        //         ae::deferToClose(  COM_RELEASE() );
    }

    // build the acceleration structure.
    {
        auto device = getRayDevice(gd->d3dDevice);

        // scene infos.
        constexpr auto AABBcount = 2u;
        constexpr auto geoCount = 2u;
        constexpr auto tlasInstanceCount = 4u;

        // we begin by making all the BLASes.
        {
          size_t AABBsStride = sizeof(D3D12_RAYTRACING_AABB);
          size_t AABBsSize = AABBsStride  * AABBcount;

          // upload aabbs.
          {
            void *data;
                gd->AABBs =
                    ae::DX::AllocUploadBuffer(gd->d3dDevice, AABBsSize, &data);

                if (data) {
                    memset(data, 0, AABBsSize);

                    // make plane AABB.
                    D3D12_RAYTRACING_AABB AABBs[AABBcount] = {};

                    auto &planeAABB = AABBs[0];
                    planeAABB.MinX = -1000;
                    planeAABB.MinY = -0.01;
                    planeAABB.MinZ = -1000;
                    planeAABB.MaxX = 1000;
                    planeAABB.MaxY = 0.01;
                    planeAABB.MaxZ = 1000;

                    // sphere AABB.
                    //NOTE: default sphere has a radius of 1.
                    auto &sphereAABB = AABBs[1];
                    sphereAABB.MinX = sphereAABB.MinY = sphereAABB.MinZ = -1;
                    sphereAABB.MaxX = sphereAABB.MaxY = sphereAABB.MaxZ = 1;
                    
                    memcpy(data, AABBs, AABBsSize );
                } else {
                    AELoggerLog("oops");
                }

                gd->AABBs->Unmap(0,
                                 nullptr // entire subres modified.
                );
          }

          // setup geometries from aabbs.
          D3D12_RAYTRACING_GEOMETRY_DESC geometryDescs[geoCount] = {};
          {
            auto &planeDesc = geometryDescs[0];
            planeDesc.Type =
              D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
            planeDesc.AABBs.AABBCount = 1;
            planeDesc.AABBs.AABBs = {gd->AABBs->GetGPUVirtualAddress(),
                                        AABBsStride};
            auto &sphereDesc = geometryDescs[1];
            sphereDesc.Type =
              D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
            sphereDesc.AABBs.AABBCount = 1;
            sphereDesc.AABBs.AABBs = {gd->AABBs->GetGPUVirtualAddress()+
                                      AABBsStride,
                                        AABBsStride};
          }

          auto fnCreateBlasFromSingleGeometryDesc = [&](int idx) {

            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS bottomLevelInputs =
            {};

            // get prebuild info.
            D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO
            bottomLevelPrebuildInfo = {};
            {
                bottomLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
                bottomLevelInputs.Type =
                    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
                bottomLevelInputs.NumDescs = 1;
                bottomLevelInputs.pGeometryDescs =
                    &geometryDescs[idx];

                device->GetRaytracingAccelerationStructurePrebuildInfo(
                    &bottomLevelInputs, &bottomLevelPrebuildInfo);
            }
          
            // create blas.
            (hr = gd->d3dDevice->CreateCommittedResource(
               &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
               D3D12_HEAP_FLAG_NONE,
               &CD3DX12_RESOURCE_DESC::Buffer(
                   bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes,
                   D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
               D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
               nullptr, // optimized clear.
               IID_PPV_ARGS(&gd->BLASes[idx])));

            //TODO: since we are in a lambda, init fail check is only going to fail
            // to out of this func, but will not return any further.
            INIT_FAIL_CHECK();

            //TODO: we could create one scratch buffer that is large enough to hold
            // for all BLAS and TLAS.
            //
            // create scratch.
            UINT64 scratchSize = bottomLevelPrebuildInfo.ScratchDataSizeInBytes;
            (hr = gd->d3dDevice->CreateCommittedResource(
               &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
               D3D12_HEAP_FLAG_NONE,
               &CD3DX12_RESOURCE_DESC::Buffer(
                   scratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
               D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
               nullptr, // optimized clear.
               IID_PPV_ARGS(&gd->blasScratches[idx])));
            INIT_FAIL_CHECK();

            auto cmd = getRayCmdList(gd->commandList);

            // Bottom Level Acceleration Structure desc
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC
            bottomLevelBuildDesc = {};
            {
              bottomLevelBuildDesc.Inputs = bottomLevelInputs;
              bottomLevelBuildDesc.ScratchAccelerationStructureData =
                gd->blasScratches[idx]->GetGPUVirtualAddress();
              bottomLevelBuildDesc.DestAccelerationStructureData =
                gd->BLASes[idx]->GetGPUVirtualAddress();
            }

            // record the building.
            cmd->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0,
                                                      nullptr);
            cmd->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(gd->BLASes[idx]));
          };

          for (int i = 0; i < geoCount;i++)
              fnCreateBlasFromSingleGeometryDesc(i);

        }
        // END blas construction.
        // NOTE: we initially had some issues getting this to work. what were
        // the fixes??? this went as:
        // - seeing that not init geo descriptions (didn't give the number of
        // them).
        // - and that those had to exist (the array is not null) when init the
        // prebuild info. this indicates the data had to be valid on the GPU, afaik.

        
        // create the TLAS.
        {
          // get prebuild info.
          D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO
            topLevelPrebuildInfo = {};
          D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS topLevelInputs =
            {};
          {
                topLevelInputs.Type =
                    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
                topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
                topLevelInputs.NumDescs = tlasInstanceCount;

                device->GetRaytracingAccelerationStructurePrebuildInfo(
                    &topLevelInputs, &topLevelPrebuildInfo);
          }

          // alloc tlas.
          (hr = gd->d3dDevice->CreateCommittedResource(
               &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
               D3D12_HEAP_FLAG_NONE,
               &CD3DX12_RESOURCE_DESC::Buffer(
                   topLevelPrebuildInfo.ResultDataMaxSizeInBytes,
                   D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
               D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
               nullptr, // optimized clear.
               IID_PPV_ARGS(&gd->tlas)));
          INIT_FAIL_CHECK();

          // instantiate blas instances.
          {
                size_t bSize = tlasInstanceCount* sizeof(D3D12_RAYTRACING_INSTANCE_DESC);
                D3D12_RAYTRACING_INSTANCE_DESC instanceDescs[tlasInstanceCount] = {};

                int baseInstance=0;
                int instanceDescOffset=0;
                int totalInstanceCount=0;

                assert(world.planeCount==1);
                
                for (int i=0; i <world.planeCount;i++)
                  {
                    auto &inst = instanceDescs[baseInstance+i];

                    // TODO: our application does not currently support arbitrary planes.
                    // we only support the inf plane with normal pointing straight up.
                    inst.Transform[0][0] = inst.Transform[1][1] =
                      inst.Transform[2][2] = 1;

                    inst.InstanceID=i;

                    // TODO: maybe we care about different blas having different instance masks,
                    // but for now, 0xff for all blas is what we do.
                    inst.InstanceMask =
                      0xFF; // bitwise OR with what given on TraceRay side, and if zero, ignore.

                    inst.AccelerationStructure =
                      gd->BLASes[baseInstance]->GetGPUVirtualAddress();
                    inst.InstanceContributionToHitGroupIndex = baseInstance;

                    totalInstanceCount++;
                  }

                baseInstance++;
                instanceDescOffset+= world.planeCount;
                
                for (int i=0; i < world.sphereCount;i++)
                  {
                    auto &inst = instanceDescs[instanceDescOffset+i];

                    //TODO: our application does not support arbitrary radii spheres,
                    // although, that would be relatively trivial to support.
                    inst.Transform[0][0] = inst.Transform[1][1] =
                      inst.Transform[2][2] = 1;
                    //setup translation.
                    // NOTE: the transform is setup as 3 float4's contiguous in memory.
                    // where if you were to write this on a sheet of paper, you would
                    // be looking at a matrix of 4 columns and 3 rows.
                    inst.Transform[0][3]=world.spheres[i].p.x;
                    inst.Transform[1][3]=world.spheres[i].p.y;
                    inst.Transform[2][3]=world.spheres[i].p.z;
                    //memcpy(&inst.Transform[3], &world.spheres[i].p, sizeof(float)*3);

                    inst.InstanceID=i;

                    // TODO: maybe we care about different blas having different instance masks,
                    // but for now, 0xff for all blas is what we do.
                    inst.InstanceMask =
                      0xFF; // bitwise OR with what given on TraceRay side, and if zero, ignore.

                    inst.AccelerationStructure =
                      gd->BLASes[baseInstance]->GetGPUVirtualAddress();
                    inst.InstanceContributionToHitGroupIndex = baseInstance;

                    totalInstanceCount++;
                  }

                assert(totalInstanceCount == tlasInstanceCount);
                
                void *data;
                gd->tlasInstances =
                    ae::DX::AllocUploadBuffer(gd->d3dDevice, bSize, &data);

                if (data) {
                    memcpy(data, &instanceDescs, bSize);
                } else {
                    AELoggerLog("oops");
                }

                gd->tlasInstances->Unmap(0,
                                         nullptr // entire subres modified.
                );
          }

          // create the required scratch buffer.
          UINT64 scratchSize = topLevelPrebuildInfo.ScratchDataSizeInBytes;

          // TODO: creating a buffer with default args like this might be
          // something common that we want to pull out into AE.
          (hr = gd->d3dDevice->CreateCommittedResource(
               &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
               D3D12_HEAP_FLAG_NONE,
               &CD3DX12_RESOURCE_DESC::Buffer(
                   scratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
               D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
               nullptr, // optimized clear.
               IID_PPV_ARGS(&gd->tlasScratch)));
          INIT_FAIL_CHECK();

          auto cmd = getRayCmdList(gd->commandList);

          // Top Level Acceleration Structure desc
          D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc =
            {};
          {
            topLevelInputs.InstanceDescs =
              gd->tlasInstances->GetGPUVirtualAddress();
            topLevelBuildDesc.Inputs = topLevelInputs;
            topLevelBuildDesc.DestAccelerationStructureData =
              gd->tlas->GetGPUVirtualAddress();
            topLevelBuildDesc.ScratchAccelerationStructureData =
              gd->tlasScratch->GetGPUVirtualAddress();
          }
          
          cmd->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0,
                                                    nullptr);
          
        }
        // END tlas construction.
        //
        // NOTE: there were some issues getting this to work in the first place.
        // fixes?
        // - we saw that we were reuse the same scratch buffer as was used
        // to init the BLAS. maybe it is the case
        // that newly alloc scratch buffers are zero, and the hidden
        // acceleration structure impl relies on that.

    }
    //
    // END the ray tracing acceleration structure stuff.
    //
    
    // create the resources + descriptors.
    {
        // create the primary texture that sits in GPU memory for fast write.
        (hr = gd->d3dDevice->CreateCommittedResource(
             &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
             D3D12_HEAP_FLAG_NONE,
             &CD3DX12_RESOURCE_DESC::Tex2D(
                 DXGI_FORMAT_R8G8B8A8_UNORM, winInfo.width, winInfo.height, 1,
                 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
             D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
             nullptr, // optimized clear.
             IID_PPV_ARGS(&gd->gpuTex)));
        INIT_FAIL_CHECK();

        // create the texture for CPU side memcpy to backbuffer, for blit to
        // window.
        D3D12_HEAP_PROPERTIES hPROP = {};
        hPROP.Type = D3D12_HEAP_TYPE_CUSTOM;
        hPROP.MemoryPoolPreference = D3D12_MEMORY_POOL_L0; // system RAM.

        // write back here is a cache protocol in which when the CPU writes to
        // the page (the virt mem mapped to the heap), this is written to cache
        // instead of whatever backs the cache (the heap). write back is a fair
        // protocol since the CPU does not require write access.
        hPROP.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_WRITE_BACK;

        (hr = gd->d3dDevice->CreateCommittedResource(
             &hPROP, D3D12_HEAP_FLAG_CREATE_NOT_ZEROED,
             &CD3DX12_RESOURCE_DESC::Tex2D(
                 DXGI_FORMAT_R8G8B8A8_UNORM, winInfo.width, winInfo.height, 1,
                 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
             D3D12_RESOURCE_STATE_COMMON, /// initial access.
             nullptr,                     // optimized clear.
             IID_PPV_ARGS(&gd->cpuTex)));
        INIT_FAIL_CHECK();

        hr = gd->cpuTex->Map(0, NULL, nullptr);
        INIT_FAIL_CHECK();

        // create the buffer for storing information about the scene.
        {
          (hr = gd->d3dDevice->CreateCommittedResource(
               &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
               D3D12_HEAP_FLAG_CREATE_NOT_ZEROED,
               &CD3DX12_RESOURCE_DESC::Buffer(sceneBufferSize),
               D3D12_RESOURCE_STATE_COPY_DEST, /// initial access.
               nullptr,                        // optimized clear.
               IID_PPV_ARGS(&gd->sceneBuffer)));
          INIT_FAIL_CHECK();

          // write to the upload buffer.
          {
                void *data = nullptr;
                uBuffer.curr().size = sceneBufferSize;
                uBuffer.curr().src = ae::DX::AllocUploadBuffer(
                    gd->d3dDevice, uBuffer.curr().size,
                    (void **)&data); // begins mapped.
                uBuffer.curr().dst = gd->sceneBuffer;
                uBuffer.curr().initState =
                    D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;

                if (data) {
                    dxr_world w = {};

                    // TODO: right now we only support planes pointing up.
                    // and because that, I am being lazy here and only init one plane info
                    // in DXR world.
                    w.planes[0].d = 0;
                    // plane is pointing straight up!
                    w.planes[0].n[1] = 1; // this gives something that is NOT 0x1, since it is a float.
                    w.planes[0].matIndex = 1;

                    for (int i =0;i<world.sphereCount;i++)
                      {
                        w.spheres[i].r = world.spheres[i].r;
                        w.spheres[i].matIndex = world.spheres[i].matIndex;
                      }

                    for (int i =0;i<world.materialCount;i++)
                      {
                        auto &mat = world.materials[i];
                        w.materials[i].scatter = mat.scatter;
                        memcpy(&w.materials[i].refColor, &mat.refColor,sizeof(float)*3);
                        memcpy(&w.materials[i].emitColor, &mat.emitColor,sizeof(float)*3);
                      }

                    // generate the cone for shooting rays from a pixel.
                    for (int i =0;i<g_raysPerPixel;i++)
                      {
                        w.rands[i].xy[0] = RandomBilateral();
                        w.rands[i].xy[1] = RandomBilateral();
                      }
                    
                    memset(data, 0, sceneBufferSize);
                    memcpy(data, &w, sizeof(w));

                } else {
                    AELoggerError("oopsy");
                }

                uBuffer.curr().src->Unmap(
                    0,      // subres
                    nullptr // entire subresource was modified.
                );

                uBuffer.iter();
          }
        }

        // create the CBV_SRV_UAV descriptor heap + descriptors.
        {
          D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
          srvHeapDesc.NumDescriptors =
              4; // TODO: can we get some static verification that this is same
                 // as above (where we define root sig?)
          srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
          srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
          (hr = gd->d3dDevice->CreateDescriptorHeap(
               &srvHeapDesc, IID_PPV_ARGS(&gd->descHeap)));
          INIT_FAIL_CHECK();

          D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
          uavDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
          uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;

          CD3DX12_CPU_DESCRIPTOR_HANDLE heapHandle(
              gd->descHeap->GetCPUDescriptorHandleForHeapStart());

          gd->d3dDevice->CreateUnorderedAccessView(
              gd->gpuTex,
              nullptr, // no counter.
              &uavDesc,
              heapHandle // where to write the descriptor.
          );

          int off = gd->d3dDevice->GetDescriptorHandleIncrementSize(
              D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
          heapHandle.Offset(1, off);

          gd->d3dDevice->CreateUnorderedAccessView(
              gd->cpuTex,
              nullptr, // no counter.
              &uavDesc,
              heapHandle // where to write the descriptor.
          );

          heapHandle.Offset(1, off);

          D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
          cbvDesc.BufferLocation = gd->sceneBuffer->GetGPUVirtualAddress();
          cbvDesc.SizeInBytes = sceneBufferSize;

          gd->d3dDevice->CreateConstantBufferView(
              &cbvDesc,
              heapHandle // where to write the descriptor.
          );

          heapHandle.Offset(1, off);

          //TODO: there is no longer a need to be writing this descriptor.
          //      alternatively, we could go back to this approach instead of setting it
          //      in the root parameter. that approach was valid but we did not see the fruits
          //      since lots of other things were incorrect.
          //
          // create the scene accel structure descriptor.
          D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
          srvDesc.Shader4ComponentMapping =
              D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
          srvDesc.ViewDimension =
              D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE;
          srvDesc.RaytracingAccelerationStructure.Location =
              gd->tlas->GetGPUVirtualAddress();

          gd->d3dDevice->CreateShaderResourceView(
              // gd->tlas,
              nullptr, &srvDesc, heapHandle);
        }
    }

    // SCHEDULE any remaining UPLOADS.
    uBuffer.flush(gd);

    // execute upload buffers + raytrace accel build.
    {
        auto &cmd = gd->commandList;
        cmd->Close();

        ID3D12CommandList *ppCommandLists[] = {cmd};
        gd->commandQueue->ExecuteCommandLists(1, ppCommandLists);

        // wait for that work to complete:
        {
          hr = (gd->commandQueue->Signal(gd->fence, ++gd->fenceValue));
          hr =
              (gd->fence->SetEventOnCompletion(gd->fenceValue, gd->fenceEvent));
          WaitForSingleObjectEx(gd->fenceEvent, INFINITE, FALSE);
        }

        // reset the cmd allocator and cmd lists for setup then as the one for
        // get currScreen to backbuffer.
        gd->commandAllocator->Reset();
        cmd->Reset(gd->commandAllocator, nullptr);
    }

    // allow for destroy of the upload buffers now that they are unused.
    for (int i = 0; i < uBuffer.capacity; i++) {
        auto &ub = uBuffer[i];
        if (ub.src)
          ub.src->Release();
        ub.src = nullptr;
    }

    // record the compute work.
    {
        auto cmd = gd->commandList;

        // NOTE(handmade_gpu): this sort of API doesn't make sense to me.
        // if the pipeline already contains the signature, why must I bind both
        // here?
        cmd->SetPipelineState(
            gd->computePipelineState); // contains the monolithic shader.
        cmd->SetComputeRootSignature(gd->rootSig);

        // bind heap and point the compute root sig to that heap.
        ID3D12DescriptorHeap *ppHeaps[] = {gd->descHeap};
        cmd->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);
        cmd->SetComputeRootDescriptorTable(
            0, gd->descHeap->GetGPUDescriptorHandleForHeapStart());

        cmd->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
                                    gd->cpuTex, D3D12_RESOURCE_STATE_COMMON,
                                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS));

        cmd->Dispatch(ae::math::div_ceil(winInfo.width, 16),
                      ae::math::div_ceil(winInfo.height, 16),
                      1); // using the pipeline.

        cmd->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
                                    gd->cpuTex,
                                    D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                    D3D12_RESOURCE_STATE_COMMON));

        hr = (cmd->Close());
        INIT_FAIL_CHECK();
    }



#undef INIT_FAIL_CHECK

    image = AllocateImage(winInfo.width, winInfo.height);


    // define camera and characteristics
    cameraP = V3(0, -10, 1); // go back 10 and up 1
    cameraZ = Normalize(cameraP);
    cameraX = Normalize(Cross(V3(0,0,1), cameraZ));
    cameraY = Normalize(Cross(cameraZ, cameraX));
    if (image.width > image.height) {
        filmH = filmW * (float)image.height / (float)image.width;
    } else if (image.height > image.width) {
        filmW = filmH * (float)image.width / (float)image.height;
    }
    halfFilmW = filmW / 2.0f;
    halfFilmH = filmH / 2.0f;
    filmCenter = cameraP - filmDist * cameraZ;
    halfPixW = 1.0f / image.width;
    halfPixH = 1.0f / image.height;
    const char *appName = "raytracer_vis";
    ae::bifrost::registerApp(appName, visualizer);
    ae::bifrost::updateApp(gameMemory, appName);
    CreateThread(nullptr,
                 0, // default stack size.
                 master_thread,
                 (void*)gameMemory,
                 0, // thread runs immediately after creation.
                 nullptr);
}

void automata_engine::Close(ae::game_memory_t *gameMemory) {
    auto gd = getGameData(gameMemory);

    // TODO: is there a proper order to release these objects?

    COM_RELEASE(gd->commandQueue);
    COM_RELEASE(gd->commandList);
    COM_RELEASE(gd->commandAllocator);

    COM_RELEASE(gd->computePipelineState);
    COM_RELEASE(gd->rootSig);

    COM_RELEASE(gd->descHeap);
    COM_RELEASE(gd->gpuTex);
    COM_RELEASE(gd->cpuTex);
    COM_RELEASE(gd->sceneBuffer);

    COM_RELEASE(gd->fence);

    COM_RELEASE(gd->debugController);
    COM_RELEASE(gd->d3dDevice);
}

// TODO: we really do not like the below.
void ae::OnVoiceBufferEnd(game_memory_t *gameMemory, intptr_t voiceHandle) {}
void ae::OnVoiceBufferProcess(game_memory_t *gameMemory, intptr_t voiceHandle,
                              float *dst, float *src, uint32_t samplesToWrite,
                              int channels, int bytesPerSample) {}
void ae::InitAsync(game_memory_t *gameMemory) {
    gameMemory->setInitialized(true);
}
