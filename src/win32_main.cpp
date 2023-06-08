#include <stdio.h>
#include <stdlib.h>

#include <automata_engine.hpp>

#include <synchapi.h>

#include <comdef.h>
#include <windows.h>

// number of render threads to spawn.
#define THREAD_COUNT 7

#include "ray.h"  // WARNING, THIS .H USES THE THREAD_COUNT MARCO.
#include "ray_dxr.h"

#define COM_RELEASE(comPtr) (comPtr != nullptr) ? comPtr->Release() : 0;

// different application modes.
enum app_mode { APP_MODE_DXR, APP_MODE_VK, APP_MODE_CPU };

#if AUTOMATA_ENGINE_DX12_BACKEND
static app_mode g_appMode = APP_MODE_DXR;
#elif AUTOMATA_ENGINE_VK_BACKEND

static app_mode g_appMode = APP_MODE_VK;
void            WaitForAndResetFence(VkDevice device, VkFence *pFence, uint64_t waitTime = 1000 * 1000 * 1000);
void            emitSingleImageBarrier(VkCommandBuffer cmd,
               VkAccessFlags                           src,
               VkAccessFlags                           dst,
               VkImageLayout                           srcLayout,
               VkImageLayout                           dstLayout,
               VkImage                                 img,
               VkPipelineStageFlags                    before,
               VkPipelineStageFlags                    after);

#endif

// a nice helper + forward decls.
game_data        *getGameData(ae::game_memory_t *gameMemory) { return (game_data *)gameMemory->data; }
void              InitD3D12(game_data *gd);
void              InitVK(ae::game_memory_t *gameMemory);
void              visualizer(ae::game_memory_t *gameMemory);
static image_32_t AllocateImage(unsigned int width, unsigned int height);
DWORD WINAPI      master_thread(_In_ LPVOID lpParameter);
dxr_world         SCENE_TO_DXR_WORLD();

/*
THE MISSION ::

 - render the amazing Sponza scene.
 - make it realtime.
 - do the raytracing with the Vulkan RT API.
 - do the first person camera navigation.
 - do the correct maths; where those are coming from
https://graphicscodex.com/projects/rays/index.html.
   - the light transport equation. radiance, irradiance. BSDF / PBR. soft/hard
shadows. scattering/attenuation/absorption/refraction. lens/pinhole(DOF). monte
carlo.
 - while you are at it, contrib to Automata-Engine so that we have some nice VK
helpers.
 - do make the things look good, we want:
   - the shadows (which means that we have some light sources).
   - the GI.
   - conversion from HDR space to linear.
   - if we can, bloom would be awesome.
   - the PBR material model, or something like that.
   - add some fog and god rays (volumetric lighting).
*/

/*

getting the Sponza stuff working. (so, I'm more interested in doing this one)

1. load the model(s) from .obj.
2. load the textures and stuff.
3. fix the problem of not being able to generate random things on the GPU.
4. modify the shaders to do better material things.
5. modify the acceleration structures quite a bit.

the above tasks are written in no particular order, but if we want to write these things in 
the desired order of completion, that would be: 

1. fix the problem of not being able to generate random things on the GPU.
2. then we start fucking around with the shader side of things and get better BRDFs + proper maths.
3. then we load the models and modify the accel structure.
4. finally we slap the textures on the meshes for maximum awesomeness.

 */


/*

incrementally supporting the Vulkan stuff.

 - so as we did with the DX stuff, this looks like the first step is getting the
VK device setup, then making sure that we have our async copy thing going nicely
from the GPU tex to the CPU one. and rendering that to the screen so that we can
see this bad boy whilst it is going :D

^ this!

*/

// ==================== GLOBALS =========================
static world_t g_world = {};
// populate word with floor and spheres.
static v3    g_cameraP  = {};
static v3    g_cameraZ  = {};
static v3    g_cameraX  = {};
static v3    g_cameraY  = {};
static float g_filmDist = 1.0f;
static float g_filmW    = 1.0f;
static float g_filmH    = 1.0f;
static float g_halfFilmW;
static float g_halfFilmH;
static v3    g_filmCenter;
float        g_halfPixW;
float        g_halfPixH;
unsigned int g_raysPerPixel = 256;
image_32_t   g_image        = {};
std::mutex   g_commandQueueMutex;

// ================== AE CALLBACKS ======================
void ae::HandleWindowResize(game_memory_t *gameMemory, int nw, int nh) {}

void ae::PreInit(game_memory_t *gameMemory)
{
    ae::defaultWinProfile = AUTOMATA_ENGINE_WINPROFILE_NORESIZE;
    ae::defaultWindowName = "Raytracer";
    ae::defaultWidth      = 1280;
    ae::defaultHeight     = 720;
}

void automata_engine::Close(ae::game_memory_t *gameMemory)
{
    auto gd = getGameData(gameMemory);

    // TODO: is there a proper order to release these objects?

#if AUTOMATA_ENGINE_DX12_BACKEND
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
    
#endif //TODO: a good idea would be to make a "CloseDX12" kind of idea.
}

// TODO: we really do not like the below.
void ae::OnVoiceBufferEnd(game_memory_t *gameMemory, intptr_t voiceHandle) {}
void ae::OnVoiceBufferProcess(game_memory_t *gameMemory,
    intptr_t                                 voiceHandle,
    float                                   *dst,
    float                                   *src,
    uint32_t                                 samplesToWrite,
    int                                      channels,
    int                                      bytesPerSample)
{}
void ae::InitAsync(game_memory_t *gameMemory) { gameMemory->setInitialized(true); }

void ae::Init(game_memory_t *gameMemory)
{
    AELoggerLog("Doing stuff...\n");

    // early init of structures so that we can also copy into CBV,
    // among other initialization needs.
    static material_t materials[4] = {};  // NOTE: this must be static else taking that addr of it is quite
                                          // poor, due to the scop being inside a func.

    
    // NOTE: the way that our material system works is that the only way a pixel gets a color
    // is if the ray is able to bounce to an emitter. along the way and before it hits the emitter,
    // the colors are collected from the .refColor of the object.
    //
    // and in a way, we can think of this as the objects absorbing specific wavelenghts, leaving the
    // wavelength of the color we actually see. so it's a bit of a mask operation.
    //
    // the scatter param simply describes how rough the surface is. the higher the value, the more rough
    // the surface is. this results in very random bounces once a ray hits this surface.
    materials[0].emitColor = V3(1.0f, 1, 1);
    materials[1].refColor  = V3(0.5f, 0.5f, 0.5f);
    materials[1].scatter   = 0.;
    materials[2].refColor  = V3(0.7f, 0.25f, 0.3f);
    materials[2].scatter   = 0.8;
    materials[3].refColor  = V3(0.0f, 0.8f, 0.0f);
    materials[3].scatter   = 1.0f;

    static plane_t planes[1] = {};
    planes[0].n              = V3(0, 0, 1);
    planes[0].d              = 0;  // plane on origin
    planes[0].matIndex       = 1;

    static sphere_t spheres[3] = {};
    spheres[0].p               = V3(0, 0, 0);
    spheres[0].r               = 1.0f;
    spheres[0].matIndex        = 2;
    spheres[1].p               = V3(-3, 0, -2);
    spheres[1].r               = 1.0f;
    spheres[1].matIndex        = 2;
    spheres[2].p               = V3(-2, 2, 0);
    spheres[2].r               = 1.0f;
    spheres[2].matIndex        = 3;
    g_world.materialCount      = ARRAY_COUNT(materials);
    g_world.materials          = materials;
    g_world.planeCount         = ARRAY_COUNT(planes);
    g_world.planes             = planes;
    g_world.sphereCount        = ARRAY_COUNT(spheres);
    g_world.spheres            = spheres;

    assert(g_world.sphereCount <= DXR_WORLD_LIMIT);
    assert(g_world.planeCount <= DXR_WORLD_LIMIT);
    assert(g_world.materialCount <= DXR_WORLD_LIMIT);

    auto gd = getGameData(gameMemory);

#if AUTOMATA_ENGINE_DX12_BACKEND
    if (g_appMode == APP_MODE_DXR) InitD3D12(gd);
#elif AUTOMATA_ENGINE_VK_BACKEND
    if (g_appMode == APP_MODE_VK) InitVK(gameMemory);
#endif

    game_window_info_t winInfo = ae::platform::getWindowInfo();

    g_image = AllocateImage(winInfo.width, winInfo.height);

    // define camera and characteristics
    g_cameraP = V3(0, -10, 1);  // go back 10 and up 1
    g_cameraZ = Normalize(g_cameraP);
    g_cameraX = Normalize(Cross(V3(0, 0, 1), g_cameraZ));
    g_cameraY = Normalize(Cross(g_cameraZ, g_cameraX));
    if (g_image.width > g_image.height) {
        g_filmH = g_filmW * (float)g_image.height / (float)g_image.width;
    } else if (g_image.height > g_image.width) {
        g_filmW = g_filmH * (float)g_image.width / (float)g_image.height;
    }
    g_halfFilmW         = g_filmW / 2.0f;
    g_halfFilmH         = g_filmH / 2.0f;
    g_filmCenter        = g_cameraP - g_filmDist * g_cameraZ;
    g_halfPixW          = 1.0f / g_image.width;
    g_halfPixH          = 1.0f / g_image.height;
    const char *appName = "raytracer_vis";
    ae::bifrost::registerApp(appName, visualizer);
    ae::bifrost::updateApp(gameMemory, appName);
    CreateThread(nullptr,
        0,  // default stack size.
        master_thread,
        (void *)gameMemory,
        0,  // thread runs immediately after creation.
        nullptr);
}
//
//
//  ==================== END AE CALLBACKS =======================

static unsigned int GetTotalPixelSize(image_32_t image) { return image.height * image.width * sizeof(unsigned int); }

static image_32_t AllocateImage(unsigned int width, unsigned int height)
{
    image_32_t newImage         = {};
    newImage.width              = width;
    newImage.height             = height;
    unsigned int totalPixelSize = GetTotalPixelSize(newImage);
    newImage.pixelPointer       = (unsigned int *)malloc(totalPixelSize);
    return newImage;
}

static void WriteImage(image_32_t image, char *fileName)
{
    unsigned int    outputPixelSize = GetTotalPixelSize(image);
    bitmap_header_t header          = {};
    header.FileType                 = 0x4D42;
    header.FileSize                 = sizeof(bitmap_header_t) + outputPixelSize;
    header.BitmapOffset             = sizeof(header);
    header.size                     = 40;
    header.Width                    = image.width;
    header.Height                   = image.height;
    header.Planes                   = 1;
    header.BitsPerPixel             = 32;
    FILE *fileHandle                = fopen("test.bmp", "wb");
    if (fileHandle) {
        fwrite(&header, sizeof(header), 1, fileHandle);
        fwrite(image.pixelPointer, outputPixelSize, 1, fileHandle);
        fclose(fileHandle);
    } else {
        fprintf(stderr, "[ERROR] Unable to write output file\n");
    }
}

static v3 RayCast(world_t *pWorld, v3 rayOrigin, v3 rayDirection)
{
    v3    result         = {};
    float tolerance      = 0.0001f;
    float minHitDistance = 0.001f;
    v3    attenuation    = V3(1, 1, 1);
    for (unsigned int bounceCount = 0; bounceCount < 8; ++bounceCount) {
        float        hitDistance = FLT_MAX;
        unsigned int hitMatIndex = 0;
        v3           nextNormal  = {};
        // sphere intersection test.
        for (unsigned int sphereIndex = 0; sphereIndex < pWorld->sphereCount; sphereIndex++) {
            sphere_t sphere                  = pWorld->spheres[sphereIndex];
            v3       sphereRelativeRayOrigin = rayOrigin - sphere.p;
            float    a                       = Dot(rayDirection, rayDirection);
            float    b                       = 2.0f * Dot(sphereRelativeRayOrigin, rayDirection);
            float    c        = Dot(sphereRelativeRayOrigin, sphereRelativeRayOrigin) - sphere.r * sphere.r;
            float    denom    = 2.0f * a;
            float    rootTerm = SquareRoot(b * b - 4.0f * a * c);
            if (rootTerm > tolerance) {
                // NOTE(Noah): The denominator can never be zero
                float tp = (-b + rootTerm) / denom;
                float tn = (-b - rootTerm) / denom;
                float t  = tp;
                if ((tn > minHitDistance) && (tn < tp)) { t = tn; }
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = sphere.matIndex;
                    nextNormal  = Normalize(t * rayDirection + sphereRelativeRayOrigin);
                }
            }
        }
        // floor intersection test
        for (unsigned int planeIndex = 0; planeIndex < pWorld->planeCount; planeIndex++) {
            plane_t plane = pWorld->planes[planeIndex];
            float   denom = Dot(plane.n, rayDirection);
            if ((denom < -tolerance) || (denom > tolerance)) {
                float t = (-plane.d - Dot(plane.n, rayOrigin)) / denom;
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = plane.matIndex;
                    nextNormal  = plane.n;
                }
            }
        }
        if (hitMatIndex) {
            material_t mat = pWorld->materials[hitMatIndex];
            // TODO(Noah): Do real reflectance stuff
            result      = result + Hadamard(attenuation, mat.emitColor);
            attenuation = Hadamard(attenuation, mat.refColor);
            rayOrigin   = rayOrigin + hitDistance * rayDirection;
            // NOTE(Noah): this does a reflection thing
            // TODO(Noah): these are not accurate permutations
            v3 pureBounce   = rayDirection - 2.0f * Dot(nextNormal, rayDirection) * nextNormal;
            v3 randomBounce = Normalize(nextNormal + V3(RandomBilateral(), RandomBilateral(), RandomBilateral()));
            rayDirection    = Normalize(Lerp(randomBounce, pureBounce, mat.scatter));
        } else {
            // sky contrib and terminate all future bounces;
            material_t mat = pWorld->materials[hitMatIndex];
            result         = result + Hadamard(attenuation, mat.emitColor);
            break;
        }
    }
    return result;
}

void blitToGameBackbuffer(ae::game_memory_t *gameMemory)
{
#define UPDATE_FAIL_CHECK()                                                                                            \
    if (hr != S_OK) return;

    HRESULT hr;

    auto gd = getGameData(gameMemory);

#if AUTOMATA_ENGINE_DX12_BACKEND
    {
        std::lock_guard<std::mutex> lock(g_commandQueueMutex);

        // Execute the command list.
        ID3D12CommandList *ppCommandLists[] = {gd->commandList};
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
    box.right     = gameMemory->backbufferWidth;
    box.bottom    = gameMemory->backbufferHeight;
    box.back      = 1;  // depth of one.

    UINT rowPitch = sizeof(uint32_t) * gameMemory->backbufferWidth;
    hr            = gd->cpuTex->ReadFromSubresource(gameMemory->backbufferPixels,
        rowPitch,
        rowPitch * gameMemory->backbufferHeight,
        0,  // src subresource.
        &box);
#endif

// TODO: put the vk_check idea in the block below.
#if AUTOMATA_ENGINE_VK_BACKEND

    // NOTE: only one thread can submit to the queue at a time.
    {
        std::lock_guard<std::mutex> lock(g_commandQueueMutex);
        VkSubmitInfo                si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount          = 1;
        si.pCommandBuffers             = &gd->cmdBuf;
        (vkQueueSubmit(gd->vkQueue, 1, &si, gd->vkFence));
    }

    // wait for the work.
    WaitForAndResetFence(gd->vkDevice, &gd->vkFence);

    size_t size = gameMemory->backbufferWidth * gameMemory->backbufferHeight * sizeof(int);

    auto   thingToMap = gd->vkCpuTexBufferBacking;
    size_t mapOffset  = 0;
    size_t mapSize    = VK_WHOLE_SIZE;

    int   flags = 0;
    void *data  = nullptr;
    // TODO: we should keep this memory persistently mapped.
    VkResult result = vkMapMemory(gd->vkDevice, thingToMap, mapOffset, mapSize, flags, &data);
    if (result == VK_SUCCESS && data) {
        // make the GPU writes to mapped region visible so that CPU can do the reading
        VkMappedMemoryRange range = {VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE};
        range.memory              = thingToMap;
        range.offset              = mapOffset;
        range.size                = mapSize;
        result                    = vkInvalidateMappedMemoryRanges(gd->vkDevice, 1, &range);
        assert(result == VK_SUCCESS);

        assert(gd->vkCpuTexBufferSize >= size);
        memcpy((void *)gameMemory->backbufferPixels, data, size);

        vkUnmapMemory(gd->vkDevice, thingToMap);
    }

    // make sure we get the data into the backbuffer memory so that it can be blit to screen.

#endif

#undef UPDATE_FAIL_CHECK
}

void visualizer(ae::game_memory_t *gameMemory)
{
    switch (g_appMode) {

        case APP_MODE_DXR:
        case APP_MODE_VK:
        {
            blitToGameBackbuffer(gameMemory);
        } break;
        case APP_MODE_CPU: {
            // TODO(Noah): Right now, the image is upside-down. Do we fix this on the
            // application side or is this something that we can fix on the engine side?
            memcpy((void *)gameMemory->backbufferPixels,
                g_image.pixelPointer,
                sizeof(uint32_t) * gameMemory->backbufferWidth * gameMemory->backbufferHeight);
        } break;
    }
}

typedef struct texel {
    int      width;
    int      height;
    uint32_t xPos;
    uint32_t yPos;
} texel_t;

struct render_thread_data {
    std::tuple<texel_t *, uint32_t> *pTexelTuple;
    unsigned int                     threadIdx;
    game_data                       *gd;
};

#if AUTOMATA_ENGINE_DX12_BACKEND
// TODO: for therse two functions below, do we need to schedule a release???
// pretty sure we do, and query interface increases the reference count of the
// thing.
ID3D12GraphicsCommandList4 *getRayCmdList(ID3D12GraphicsCommandList *cmd)
{
    ID3D12GraphicsCommandList4 *r = nullptr;
    cmd->QueryInterface(&r);
    assert(r);
    return r;
}
ID3D12Device5 *getRayDevice(ID3D12Device *device)
{
    ID3D12Device5 *r = nullptr;
    device->QueryInterface(&r);
    assert(r);
    return r;
}
#endif

// NOTE(Noah):
// Here's some documentation on some of the first multithreading bugs I have
// ever encountered!
//
// It seems like the threads are overlapping (drawing to same texels).
// And the stagger pattern at the beginning (looked like staircase going down
// from left -> right) is the overlapping threads having their ptr_to_texels
// array updated. stagger pattern ends with texels being draw in what appear to
// be those belonging to the very last thread group.
//
// In fact, this is exactly what is happening. we pass a pointer, which is going
// to be the same one every time and the threads are all in the same virtual
// memory space. So next time thru the loop, the texels arr is updated for all
// threads.
DWORD WINAPI render_thread(_In_ LPVOID lpParameter)
{
    auto threadData = (struct render_thread_data *)lpParameter;
    auto gd         = threadData->gd;

    std::tuple<texel_t *, uint32_t> *ptr_to_tuple = threadData->pTexelTuple;

    texel_t *texels = std::get<0>(*ptr_to_tuple);

    for (uint32_t i = 0; i < std::get<1>(*ptr_to_tuple); i++) {
        texel_t texel = texels[i];

        switch (g_appMode)

        {
#if AUTOMATA_ENGINE_VK_BACKEND
            case APP_MODE_VK: {
                auto  tIdx = threadData->threadIdx;
                auto &cmd  = gd->rayCmdBufs[tIdx];

                VkCommandBufferBeginInfo beginInfo = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
                (vkBeginCommandBuffer(cmd, &beginInfo));

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, gd->vkRayPipeline);
                vkCmdBindDescriptorSets(cmd,
                    VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
                    gd->pipelineLayout,
                    0,  // set # of first desc set to bind.
                    1,  // desc sets.
                    &gd->theDescSet,
                    // the two fields below are for dynamic offsets or whatever.
                    0,
                    nullptr);

                struct {
                    unsigned int xPos;
                    unsigned int yPos;
                    unsigned int seed;
                    unsigned int width;
                    unsigned int height;
                } constants = {
                    texel.xPos, texel.yPos, (unsigned int)(ae::timing::epoch()), g_image.width, g_image.height};

                vkCmdPushConstants(cmd,
                    gd->pipelineLayout,
                    VK_SHADER_STAGE_ALL,  //TODO: maybe we can get more granular about this.
                    0,                    //offset.
                    sizeof(constants),
                    &constants);

                auto shaderTableAddr = ae::VK::getBufferVirtAddr(gd->vkDevice, gd->vkShaderTable);

                size_t tableElemSize  = gd->vkShaderGroupHandleSize;
                size_t tableAlignment = gd->vkShaderGroupTableAlignment;

                auto stride = tableElemSize;
                auto size   = tableElemSize;

                VkStridedDeviceAddressRegionKHR raygenTable = {shaderTableAddr, stride, size};

                VkStridedDeviceAddressRegionKHR missTable = {shaderTableAddr + tableAlignment, stride, size};

                VkStridedDeviceAddressRegionKHR hitShaderTable = {shaderTableAddr + tableAlignment * 2, stride, size};

                VkStridedDeviceAddressRegionKHR callableShaderTable = {
                    // NOTE: this is valid for us to do with the VK API,
                    // but from an API standpoint I would have preffered that we
                    // are able to pass nullptr as the table.
                };

                vkCmdTraceRaysKHR(
                    cmd, &raygenTable, &missTable, &hitShaderTable, &callableShaderTable, texel.width, texel.height, 1);

                (vkEndCommandBuffer(cmd));

                auto &fence = gd->rayFences[tIdx];
                // NOTE: only one thread can submit to the queue at a time.
                {
                    std::lock_guard<std::mutex> lock(g_commandQueueMutex);
                    VkSubmitInfo                si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
                    si.commandBufferCount          = 1;
                    si.pCommandBuffers             = &cmd;
                    (vkQueueSubmit(gd->vkQueue, 1, &si, fence));
                }

                // wait for the work.
                WaitForAndResetFence(gd->vkDevice, &fence);

                // TODO: add vk_check here.
                (vkResetCommandBuffer(cmd, 0));
                (vkResetCommandPool(gd->vkDevice, gd->rayCommandPools[tIdx], 0));

            } break;

#endif

#if AUTOMATA_ENGINE_DX12_BACKEND
            case APP_MODE_DXR: {
                HRESULT hr;
                // record command list.
                auto tIdx = threadData->threadIdx;
                auto cmd  = getRayCmdList(gd->rayCmdLists[tIdx]);
                defer(COM_RELEASE(cmd));

                // dispatch rays for texel.
                cmd->SetPipelineState1(gd->rayStateObject);

                D3D12_DISPATCH_RAYS_DESC rayDesc = {};

                //   D3D12_GPU_VIRTUAL_ADDRESS_RANGE
                rayDesc.RayGenerationShaderRecord = {
                    gd->rayShaderTable->GetGPUVirtualAddress(),
                    D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT  // size
                };

                // D3D12_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE
                rayDesc.MissShaderTable = {
                    gd->rayShaderTable->GetGPUVirtualAddress() + D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
                    D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT,  // size
                    D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT   // stride
                };

                // D3D12_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE
                rayDesc.HitGroupTable = {
                    gd->rayShaderTable->GetGPUVirtualAddress() + 2 * D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT,
                    // TODO: this constant of 2 here is hacky.
                    D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT * 2,  // size
                    D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT       // stride
                };

                rayDesc.Width  = texel.width;
                rayDesc.Height = texel.height;
                rayDesc.Depth  = 1;

                // bind the other crap so the shaders actually know where to get the
                // resources, etc. bind tlas too.
                {
                    cmd->SetComputeRootSignature(gd->rootSig);
                    cmd->SetDescriptorHeaps(1, &gd->descHeap);
                    cmd->SetComputeRootDescriptorTable(0, gd->descHeap->GetGPUDescriptorHandleForHeapStart());
                    struct {
                        unsigned int xPos;
                        unsigned int yPos;
                        unsigned int seed;
                        unsigned int width;
                        unsigned int height;
                    } constants = {
                        texel.xPos, texel.yPos, (unsigned int)(ae::timing::epoch()), g_image.width, g_image.height};
                    cmd->SetComputeRoot32BitConstants(1,  // root param.
                        5,                                // num 32 bit vals to set.
                        &constants,
                        0  // offset.
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
                    ID3D12CommandList          *ppCommandLists[] = {gd->rayCmdLists[tIdx]};
                    gd->commandQueue->ExecuteCommandLists(1, ppCommandLists);
                    // Schedule a Signal command in the queue.
                    hr = (gd->commandQueue->Signal(gd->rayFences[tIdx], ++gd->rayFenceValues[tIdx]));
                    if (hr != S_OK) throw;
                }

                // use a render thread local fence to wait on that work.
                {
                    hr = (gd->fence->SetEventOnCompletion(gd->rayFenceValues[tIdx], gd->rayFenceEvents[tIdx]));
                    if (hr != S_OK) throw;
                    WaitForSingleObjectEx(gd->rayFenceEvents[tIdx], INFINITE, FALSE);

                    // TODO: this solves some "cmd list allocator reset" ideas.
                    // but also, if I just let those happen, the app still goes to
                    // completion and I get the raytraced image.
                    //
                    // so, there seems to be some funny business going on. we should
                    // definitely investigate that!
                    //
                    // Sleep(500);
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
                    nullptr  // initial pipeline state.
                    ));

            } break;
#endif // #if AUTOMATA_ENGINE_DX12_BACKEND

            case APP_MODE_CPU: {
                unsigned int *out = g_image.pixelPointer + texel.yPos * g_image.width + texel.xPos;
                // Raytracer works by averaging all colors from all rays shot from this
                // pixel.
                for (unsigned int y = texel.yPos; y < (texel.height + texel.yPos); y++) {
                    float filmY = -1.0f + 2.0f * (float)y / (float)g_image.height;
                    for (unsigned int x = texel.xPos; x < (texel.width + texel.xPos); x++) {
                        float filmX   = -1.0f + 2.0f * (float)x / (float)g_image.width;
                        v3    color   = {};
                        float contrib = 1.0f / (float)g_raysPerPixel;
                        for (unsigned int rayIndex = 0; rayIndex < g_raysPerPixel; rayIndex++) {
                            float offX = filmX + (RandomBilateral() * g_halfPixW);
                            float offY = filmY + (RandomBilateral() * g_halfPixH);
                            v3    filmP =
                                g_filmCenter + (offX * g_halfFilmW * g_cameraX) + (offY * g_halfFilmH * g_cameraY);
                            v3 rayOrigin    = g_cameraP;
                            v3 rayDirection = Normalize(filmP - g_cameraP);
                            color           = color + contrib * RayCast(&g_world, rayOrigin, rayDirection);
                        }
                        v4           BMPColor = {255.0f * LinearToSRGB(color.r),
                                      255.0f * LinearToSRGB(color.g),
                                      255.0f * LinearToSRGB(color.b),
                                      255.0f};
                        unsigned int BMPValue = BGRAPack4x8(BMPColor);
                        *out++                = BMPValue;  // ARGB
                    }
                    out += g_image.width - texel.width;
                }

            } break;
        }

        // TODO(Noah): It seems that the thread continues even after we close
        // the window?? (we had prints that were showing) it could be the
        // terminal doing a buffering thing, and just being slow. OR, it could
        // be that the threads are for reals still alive?? If it is the second
        // one, this is a cause for concern.
    }
    ExitThread(0);
}

// TODO: once a thread completes work it should steal texels from other threads
// still working.
//
// on a per-thread basis, we could maybe introduce cpu SIMD.

DWORD WINAPI master_thread(_In_ LPVOID lpParameter)
{
    auto gameMemory = (ae::game_memory_t *)lpParameter;

#define THREAD_GROUP_SIZE 32
#define PIXELS_PER_TEXEL  (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)
    uint32_t maxTexelsPerThread =
        (uint32_t)ceilf((float)(g_image.width * g_image.height) / (float)(PIXELS_PER_TEXEL * THREAD_COUNT));
    AELoggerLog("maxTexelsPerThread: %d", maxTexelsPerThread);
    {
        HANDLE   threadHandles[THREAD_COUNT];
        uint32_t xPos = 0;
        uint32_t yPos = 0;
        // TODO(Noah): Could do entire image as BSP tree -> assign threads to
        // these regions. then break up these regions into texels.
        texel_t *texels = nullptr;

        std::tuple<texel_t *, uint32_t> texelParams[THREAD_COUNT];
        render_thread_data              renderThreadData[THREAD_COUNT];

        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            for (uint32_t j = 0; j < maxTexelsPerThread; j++) {
                texel_t texel;
                texel.width  = THREAD_GROUP_SIZE;
                texel.height = THREAD_GROUP_SIZE;
                texel.xPos   = xPos;
                texel.yPos   = yPos;
                xPos += THREAD_GROUP_SIZE;
                bool isPartialTexel = false;
                if (texel.yPos + texel.height > g_image.height) {
                    texel.height -= (texel.yPos + texel.height) - g_image.height;
                    texel.height   = ae::math::max(texel.height, 0);
                    isPartialTexel = true;
                }
                if (xPos >= g_image.width) {
                    if (xPos > g_image.width) {
                        texel.width -= xPos - g_image.width;
                        texel.width    = ae::math::max(texel.width, 0);
                        isPartialTexel = true;
                    }
                    xPos = 0;
                    yPos += THREAD_GROUP_SIZE;
                }
                if (texel.xPos >= 0 && (texel.xPos + texel.width <= g_image.width) && texel.yPos >= 0 &&
                    (texel.yPos + texel.height <= g_image.height)) {
                    if (isPartialTexel) j--;  // NOTE(Noah): This is hack ...
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
                (i + 1 < THREAD_COUNT) ? maxTexelsPerThread
                                       : StretchyBufferCount(texels) - (THREAD_COUNT - 1) * maxTexelsPerThread);

            auto &rtd       = renderThreadData[i];
            rtd.pTexelTuple = &texelParams[i];
            rtd.threadIdx   = i;
            rtd.gd          = getGameData(gameMemory);

            threadHandles[i] = CreateThread(nullptr,
                0,  // default stack size.
                render_thread,
                (LPVOID)&rtd,
                0,  // thread runs immediately after creation.
                nullptr);
        }
        // wait for all threads to complete.
        for (uint32_t i = 0; i < THREAD_COUNT; i++) { WaitForSingleObject(threadHandles[i], INFINITE); }
        StretchyBufferFree(texels);
    }

    image_32_t img = {};
    switch (g_appMode) {
        case APP_MODE_DXR: {
            // NOTE: this is a little hacky but I don't think its TOO bad. it could of
            // course
            //  be better. but anyways, we're going to sleep this thread for just a
            //  little bit. in doing so, we allow the render thread to "catch up" and
            //  fetch to the game backbuffer the very last image. the final image is
            //  const, so doesn't matter if the render thread fetches many time. just
            //  need it to have fetched the final at least once.
            Sleep(500  // ms
            );
            img.width        = gameMemory->backbufferWidth;
            img.height       = gameMemory->backbufferHeight;
            img.pixelPointer = gameMemory->backbufferPixels;
        } break;
        case APP_MODE_CPU:

        {
            img = g_image;
        } break;
    }

    WriteImage(img, "test.bmp");
    AELoggerLog("Done. Image written to test.bmp\n");
    ExitThread(0);
}

#if AUTOMATA_ENGINE_DX12_BACKEND
struct upload_buffer {
    ID3D12Resource *src;
    ID3D12Resource *dst;
    UINT64          size;
    unsigned int    initState;
};

struct upload_buffer_helper {
    static constexpr unsigned int capacity = 10;
    upload_buffer                 uBuffers[capacity];
    unsigned int                  ubCount = 0;

    upload_buffer &operator[](size_t index)
    {
        assert(index < capacity);
        return uBuffers[index];
    }

    void flush(game_data *gd)
    {
        auto &cmd = gd->commandList;

        for (int i = 0; i < ubCount; i++) {
            auto &ub = uBuffers[i];
            if (ub.src && ub.dst) {
                // NOTE: we assume state before of COPY_DEST.
                cmd->CopyBufferRegion(ub.dst, 0, ub.src, 0, ub.size);
                cmd->ResourceBarrier(1,
                    &CD3DX12_RESOURCE_BARRIER::Transition(
                        ub.dst, D3D12_RESOURCE_STATE_COPY_DEST, (D3D12_RESOURCE_STATES)ub.initState));
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
#endif  // #if AUTOMATA_ENGINE_DX12_BACKEND

#if AUTOMATA_ENGINE_VK_BACKEND

void InitVK(ae::game_memory_t *gameMemory)
{
    auto gd = getGameData(gameMemory);

#define VK_CHECK(x)                                                                                                    \
    do {                                                                                                               \
        VkResult err = x;                                                                                              \
        if (err) {                                                                                                     \
            AELoggerError("Detected Vulkan error: %s", ae::VK::VkResultToString(err));                                 \
            abort();                                                                                                   \
        }                                                                                                              \
    } while (0)

    // Volk is a Vulkan loader (gets the function pointers).
    if (volkInitialize()) {
        AELoggerError("Failed to initialize volk.");
        return;
    }

    auto validateExtensions = [](const char **required, const VkExtensionProperties *available) -> bool {
        for (uint32_t i = 0; i < StretchyBufferCount(required); i++) {
            auto extension = required[i];
            bool found     = false;
            for (uint32_t j = 0; j < StretchyBufferCount(available); j++) {
                auto available_extension = available[j];
                if (strcmp(available_extension.extensionName, extension) == 0) {
                    found = true;
                    break;
                }
            }
            if (!found) { return false; }
        }
        return true;
    };

    // query the extensions we want.
    const char **active_instance_extensions = nullptr;
    {
        uint32_t               instance_extension_count;
        VkExtensionProperties *instance_extensions = nullptr;
        defer(StretchyBufferFree(instance_extensions));

        VK_CHECK(vkEnumerateInstanceExtensionProperties(nullptr, &instance_extension_count, nullptr));
        StretchyBufferInitWithCount(instance_extensions, instance_extension_count);
        VK_CHECK(vkEnumerateInstanceExtensionProperties(nullptr, &instance_extension_count, instance_extensions));
        assert(instance_extension_count == StretchyBufferCount(instance_extensions));

        // specify desired instance extensions and check to make sure available.

        StretchyBufferPush(active_instance_extensions, VK_KHR_SURFACE_EXTENSION_NAME);
        // NOTE: Below is deprecated by debug_utils instance extension.
        // StretchyBufferPush(active_instance_extensions, VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
        StretchyBufferPush(active_instance_extensions, VK_KHR_WIN32_SURFACE_EXTENSION_NAME);
        StretchyBufferPush(active_instance_extensions, /*VK_EXT_debug_utils*/ VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        if (!validateExtensions(active_instance_extensions, instance_extensions)) {
            AELoggerError("Required instance extensions are missing.");
            AELoggerLog("instance exts found count = %d", StretchyBufferCount(instance_extensions));
            return;
        }
    }

    // query the instance layers we want.
    const char **requested_validation_layers = nullptr;
    {
        auto validateLayers = [](const char              **required,
                                  uint32_t                 requiredCount,
                                  const VkLayerProperties *available,
                                  uint32_t                 availableCount) -> bool {
            for (uint32_t i = 0; i < requiredCount; i++) {
                auto *extension = required[i];
                bool  found     = false;
                for (uint32_t j = 0; j < availableCount; j++) {
                    auto available_extension = available[j];
                    if (strcmp(available_extension.layerName, extension) == 0) {
                        found = true;
                        break;
                    }
                }
                if (!found) { return false; }
            }
            return true;
        };

        uint32_t           instance_layer_count;
        VkLayerProperties *supported_validation_layers = nullptr;
        defer(StretchyBufferFree(supported_validation_layers));
        VK_CHECK(vkEnumerateInstanceLayerProperties(&instance_layer_count, nullptr));
        StretchyBufferInitWithCount(supported_validation_layers, instance_layer_count);
        VK_CHECK(vkEnumerateInstanceLayerProperties(&instance_layer_count, supported_validation_layers));

        {
            std::vector<std::vector<const char *>> validation_layer_priority_list = {
                // The preferred validation layer is "VK_LAYER_KHRONOS_validation"
                {"VK_LAYER_KHRONOS_validation"},
                // Otherwise we fallback to using the LunarG meta layer
                {"VK_LAYER_LUNARG_standard_validation"},
                // Otherwise we attempt to enable the individual layers that compose the LunarG meta layer since it doesn't exist
                {
                    "VK_LAYER_GOOGLE_threading",
                    "VK_LAYER_LUNARG_parameter_validation",
                    "VK_LAYER_LUNARG_object_tracker",
                    "VK_LAYER_LUNARG_core_validation",
                    "VK_LAYER_GOOGLE_unique_objects",
                },
                // Otherwise as a last resort we fallback to attempting to enable the LunarG core layer
                {"VK_LAYER_LUNARG_core_validation"}};

            for (auto validation_layers : validation_layer_priority_list) {
                if (validateLayers(validation_layers.data(),
                        validation_layers.size(),
                        supported_validation_layers,
                        StretchyBufferCount(supported_validation_layers))) {
                    for (auto layer : validation_layers) { StretchyBufferPush(requested_validation_layers, layer); }
                    break;
                }

                AELoggerWarn("Couldn't find {%s, ...} - falling back", validation_layers[0]);
            }
        }

        if (requested_validation_layers == nullptr) { AELoggerWarn("No validation layers enabled"); }
    }

    VkApplicationInfo app = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName  = ae::defaultWindowName;
    app.pEngineName       = "Automata Engine";         // TODO: is there a place from AE that we can get this?
    app.apiVersion        = VK_MAKE_VERSION(1, 2, 0);  // highest version the app will use.

    VkInstanceCreateInfo instance_info = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instance_info.pApplicationInfo     = &app;

    instance_info.enabledExtensionCount   = StretchyBufferCount(active_instance_extensions);
    instance_info.ppEnabledExtensionNames = active_instance_extensions;

    // layers are inserted into VK API call chains when they are enabled.
    instance_info.enabledLayerCount   = StretchyBufferCount(requested_validation_layers);
    instance_info.ppEnabledLayerNames = requested_validation_layers;

    // the VK instance is the "VK runtime" sort of idea. by this point, we aren't talking with GPU.
    VK_CHECK(vkCreateInstance(&instance_info, nullptr, &gd->vkInstance));

    // TODO: maybe building the instance is something that can be pretty easy for AE to "just do" for the app.
    // ae::VK::initVK(desiredExtensions, desiredLayers, appInfo);

    // Load VK instance functions.
    volkLoadInstance(gd->vkInstance);

    // TODO: maybe this is a candidate to put in AE as well.
    {
        VkDebugUtilsMessengerCreateInfoEXT debugMsgerCI = {VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
        debugMsgerCI.messageSeverity                    = VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;

        {
            debugMsgerCI.messageSeverity |=
                VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT;
        }

        debugMsgerCI.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                   VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                   VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        debugMsgerCI.pfnUserCallback = ae::VK::ValidationDebugCallback;

        vkCreateDebugUtilsMessengerEXT(gd->vkInstance, &debugMsgerCI, nullptr, &gd->vkDebugMsg);
    }

    // now we init device and queue.
    {
        // find the GPU.
        uint32_t          gpu_count = 0;
        VkPhysicalDevice *gpus      = nullptr;
        defer(StretchyBufferFree(gpus));
        VK_CHECK(vkEnumeratePhysicalDevices(gd->vkInstance, &gpu_count, nullptr));
        if (gpu_count != 1) {
            if (gpu_count < 1)
                AELoggerError("No physical device found.");
            else
                AELoggerError("Too many GPUs! Automata Engine only supports single adapter systems.");
            return;
        }
        StretchyBufferInitWithCount(gpus, gpu_count);
        VK_CHECK(vkEnumeratePhysicalDevices(gd->vkInstance, &gpu_count, gpus));
        gd->vkGpu = gpus[0];

        VkPhysicalDeviceProperties deviceProperties;
        (vkGetPhysicalDeviceProperties(gd->vkGpu, &deviceProperties));

        // find the queues and pick one.
        uint32_t                 queue_family_count      = 0;
        VkQueueFamilyProperties *queue_family_properties = nullptr;
        defer(StretchyBufferFree(queue_family_properties));
        (vkGetPhysicalDeviceQueueFamilyProperties(gd->vkGpu, &queue_family_count, nullptr));
        if (queue_family_count < 1) {
            AELoggerError("No queue family found.");
            return;
        }
        StretchyBufferInitWithCount(queue_family_properties, queue_family_count);
        vkGetPhysicalDeviceQueueFamilyProperties(gd->vkGpu, &queue_family_count, queue_family_properties);
        for (uint32_t i = 0; i < queue_family_count; i++) {
            // Find a queue family which supports graphics and presentation.
            if ((queue_family_properties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
                gd->vkQueueIndex = i;
                break;
            }
        }
        if (gd->vkQueueIndex < 0) {
            AELoggerError("Did not find suitable queue which supports graphics, compute and presentation.");
        }

        uint32_t               device_extension_count;
        VkExtensionProperties *device_extensions = nullptr;
        defer(StretchyBufferFree(device_extensions));
        VK_CHECK(vkEnumerateDeviceExtensionProperties(gd->vkGpu, nullptr, &device_extension_count, nullptr));
        StretchyBufferInitWithCount(device_extensions, device_extension_count);
        VK_CHECK(vkEnumerateDeviceExtensionProperties(gd->vkGpu, nullptr, &device_extension_count, device_extensions));

        // Initialize required extensions.
        const char **required_device_extensions = nullptr;
        defer(StretchyBufferFree(required_device_extensions));
        //        StretchyBufferPush(required_device_extensions, VK_KHR_SWAPCHAIN_EXTENSION_NAME);
        StretchyBufferPush(required_device_extensions, VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
        StretchyBufferPush(required_device_extensions, VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
        StretchyBufferPush(required_device_extensions, VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME);
        StretchyBufferPush(required_device_extensions, VK_KHR_RAY_QUERY_EXTENSION_NAME);
        if (!validateExtensions(required_device_extensions, device_extensions)) {
            AELoggerError("missing required device extensions");
            return;  //throw std::exception();
        }

        // add some additional features from the device to request.
        //TODO: we need to actually check the card for support for all these features below.

        // TODO: currently I understand that the buffer device feature is so that we can access buffer data in shader where that device address is maybe an offset
        //       into some memory, without a VkBuffer. something like that??

        VkPhysicalDeviceVulkan12Features superCoolFeatures = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
        superCoolFeatures.bufferDeviceAddress              = VK_TRUE;

        VkPhysicalDeviceAccelerationStructureFeaturesKHR evenMoreFeatures = {
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
        evenMoreFeatures.pNext                 = &superCoolFeatures;
        evenMoreFeatures.accelerationStructure = VK_TRUE;

        VkPhysicalDeviceRayTracingPipelineFeaturesKHR moreFeatures = {
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR};  //::rayTracingPipeline
        moreFeatures.pNext              = &evenMoreFeatures;
        moreFeatures.rayTracingPipeline = VK_TRUE;

        VkPhysicalDeviceRayQueryFeaturesKHR features = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR};
        features.rayQuery                            = VK_TRUE;
        features.pNext                               = &moreFeatures;

        VkDeviceQueueCreateInfo queue_info     = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        const float             queue_priority = 1.0f;
        queue_info.queueFamilyIndex            = gd->vkQueueIndex;
        queue_info.queueCount                  = 1;
        queue_info.pQueuePriorities            = &queue_priority;

        VkDeviceCreateInfo device_info      = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        device_info.pNext                   = &features;  // requested features.
        device_info.queueCreateInfoCount    = 1;
        device_info.pQueueCreateInfos       = &queue_info;
        device_info.enabledExtensionCount   = StretchyBufferCount(required_device_extensions);
        device_info.ppEnabledExtensionNames = required_device_extensions;

        VK_CHECK(vkCreateDevice(gd->vkGpu, &device_info, nullptr, &gd->vkDevice));

        // presumably loads device functions.
        volkLoadDevice(gd->vkDevice);

        vkGetDeviceQueue(gd->vkDevice, gd->vkQueueIndex, 0, &gd->vkQueue);
    }

    // init the command buffer(s) for submit.
    {
        auto makeCmdBuf = [&](VkCommandPool *poolOut, VkCommandBuffer *bufOut) {
            VkCommandPoolCreateInfo cmd_pool_info = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
            //cmd_pool_info.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT; // command buffers alloc from pool are short-lived.
            cmd_pool_info.queueFamilyIndex = gd->vkQueueIndex;
            cmd_pool_info.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            VK_CHECK(vkCreateCommandPool(gd->vkDevice, &cmd_pool_info, nullptr, poolOut));

            VkCommandBufferAllocateInfo cmd_buf_info = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
            cmd_buf_info.commandPool                 = *poolOut;
            cmd_buf_info.level                       = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            cmd_buf_info.commandBufferCount          = 1;
            VK_CHECK(vkAllocateCommandBuffers(gd->vkDevice, &cmd_buf_info, bufOut));
        };

        makeCmdBuf(&gd->commandPool, &gd->cmdBuf);

        for (int it = 0; it < THREAD_COUNT; it++) {
            makeCmdBuf(&gd->rayCommandPools[it], &gd->rayCmdBufs[it]);
            VkFenceCreateInfo ci = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            VK_CHECK(vkCreateFence(gd->vkDevice, &ci, nullptr, &gd->rayFences[it]));
        }
    }

    // create the pipeline layout.
    {
        // we have two UAV textures to bind.
        // we have one CBV to bind, and another CBV that maps to push constants.
        // then we have the SRV acceleration structure to bind.

        VkDescriptorSetLayoutCreateInfo ci = {VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};

        VkDescriptorSetLayoutBinding bindings[4] = {};
        bindings[0]                              = {
            0,  //binding.
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            1,                    //count.
            VK_SHADER_STAGE_ALL,  //TODO: hopefully this includes the raytracing stuff.
            nullptr               //samplers.
        };
        bindings[1] = {
            1,  //binding.
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            1,                    //count.
            VK_SHADER_STAGE_ALL,  //TODO: hopefully this includes the raytracing stuff.
            nullptr               //samplers.
        };
        bindings[2] = {
            2,  //binding.
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            1,                    //count.
            VK_SHADER_STAGE_ALL,  //TODO: hopefully this includes the raytracing stuff.
            nullptr               //samplers.
        };
        bindings[3] = {
            3,
            VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
            1,
            VK_SHADER_STAGE_ALL,
            nullptr  //samplers.
        };

        ci.bindingCount = _countof(bindings);
        ci.pBindings    = bindings;

        VK_CHECK(vkCreateDescriptorSetLayout(gd->vkDevice,
            &ci,
            nullptr,  // allocator.
            &gd->descSet));

        VkPushConstantRange consts = {
            VK_SHADER_STAGE_ALL,
            0,               //offset.
            5 * sizeof(int)  //size
        };
        VkPipelineLayoutCreateInfo li = {VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        li.setLayoutCount             = 1;  //1 descriptor set layout.
        li.pSetLayouts                = &gd->descSet;
        li.pushConstantRangeCount     = 1;
        li.pPushConstantRanges        = &consts;

        VK_CHECK(vkCreatePipelineLayout(gd->vkDevice, &li, nullptr, &gd->pipelineLayout));
    }

    // create the compute pipeline.

    {
        const char                     *shaderEntry = "copy_shader";
        VkPipelineShaderStageCreateInfo stageInfo   = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        stageInfo.pName                             = shaderEntry;
        stageInfo.stage                             = VK_SHADER_STAGE_COMPUTE_BIT;
        stageInfo.module                            = ae::VK::loadShaderModule(gd->vkDevice,
            "res\\shader.hlsl",
            L"copy_shader",
            L"cs_6_0"  // TODO: is there a way to verify that this is the same as shaderEntry above?
        );

        VkComputePipelineCreateInfo ci = {VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        ci.stage                       = stageInfo;

        ci.layout = gd->pipelineLayout;
        VK_CHECK(vkCreateComputePipelines(gd->vkDevice,
            VK_NULL_HANDLE,  //pipeline cache.
            1,               //createInfoCount,
            &ci,
            nullptr,  // allocation callbacks.
            &gd->vkComputePipeline));
    }

    ae::game_window_info_t winInfo = automata_engine::platform::getWindowInfo();

    // find a heap that is a good and nice one :D
    {
        // TODO: this guy below maybe is an AE thing.
        VkPhysicalDeviceMemoryProperties deviceMemoryProperties;

        auto getDesiredMemoryTypeIndex = [&](VkMemoryPropertyFlags positiveProp,
                                             VkMemoryPropertyFlags negativeProp) -> uint32_t {
            vkGetPhysicalDeviceMemoryProperties(gd->vkGpu, &deviceMemoryProperties);
            for (uint32_t i = 0; i < deviceMemoryProperties.memoryTypeCount; i++) {
                auto flags = deviceMemoryProperties.memoryTypes[i].propertyFlags;
                if (((flags & positiveProp) == positiveProp) && ((flags & negativeProp) == 0)) { return i; }
            }
            AELoggerError("Could not find a heap with a suitable memory type!");
            assert(false);  //TODO.
        };

        gd->vkUploadHeapIdx =
            getDesiredMemoryTypeIndex(VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT | VK_MEMORY_PROPERTY_PROTECTED_BIT);

        gd->vkVramHeapIdx =
            getDesiredMemoryTypeIndex(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    }

    // create some resources.
    size_t worldSize = sizeof(dxr_world);
    {
        VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;

        gd->vkCpuTexSize = ae::VK::createImage2D_dumb(gd->vkDevice,
            winInfo.width,
            winInfo.height,
            gd->vkVramHeapIdx,
            format,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            &gd->vkCpuTex,
            &gd->vkCpuTexBacking);

        // NOTE: this be the one that we going to raytrace into :)
        gd->vkGpuTexSize = ae::VK::createImage2D_dumb(gd->vkDevice,
            winInfo.width,
            winInfo.height,
            gd->vkVramHeapIdx,
            format,
            VK_IMAGE_USAGE_STORAGE_BIT,
            &gd->vkGpuTex,
            &gd->vkGpuTexBacking);

        auto makeDumbView = [&](VkImage img, VkImageView *viewOut) {
            // create the image view(s).
            VkImageViewCreateInfo imageViewCI = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            imageViewCI.image                 = img;
            imageViewCI.viewType              = VK_IMAGE_VIEW_TYPE_2D;
            imageViewCI.format                = format;
            imageViewCI.subresourceRange      = {VK_IMAGE_ASPECT_COLOR_BIT,  //aspect mask
                     0,                                                      //base mip
                     VK_REMAINING_MIP_LEVELS,
                     0,  //base array,
                     VK_REMAINING_ARRAY_LAYERS};

            VK_CHECK(vkCreateImageView(gd->vkDevice, &imageViewCI, nullptr, viewOut));
        };

        makeDumbView(gd->vkCpuTex, &gd->vkCpuTexView);

        makeDumbView(gd->vkGpuTex, &gd->vkGpuTexView);

        // create the buffer to store the row major form of the texture.
        gd->vkCpuTexBufferSize = ae::VK::createBufferDumb(gd->vkDevice,
            gd->vkCpuTexSize,
            gd->vkUploadHeapIdx,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            &gd->vkCpuTexBuffer,
            &gd->vkCpuTexBufferBacking);

        // create the buffer for storing the scene constant data.
        void *data;

        size_t worldActualSize = ae::VK::createUploadBufferDumb(gd->vkDevice,
            worldSize,
            gd->vkUploadHeapIdx,
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            &gd->vkWorldBuffer,
            &gd->vkWorldBufferBacking,
            &data);
        assert(worldSize <= worldActualSize);
        auto mappedThing = gd->vkWorldBufferBacking;
        if (worldActualSize && data) {
            // write the world data teehee.
            dxr_world w = SCENE_TO_DXR_WORLD();
            memcpy(data, &w, worldSize);

            ae::VK::flushAndUnmapUploadBuffer(gd->vkDevice, worldSize, gd->vkWorldBufferBacking);
        }
    }

    // create the fence so that we can do the waiting stuff.
    {
        VkFenceCreateInfo ci = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        VK_CHECK(vkCreateFence(gd->vkDevice, &ci, nullptr, &gd->vkFence));
    }

    // setup the acceleration structures.
    {
        // TODO: we are going to require TWO blas.
        //
        // do that blas stuff.
        VkAccelerationStructureBuildSizesInfoKHR blasPrebuildInfo = {
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};

        constexpr uint32_t  aabbCount    = 1;
        size_t              aabbDataSize = sizeof(VkAabbPositionsKHR);  // TODO.
        VkAabbPositionsKHR *aabbData;
        ae::VK::createUploadBufferDumb(gd->vkDevice,
            aabbDataSize,
            gd->vkUploadHeapIdx,
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
            &gd->vkAabbBuffer,
            &gd->vkAabbBufferBacking,
            (void **)&aabbData);

        if (aabbData) {
            auto &aabb = aabbData[0];
            // write.
            aabb.minX = -1;
            aabb.minY = -1;
            aabb.minZ = -1;
            aabb.maxX = 1;
            aabb.maxY = 1;
            aabb.maxZ = 1;

            // flush and unmap.
            ae::VK::flushAndUnmapUploadBuffer(gd->vkDevice, aabbDataSize, gd->vkAabbBufferBacking);
        }

        VkAccelerationStructureGeometryKHR sphereGeo = {VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        sphereGeo.geometryType                       = VK_GEOMETRY_TYPE_AABBS_KHR;
        sphereGeo.geometry.aabbs.sType               = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_AABBS_DATA_KHR;
        sphereGeo.geometry.aabbs.stride              = sizeof(VkAabbPositionsKHR);
        sphereGeo.geometry.aabbs.data.deviceAddress  = ae::VK::getBufferVirtAddr(gd->vkDevice, gd->vkAabbBuffer);

        VkAccelerationStructureBuildGeometryInfoKHR blasBuildInfo = {
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
        blasBuildInfo.type          = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        blasBuildInfo.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        blasBuildInfo.mode          = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        blasBuildInfo.geometryCount = 1;
        blasBuildInfo.pGeometries   = &sphereGeo;

        vkGetAccelerationStructureBuildSizesKHR(gd->vkDevice,
            VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
            &blasBuildInfo,
            &aabbCount,  // aabb counts for geometries.
            &blasPrebuildInfo);

        // create the blas buffer and blas scratch.
        ae::VK::createBufferDumb(gd->vkDevice,
            blasPrebuildInfo.accelerationStructureSize,
            gd->vkVramHeapIdx,
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
            &gd->vkBlasBuffer,
            &gd->vkBlasBufferBacking);
        // scratch.
        ae::VK::createBufferDumb(gd->vkDevice,
            blasPrebuildInfo.buildScratchSize,
            gd->vkVramHeapIdx,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
            &gd->vkScratch[0],
            &gd->vkScratchBacking[0]);

        // create the blas.
        {
            VkAccelerationStructureCreateInfoKHR ci = {VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
            //      createFlags;
            ci.buffer = gd->vkBlasBuffer;  // where it will be stored, the AS is auto bound to this.
            ci.offset = 0;                 //VkDeviceSize                             offset;
            ci.size   = blasPrebuildInfo.accelerationStructureSize;  // size required for AS.
            ci.type =
                VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;  //      VkAccelerationStructureTypeKHR           type;

            // NOTE: the device address is for replay stuff.
            // VkDeviceAddress                          deviceAddress;

            VK_CHECK(vkCreateAccelerationStructureKHR(gd->vkDevice, &ci, nullptr, &gd->vkBlas));
        }

        // do that tlas stuff.

        VkAccelerationStructureBuildSizesInfoKHR tlasPrebuildInfo = {
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};

        // instance data.
        constexpr uint32_t instanceDataCount = 1;
        size_t             instanceDataSize  = sizeof(VkAccelerationStructureInstanceKHR) * instanceDataCount;
        VkAccelerationStructureInstanceKHR *instanceData;
        ae::VK::createUploadBufferDumb(gd->vkDevice,
            instanceDataSize,
            gd->vkUploadHeapIdx,
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
            &gd->vkInstanceBuffer,
            &gd->vkInstanceBufferBacking,
            (void **)&instanceData);

        if (instanceData) {
            // TODO: this should be a for loop for all the instances in the world.
            {
                int   idx  = 0;
                auto &inst = instanceData[0];  //[idx];

                inst.transform.matrix[0][0] = inst.transform.matrix[1][1] = inst.transform.matrix[2][2] = 1;

                inst.instanceCustomIndex = 0;  // TODO.

                inst.mask = 0xFF;  // bitwise OR with what given on TraceRay
                // side, and if zero, ignore.

                // TODO: maybe we need this more than once, who knows.
                VkAccelerationStructureDeviceAddressInfoKHR info = {
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR};
                info.accelerationStructure          = gd->vkBlas;
                inst.accelerationStructureReference = vkGetAccelerationStructureDeviceAddressKHR(gd->vkDevice, &info);

                // TODO:
                inst.instanceShaderBindingTableRecordOffset = 0;
            }

            ae::VK::flushAndUnmapUploadBuffer(gd->vkDevice, instanceDataSize, gd->vkInstanceBufferBacking);
        }

        // TODO: add the other spheres, and the rest of the world.
        VkAccelerationStructureGeometryKHR sphereInstance = {VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};
        sphereInstance.geometryType                       = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        sphereInstance.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
        sphereInstance.geometry.instances.arrayOfPointers = VK_FALSE;

        sphereInstance.geometry.instances.data.deviceAddress =
            ae::VK::getBufferVirtAddr(gd->vkDevice, gd->vkInstanceBuffer);

        VkAccelerationStructureBuildGeometryInfoKHR tlasBuildInfo = {
            VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR};
        tlasBuildInfo.type          = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        tlasBuildInfo.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        tlasBuildInfo.mode          = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        tlasBuildInfo.geometryCount = 1;
        tlasBuildInfo.pGeometries   = &sphereInstance;

        vkGetAccelerationStructureBuildSizesKHR(gd->vkDevice,
            VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
            &tlasBuildInfo,
            &instanceDataCount,
            &tlasPrebuildInfo);

        // create the tlas buffer.
        ae::VK::createBufferDumb(gd->vkDevice,
            tlasPrebuildInfo.accelerationStructureSize,
            gd->vkVramHeapIdx,
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
            &gd->vkTlasBuffer,
            &gd->vkTlasBufferBacking);
        // scratch.
        ae::VK::createBufferDumb(gd->vkDevice,
            tlasPrebuildInfo.buildScratchSize,
            gd->vkVramHeapIdx,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
            &gd->vkScratch[2],
            &gd->vkScratchBacking[2]);

        // Provided by VK_KHR_acceleration_structure
        VkAccelerationStructureCreateInfoKHR ci = {VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        //      createFlags;
        ci.buffer = gd->vkTlasBuffer;  // where it will be stored, the AS is auto bound to this.
        ci.offset = 0;                 //VkDeviceSize                             offset;
        ci.size   = tlasPrebuildInfo.accelerationStructureSize;  // size required for AS.
        ci.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;  //      VkAccelerationStructureTypeKHR           type;

        // NOTE: the device address is for replay stuff.
        // VkDeviceAddress                          deviceAddress;

        VK_CHECK(vkCreateAccelerationStructureKHR(gd->vkDevice, &ci, nullptr, &gd->vkTlas));

        // update the build infos now that things exist.
        tlasBuildInfo.scratchData.deviceAddress = ae::VK::getBufferVirtAddr(gd->vkDevice, gd->vkScratch[2]);
        tlasBuildInfo.dstAccelerationStructure  = gd->vkTlas;
        blasBuildInfo.scratchData.deviceAddress = ae::VK::getBufferVirtAddr(gd->vkDevice, gd->vkScratch[0]);
        blasBuildInfo.dstAccelerationStructure  = gd->vkBlas;

        // record the acceleration structure builds.
        auto &cmd = gd->cmdBuf;

        VkCommandBufferBeginInfo beginInfo = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

        // before doing the buildiong of the acceleration structures, transit the images to
        // general layout.
        // TODO: is this the right idea ???
        {
            emitSingleImageBarrier(cmd,
                VK_ACCESS_NONE,
                VK_ACCESS_NONE,
                VK_IMAGE_LAYOUT_UNDEFINED,  //discard is OK.
                VK_IMAGE_LAYOUT_GENERAL,
                gd->vkCpuTex,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

            emitSingleImageBarrier(cmd,
                VK_ACCESS_NONE,
                VK_ACCESS_NONE,
                VK_IMAGE_LAYOUT_UNDEFINED,  //discard is OK.
                VK_IMAGE_LAYOUT_GENERAL,
                gd->vkGpuTex,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
        }

        // build the lads. this is where we separate the men from the boys.
        {
            constexpr auto infoCount = 1u;

            // NOTE: the data here is going to be static because we need to persist until we "end"
            // the command buffer?
            static VkAccelerationStructureBuildRangeInfoKHR blasBuildRange = {
                1,  // prim count. TODO: i think this is same info as e.g. "AABB counts for geometries".
                0,  // offset into AABB data.
                0,  // only need for triangle data.
                0   // TODO: don't think this is relevant for blas.
            };
            ;
            static VkAccelerationStructureBuildRangeInfoKHR *blasBuildRanges[infoCount];
            blasBuildRanges[0] = &blasBuildRange;

            vkCmdBuildAccelerationStructuresKHR(cmd,
                // TODO: info count here changes to 2.
                infoCount,  // number of acceleration structures to build.
                &blasBuildInfo,
                (const VkAccelerationStructureBuildRangeInfoKHR *const *)blasBuildRanges);

            VkBufferMemoryBarrier barrier = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            barrier.srcAccessMask         = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
            barrier.dstAccessMask         = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR;
            barrier.buffer                = gd->vkBlasBuffer;
            barrier.offset                = 0;
            barrier.size                  = blasPrebuildInfo.accelerationStructureSize;

            vkCmdPipelineBarrier(cmd,
                VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
                0,         //dependencyFlags,
                0,         //                                   memoryBarrierCount,
                nullptr,   //                      pMemoryBarriers,
                1,         //                               bufferMemoryBarrierCount,
                &barrier,  //                pBufferMemoryBarriers,
                0,         //                                    imageMemoryBarrierCount,
                nullptr);

            // NOTE: the data here is going to be static because we need to persist until we "end"
            // the command buffer?
            static VkAccelerationStructureBuildRangeInfoKHR tlasBuildRange = {
                1,  // prim count. TODO: i think this is same info as e.g. "AABB counts for geometries".
                0,  // offset into instance data.
                0,
                0  // no transform offset. TODO: not sure what this if for.
            };
            ;
            static VkAccelerationStructureBuildRangeInfoKHR *tlasBuildRanges[infoCount];
            tlasBuildRanges[0] = &tlasBuildRange;

            vkCmdBuildAccelerationStructuresKHR(cmd,
                infoCount,  // number of acceleration structures to build.
                &tlasBuildInfo,
                (const VkAccelerationStructureBuildRangeInfoKHR *const *)tlasBuildRanges);
        }

        // submit and wait for that work.
        VK_CHECK(vkEndCommandBuffer(cmd));

        VkSubmitInfo si       = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers    = &cmd;
        (vkQueueSubmit(gd->vkQueue, 1, &si, gd->vkFence));  //NOTE: not multithreaded yet.

        uint64_t tenSeconds = 1000 * 1000 * 1000 * 10u;
        WaitForAndResetFence(gd->vkDevice, &gd->vkFence, tenSeconds);

        // reset the command list so that we can reuse later as the "copy" command list.
        VK_CHECK(vkResetCommandBuffer(cmd, 0));
        VK_CHECK(vkResetCommandPool(gd->vkDevice, gd->commandPool, 0));

    }  // end setup the acceleration structure.

    // create/write the descriptor set for use with all pipelines.
    {
        // NOTE: this kind of API here is really not making sense to me. TODO.
        // why is it that I specify the pools (which describe the shape of this pool, and therefore the max descriptors that can be made).
        // I think the idea is that a descriptor set is a higher level structure, so its like the set pulls from the lower-level pools.
        // just, really odd.

        // begin by create the pool.
        VkDescriptorPoolSize pools[3] = {{VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 2},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1},
            {VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1}};

        VkDescriptorPoolCreateInfo ci = {VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        ci.maxSets                    = 1;  //TODO.
        ci.poolSizeCount              = _countof(pools);
        ci.pPoolSizes                 = pools;
        VK_CHECK(vkCreateDescriptorPool(gd->vkDevice, &ci, nullptr, &gd->descPool));

        VkDescriptorSetAllocateInfo allocInfo = {VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorSetCount          = 1;
        allocInfo.descriptorPool              = gd->descPool;
        allocInfo.pSetLayouts                 = &gd->descSet;

        VK_CHECK(vkAllocateDescriptorSets(gd->vkDevice, &allocInfo, &gd->theDescSet));

        VkDescriptorImageInfo imageInfo = {
            VK_NULL_HANDLE,  // sampler.
            gd->vkCpuTexView,
            VK_IMAGE_LAYOUT_GENERAL  // layout at the time of access through this descriptor.
        };

        VkDescriptorImageInfo imageInfo2 = {
            VK_NULL_HANDLE,  // sampler.
            gd->vkGpuTexView,
            VK_IMAGE_LAYOUT_GENERAL  // layout at the time of access through this descriptor.
        };

        VkWriteDescriptorSet writes[4] = {};

        writes[0].sType           = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[0].dstSet          = gd->theDescSet;
        writes[0].dstBinding      = 1;  // binding within set to write.
        writes[0].descriptorCount = 1;
        writes[0].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[0].pImageInfo      = &imageInfo;

        writes[1].sType           = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[1].dstSet          = gd->theDescSet;
        writes[1].dstBinding      = 0;  // binding within set to write.
        writes[1].descriptorCount = 1;
        writes[1].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[1].pImageInfo      = &imageInfo2;

        VkDescriptorBufferInfo bufferInfo = {gd->vkWorldBuffer, 0, worldSize};

        writes[2].sType           = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[2].dstSet          = gd->theDescSet;
        writes[2].dstBinding      = 2;  // binding within set to write.
        writes[2].descriptorCount = 1;
        writes[2].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[2].pBufferInfo     = &bufferInfo;

        VkWriteDescriptorSetAccelerationStructureKHR tlasInfo = {
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
        tlasInfo.accelerationStructureCount = 1;
        tlasInfo.pAccelerationStructures    = &gd->vkTlas;

        writes[3].sType           = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[3].dstSet          = gd->theDescSet;
        writes[3].dstBinding      = 3;  // binding within set to write.
        writes[3].descriptorCount = 1;
        writes[3].descriptorType  = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        writes[3].pNext           = &tlasInfo;

        vkUpdateDescriptorSets(gd->vkDevice,
            _countof(writes),  // write count.
            writes,
            0,       //copy count.
            nullptr  // copies.
        );
    }

    // setup the raytracing pipeline
    constexpr size_t shaderGroupCount = 4;
    {
        VkRayTracingPipelineCreateInfoKHR ci = {VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR};

        VkPipelineShaderStageCreateInfo stages[5] = {};
        int                             idx       = 0;

        auto lib = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"", L"lib_6_3");

        auto raygen = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"ray_gen_shader", L"lib_6_3");

        auto miss = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"miss_main", L"lib_6_3");

        auto hit = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"closest_hit_simple", L"lib_6_3");

        auto sphere = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"intersection_sphere", L"lib_6_3");

        auto plane = ae::VK::loadShaderModule(gd->vkDevice, "res\\shader.hlsl", L"intersection_plane", L"lib_6_3");

        // setup stages.

        // NOTE: notice that the lib var goes out of scope here and is still
        // used as data later. this is OK since we copy the VK handle by value.

        stages[idx].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[idx].stage  = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        stages[idx].module = raygen;
        stages[idx].pName  = "ray_gen_shader";
        idx++;

        stages[idx].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[idx].stage  = VK_SHADER_STAGE_MISS_BIT_KHR;
        stages[idx].module = miss;
        stages[idx].pName  = "miss_main";
        idx++;

        stages[idx].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[idx].stage  = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
        stages[idx].module = sphere;
        stages[idx].pName  = "intersection_sphere";
        idx++;

        stages[idx].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[idx].stage  = VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
        stages[idx].module = plane;
        stages[idx].pName  = "intersection_plane";
        idx++;

        // TODO: the error is here, for some reason, our closest hit shader is just not working ...
        stages[idx].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[idx].stage  = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
        stages[idx].module = hit;
        stages[idx].pName  = "closest_hit_simple";
        idx++;

        assert(idx == _countof(stages));
        ci.stageCount = idx;
        ci.pStages    = stages;

        VkRayTracingShaderGroupCreateInfoKHR groups[shaderGroupCount] = {};

        // raygen.
        groups[0].sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[0].type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[0].generalShader      = 0;
        groups[0].closestHitShader   = VK_SHADER_UNUSED_KHR;
        groups[0].anyHitShader       = VK_SHADER_UNUSED_KHR;
        groups[0].intersectionShader = VK_SHADER_UNUSED_KHR;

        // miss.
        groups[1].sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[1].type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[1].generalShader      = 1;
        groups[1].closestHitShader   = VK_SHADER_UNUSED_KHR;
        groups[1].anyHitShader       = VK_SHADER_UNUSED_KHR;
        groups[1].intersectionShader = VK_SHADER_UNUSED_KHR;

        // sphere.
        groups[2].sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[2].type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
        groups[2].generalShader      = VK_SHADER_UNUSED_KHR;
        groups[2].closestHitShader   = 4;
        groups[2].anyHitShader       = VK_SHADER_UNUSED_KHR;
        groups[2].intersectionShader = 2;

        // plane.
        groups[3].sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[3].type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR;
        groups[3].generalShader      = VK_SHADER_UNUSED_KHR;
        groups[3].closestHitShader   = 4;
        groups[3].anyHitShader       = VK_SHADER_UNUSED_KHR;
        groups[3].intersectionShader = 3;

        ci.groupCount = shaderGroupCount;
        ci.pGroups    = groups;

        ci.maxPipelineRayRecursionDepth = 1;  //TODO ????
                                              //    const VkPipelineLibraryCreateInfoKHR*                pLibraryInfo;
                                              //const VkRayTracingPipelineInterfaceCreateInfoKHR*    pLibraryInterface;
                                              //  const VkPipelineDynamicStateCreateInfo*              pDynamicState;
        ci.layout = gd->pipelineLayout;
        //    VkPipeline                                           basePipelineHandle;
        // int32_t                                              basePipelineIndex;

        // TODO: for some reason this shit is failing ... that's really stupid!
        // hey! so i removed some crap and I managed to get this to not fail and give back some infos.
        VK_CHECK(vkCreateRayTracingPipelinesKHR(gd->vkDevice,
            VK_NULL_HANDLE,  //VkDeferredOperationKHR                      deferredOperation,
            VK_NULL_HANDLE,  //VkPipelineCache                             pipelineCache,
            1,               //uint32_t                                    createInfoCount,
            &ci,             //const VkRayTracingPipelineCreateInfoKHR*    pCreateInfos,
            nullptr,         //alloc callback.
            &gd->vkRayPipeline));
    }

    // get some infos about the device.
    {
        VkPhysicalDeviceRayTracingPipelinePropertiesKHR rayProps = {
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR};
        VkPhysicalDeviceProperties2 props = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
        props.pNext                       = &rayProps;
        vkGetPhysicalDeviceProperties2(gd->vkGpu, &props);

        gd->vkShaderGroupHandleSize     = rayProps.shaderGroupHandleSize;
        gd->vkShaderGroupTableAlignment = rayProps.shaderGroupBaseAlignment;
    }

    // setup the shader binding tables.
    {
        // allocate the buffer for the shader table.
        size_t bufferSize =
            gd->vkShaderGroupTableAlignment * 3 +
            gd->vkShaderGroupTableAlignment * 4;  // TODO: this is a little big, but it should do the trick.

        void *data;

        ae::VK::createUploadBufferDumb(gd->vkDevice,
            bufferSize,
            gd->vkUploadHeapIdx,
            VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,  //TODO: is there more stuff needed here?
            &gd->vkShaderTable,
            &gd->vkShaderTableBacking,
            &data);

        if (data) {
            uint8_t *pData = (uint8_t *)data;

            // raygen.
            VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(gd->vkDevice,
                gd->vkRayPipeline,
                0,  //uint32_t                                    firstGroup,
                1,  //group count
                gd->vkShaderGroupHandleSize,
                pData));
            pData += gd->vkShaderGroupTableAlignment;

            // miss.
            VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(gd->vkDevice,
                gd->vkRayPipeline,
                1,  //uint32_t                                    firstGroup,
                1,  //group count
                gd->vkShaderGroupHandleSize,
                pData));
            pData += gd->vkShaderGroupTableAlignment;

            // hit group table.
            VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(gd->vkDevice,
                gd->vkRayPipeline,
                2,  //uint32_t                                    firstGroup,
                2,  //group count
                gd->vkShaderGroupHandleSize * 2,
                pData));

            ae::VK::flushAndUnmapUploadBuffer(gd->vkDevice, bufferSize, gd->vkShaderTableBacking);
        }

    }  // END setup the shader bind tables.

    // start recording the cmd buffer to do the good copy work :D
    {
        auto cmd = gd->cmdBuf;

        VkCommandBufferBeginInfo beginInfo = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, gd->vkComputePipeline);

        // NOTE: there seems to be a lot of crap going on with these descriptor sets,
        // so I'm just going to break down here.
        //
        // you have the pipeline layout which is the "signature" of the pipeline. it is what the pipeline expects.
        // a single layout consists of a set of descriptor set layouts.
        //
        // this is to say that the pipeline is merely a set of descriptor sets.
        //
        // the layouts are just that, layouts.
        // desc set layout is used to help us alloc the desc set.
        //
        // when we go to actually do stuff, we need to bind real descriptor sets into the pipeline.
        // and so that's what the call is below.
        //
        // and when we do the bind, we're specifying the index into the global set (so elem is desc set).
        //
        // that's a whole lotta crap, man! surely this can be done better from an API standpoint :D
        vkCmdBindDescriptorSets(cmd,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            gd->pipelineLayout,
            0,  // set number of first descriptor to bind.
            1,
            &gd->theDescSet,  // const VkDescriptorSet*                      pDescriptorSets,
            0,                // dynamic offsets
            nullptr           // ^
        );

        // dispatch using the bound pipeline.
        vkCmdDispatch(cmd, ae::math::div_ceil(winInfo.width, 16), ae::math::div_ceil(winInfo.height, 16), 1);

        emitSingleImageBarrier(cmd,
            VK_ACCESS_SHADER_WRITE_BIT,  // sync with prior shader writes.
            VK_ACCESS_TRANSFER_READ_BIT,
            VK_IMAGE_LAYOUT_GENERAL,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            gd->vkCpuTex,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT);

        VkBufferImageCopy region = {
            0,  // buffer offset.
            0,  // buffer len
            0,  // buffer height
            // VkImageSubresourceLayers
            {
                VK_IMAGE_ASPECT_COLOR_BIT,
                0,  // mip level.
                0,  //array base.
                1   //array layers
            },
            {0, 0, 0},                                                      //offset
            {gameMemory->backbufferWidth, gameMemory->backbufferHeight, 1}  //extent.
        };
        vkCmdCopyImageToBuffer(cmd, gd->vkCpuTex, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, gd->vkCpuTexBuffer, 1, &region);

        // NOTE: for this barrier we do not care about sync since we always wait for this command list.
        emitSingleImageBarrier(cmd,
            VK_ACCESS_TRANSFER_READ_BIT,
            VK_ACCESS_NONE,
            VK_IMAGE_LAYOUT_UNDEFINED,  //discard is OK.
            VK_IMAGE_LAYOUT_GENERAL,
            gd->vkCpuTex,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);

        // TODO: do we need to transfer the buffer to be in the general layout for CPU reading??

        //        hr = (cmd->Close());
        VK_CHECK(vkEndCommandBuffer(cmd));

    }  // end init the cmdBuf for doing the good copy work.

}  // end InitVK

void emitSingleImageBarrier(VkCommandBuffer cmd,
    VkAccessFlags                           src,
    VkAccessFlags                           dst,
    VkImageLayout                           srcLayout,
    VkImageLayout                           dstLayout,
    VkImage                                 img,
    VkPipelineStageFlags                    before,
    VkPipelineStageFlags                    after)
{
    VkImageMemoryBarrier barrier = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    barrier.srcAccessMask        = src;
    barrier.dstAccessMask        = dst;
    barrier.oldLayout            = srcLayout;
    barrier.newLayout            = dstLayout;
    barrier.image                = img;
    barrier.subresourceRange     = {
        VK_IMAGE_ASPECT_COLOR_BIT,
        0,  // base mip level.
        1,  //mip count.
        0,  //array base.
        1   //array layers
    };

    vkCmdPipelineBarrier(cmd,
        before,   //NOTE: no need to sync with anything before. we stall GPU between cmd buffers of this kind.
        after,    // dst stage.
        0,        //dependencyFlags,
        0,        //                                   memoryBarrierCount,
        nullptr,  //                      pMemoryBarriers,
        0,        //                               bufferMemoryBarrierCount,
        nullptr,  //                pBufferMemoryBarriers,
        1,        //                                    imageMemoryBarrierCount,
        &barrier);
};

void WaitForAndResetFence(VkDevice device, VkFence *pFence, uint64_t waitTime)
{
    VkResult result = (vkWaitForFences(device,
        1,
        pFence,
        // wait until all the fences are signaled. this blocks the CPU thread.
        VK_TRUE,
        waitTime));
    // TODO: there was an interesting bug where if I went 1ms on the timeout, things were failing,
    // where the fence was reset too soon. figure out what what going on in that case.

    if (result == VK_SUCCESS) {
        // reset fences back to unsignaled so that can use em' again;
        vkResetFences(device, 1, pFence);
    } else {
        AELoggerError("some error occurred during the fence wait thing., %s", ae::VK::VkResultToString(result));
    }
}

#undef VK_CHECK

#endif  // AUTOMATA_ENGINE_VK_BACKEND

#if AUTOMATA_ENGINE_DX12_BACKEND
void InitD3D12(game_data *gd)
{
    HRESULT hr;
#define INIT_FAIL_CHECK()                                                                                              \
    if (hr != S_OK) {                                                                                                  \
        AELoggerError("INIT_FAIL_CHECK failed at line=%d with hr=%x", __LINE__, hr);                                   \
        return;                                                                                                        \
    }

    // init the d3d device (and debug stuff)
    {
        UINT dxgiFactoryFlags = 0;

#if defined(_DEBUG)
        {
            if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&gd->debugController)))) {
                gd->debugController->EnableDebugLayer();
                dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
            }
        }
#endif

        IDXGIFactory2 *dxgiFactory = NULL;
        hr                         = CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&dxgiFactory));
        INIT_FAIL_CHECK();
        defer(COM_RELEASE(dxgiFactory));

        IDXGIAdapter1 *hardwareAdapter = nullptr;
        ae::DX::findHardwareAdapter(dxgiFactory, &hardwareAdapter);
        if (!hardwareAdapter) return;
        defer(COM_RELEASE(hardwareAdapter));

        (hr = D3D12CreateDevice(hardwareAdapter,
             D3D_FEATURE_LEVEL_12_0,  // minimum feature level
             IID_PPV_ARGS(&gd->d3dDevice)));
        INIT_FAIL_CHECK();
    }

    // create the root sig.
    {
        // Determine supported root sig version.
        D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData = {};
        featureData.HighestVersion                    = D3D_ROOT_SIGNATURE_VERSION_1_1;
        if (FAILED(
                gd->d3dDevice->CheckFeatureSupport(D3D12_FEATURE_ROOT_SIGNATURE, &featureData, sizeof(featureData)))) {
            featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;
        }

        CD3DX12_DESCRIPTOR_RANGE1 ranges[2];
        CD3DX12_ROOT_PARAMETER1   rootParameters[3];

        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV,
            2,                                         // num descriptors.
            0,                                         // BaseShaderRegister: map to register(u0) in HLSL.
            0,                                         // RegisterSpace: map to register(u0, space0) in HLSL.
            D3D12_DESCRIPTOR_RANGE_FLAG_DATA_VOLATILE  // descriptor static,
                                                       // data pointed to is
                                                       // not.
        );

        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV,
            1,  // num descriptors.
            0,  // b0
            0,  // space0
            D3D12_DESCRIPTOR_RANGE_FLAG_DATA_STATIC);

        rootParameters[0].InitAsDescriptorTable(_countof(ranges), ranges, D3D12_SHADER_VISIBILITY_ALL);

        rootParameters[1].InitAsConstants(5,  // num constants.
            0,                                // register.
            1                                 // space.
        );

        rootParameters[2].InitAsShaderResourceView(0,  // register
            0                                          // space.
        );

        CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
        rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters);

        // TODO: could make a func called "serialize root sig".
        ID3DBlob *signature;
        ID3DBlob *error;
        defer(COM_RELEASE(signature));
        defer(COM_RELEASE(error));

        if ((hr = D3DX12SerializeVersionedRootSignature(
                 &rootSignatureDesc, featureData.HighestVersion, &signature, &error)) == S_OK) {
            (hr = gd->d3dDevice->CreateRootSignature(
                 0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&gd->rootSig)));
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
        bool        r = ae::HLSL::compileBlobFromFile(shaderFilePath, L"copy_shader", L"cs_6_0", &computeShader);
        if (!r || !computeShader) {
            // compileshader will print already.
            return;
        }

        D3D12_COMPUTE_PIPELINE_STATE_DESC computePipelineDesc = {};
        computePipelineDesc.pRootSignature                    = gd->rootSig;
        computePipelineDesc.CS =
            CD3DX12_SHADER_BYTECODE(computeShader->GetBufferPointer(), computeShader->GetBufferSize());

        hr = (gd->d3dDevice->CreateComputePipelineState(&computePipelineDesc, IID_PPV_ARGS(&gd->computePipelineState)));
        INIT_FAIL_CHECK();
    }

    LPCWSTR c_raygenShaderName          = L"ray_gen_shader";
    LPCWSTR c_missShaderName            = L"miss_main";
    LPCWSTR c_planeIntersectShaderName  = L"intersection_plane";
    LPCWSTR c_sphereIntersectShaderName = L"intersection_sphere";
    LPCWSTR c_closestHitShaderName      = L"closesthit_main";

    LPCWSTR c_planeHitgroupName  = L"hitgroup_plane";
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

        CD3DX12_STATE_OBJECT_DESC raytracingPipeline{D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE};

        IDxcBlob *rayLib = nullptr;
        defer(COM_RELEASE(rayLib));

        // compile the raytracing shaders.
        // TODO: this is dumb because we are compiling the same file twice,
        // plus reading it from disk twice.
        const char *shaderFilePath = "res\\shader.hlsl";
        bool        r              = ae::HLSL::compileBlobFromFile(shaderFilePath,
            L"",  // TODO:empty entry works??,
            L"lib_6_3",
            &rayLib);
        if (!r || !rayLib) {
            // compileshader will print already.
            return;
        }

        auto                  lib     = raytracingPipeline.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(rayLib->GetBufferPointer(), rayLib->GetBufferSize());

        lib->SetDXILLibrary(&libdxil);
        // NOTE: we do not want to export all shader signatures since
        // copy_shader is not part of our raytracing pipeline.
        lib->DefineExport(c_raygenShaderName);
        lib->DefineExport(c_missShaderName);
        lib->DefineExport(c_planeIntersectShaderName);
        lib->DefineExport(c_sphereIntersectShaderName);
        lib->DefineExport(c_closestHitShaderName);

        // create hit group plane
        auto hitGroup = raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
        hitGroup->SetIntersectionShaderImport(c_planeIntersectShaderName);
        hitGroup->SetHitGroupExport(c_planeHitgroupName);
        hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
        hitGroup->SetClosestHitShaderImport(c_closestHitShaderName);

        // create hit group sphere
        auto hitGroup2 = raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
        hitGroup2->SetIntersectionShaderImport(c_sphereIntersectShaderName);
        hitGroup2->SetHitGroupExport(c_sphereHitgroupName);
        hitGroup2->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
        hitGroup2->SetClosestHitShaderImport(c_closestHitShaderName);

        // Define the maximum sizes in bytes for the ray payload and attribute
        // structure.
        auto shaderConfig  = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
        UINT payloadSize   = 16 * sizeof(float);  // TODO: this is copy-pasta.
        UINT attributeSize = 8 * sizeof(float);  // TODO: this is copy-pasta.
        shaderConfig->Config(payloadSize, attributeSize);

        auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
        globalRootSignature->SetRootSignature(gd->rootSig);  // TODO: we can just reuse the same root sig
        // here, right?

        auto pipelineConfig    = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
        UINT maxRecursionDepth = 8;  // no more than 7 bounces (+1 to include the primary ray).
        pipelineConfig->Config(maxRecursionDepth);

        hr = device->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&gd->rayStateObject));
        INIT_FAIL_CHECK();
    }

    // upload buffers list.
    upload_buffer_helper uBuffer = {};

    // build the shader table which holds the shader records.
    // (for raytracing)
    {
        // some sizes.
        constexpr UINT shaderIdentifierSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
        constexpr UINT recordAlign          = D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT;
        constexpr UINT shaderTableAlign     = D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;
        static_assert(shaderIdentifierSize <= shaderTableAlign, "we assume shaderIdentifierSize <= shaderTableAlign.");

#define MAX_RECORDS 10

        // shader records to put in there.
        struct {
            struct {
                void   *ident;
                LPCWSTR name;
            } records[MAX_RECORDS];
            size_t recordCount;
        } shaderTables[] = {{{{nullptr, c_raygenShaderName}}, 1},
            {{{nullptr, c_missShaderName}}, 1},
            {{{nullptr, c_planeHitgroupName}, {nullptr, c_sphereHitgroupName}}, 2}};

        uint8_t *data       = nullptr;
        uBuffer.curr().size = shaderTableAlign * _countof(shaderTables);
        uBuffer.curr().src  = ae::DX::AllocUploadBuffer(gd->d3dDevice,
            uBuffer.curr().size,
            (void **)&data);  // begins mapped.

        // create the backing resource for the shader table.
        (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
             D3D12_HEAP_FLAG_NONE,
             &CD3DX12_RESOURCE_DESC::Buffer(uBuffer.curr().size),

             // state needed for things read from a shader that is not a pixel
             // shader.
             D3D12_RESOURCE_STATE_COPY_DEST,

             nullptr,  // optimized clear.
             IID_PPV_ARGS(&gd->rayShaderTable)));
        INIT_FAIL_CHECK();

        // TODO: maybe we use diff resources for the shader tables?
        //  there is currently some wasted memory due to table start addr
        //  requirements.
        uBuffer.curr().dst       = gd->rayShaderTable;
        uBuffer.curr().initState = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;

        ID3D12StateObjectProperties *o;
        defer(COM_RELEASE(o));
        gd->rayStateObject->QueryInterface(&o);

        if (data) {
            auto totalSize = uBuffer.curr().size;
            memset(data, 0, totalSize);
            for (int j = 0; j < _countof(shaderTables); j++) {
                auto    &shaderRecords = shaderTables[j].records;
                uint8_t *pRecord       = data;
                auto     rc            = shaderTables[j].recordCount;
                assert(rc <= MAX_RECORDS);
                for (int i = 0; i < rc; i++) {
                    auto v = shaderRecords[i].ident = o->GetShaderIdentifier(shaderRecords[i].name);
                    assert(v);

                    assert(size_t(pRecord + shaderIdentifierSize - data) <= totalSize);
                    memcpy(pRecord, v, shaderIdentifierSize);
                    pRecord += recordAlign;
                }

                data += shaderTableAlign;
            }

        } else {
            AELoggerError("oopsy");
        }

#undef MAX_RECORDS

        uBuffer.curr().src->Unmap(0,  // subres
            nullptr                   // entire subresource was modified.
        );

        uBuffer.iter();
    }

    ae::game_window_info_t winInfo         = automata_engine::platform::getWindowInfo();
    const size_t           sceneBufferSize = ae::math::align_up(sizeof(dxr_world), 256ull);

    auto fnCreateCmdList = [&](ID3D12CommandAllocator    **ppAllocator,
                               ID3D12GraphicsCommandList **ppCmdList,
                               ID3D12Fence               **ppFence,
                               HANDLE                     *pEvent) {
        // command allocator.
        (hr = gd->d3dDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(ppAllocator)));
        INIT_FAIL_CHECK();

        // command list.
        (hr = gd->d3dDevice->CreateCommandList(0,  // nodemask, this is for single GPU scenarios.
             D3D12_COMMAND_LIST_TYPE_DIRECT,
             *ppAllocator,  // how the device allocates commands for this
                            // list.
             nullptr,       // initial pipeline state.
             IID_PPV_ARGS(ppCmdList)));

        (hr = gd->d3dDevice->CreateFence(0,  // init value,
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
        queueDesc.Type                     = D3D12_COMMAND_LIST_TYPE_DIRECT;
        (hr = gd->d3dDevice->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&gd->commandQueue)));
        INIT_FAIL_CHECK();
    }

    // create all the cmd list stuffs.
    fnCreateCmdList(&gd->commandAllocator, &gd->commandList, &gd->fence, &gd->fenceEvent);
    gd->fenceValue = 0;
    for (int i = 0; i < THREAD_COUNT; i++) {
        fnCreateCmdList(&gd->rayCmdAllocators[i], &gd->rayCmdLists[i], &gd->rayFences[i], &gd->rayFenceEvents[i]);
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
        constexpr auto AABBcount         = 2u;
        constexpr auto geoCount          = 2u;
        constexpr auto tlasInstanceCount = 4u;

        // we begin by making all the BLASes.
        {
            size_t AABBsStride = sizeof(D3D12_RAYTRACING_AABB);
            size_t AABBsSize   = AABBsStride * AABBcount;

            // upload aabbs.
            {
                void *data;
                gd->AABBs = ae::DX::AllocUploadBuffer(gd->d3dDevice, AABBsSize, &data);

                if (data) {
                    memset(data, 0, AABBsSize);

                    // make plane AABB.
                    D3D12_RAYTRACING_AABB AABBs[AABBcount] = {};

                    auto &planeAABB = AABBs[0];
                    planeAABB.MinX  = -1000;
                    planeAABB.MinY  = -0.01;
                    planeAABB.MinZ  = -1000;
                    planeAABB.MaxX  = 1000;
                    planeAABB.MaxY  = 0.01;
                    planeAABB.MaxZ  = 1000;

                    // sphere AABB.
                    // NOTE: default sphere has a radius of 1.
                    auto &sphereAABB = AABBs[1];
                    sphereAABB.MinX = sphereAABB.MinY = sphereAABB.MinZ = -1;
                    sphereAABB.MaxX = sphereAABB.MaxY = sphereAABB.MaxZ = 1;

                    memcpy(data, AABBs, AABBsSize);
                } else {
                    AELoggerLog("oops");
                }

                gd->AABBs->Unmap(0,
                    nullptr  // entire subres modified.
                );
            }

            // setup geometries from aabbs.
            D3D12_RAYTRACING_GEOMETRY_DESC geometryDescs[geoCount] = {};
            {
                auto &planeDesc            = geometryDescs[0];
                planeDesc.Type             = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                planeDesc.AABBs.AABBCount  = 1;
                planeDesc.AABBs.AABBs      = {gd->AABBs->GetGPUVirtualAddress(), AABBsStride};
                auto &sphereDesc           = geometryDescs[1];
                sphereDesc.Type            = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                sphereDesc.AABBs.AABBCount = 1;
                sphereDesc.AABBs.AABBs     = {gd->AABBs->GetGPUVirtualAddress() + AABBsStride, AABBsStride};
            }

            auto fnCreateBlasFromSingleGeometryDesc = [&](int idx) {
                D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS bottomLevelInputs = {};

                // get prebuild info.
                D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO
                bottomLevelPrebuildInfo = {};
                {
                    bottomLevelInputs.DescsLayout    = D3D12_ELEMENTS_LAYOUT_ARRAY;
                    bottomLevelInputs.Type           = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
                    bottomLevelInputs.NumDescs       = 1;
                    bottomLevelInputs.pGeometryDescs = &geometryDescs[idx];

                    device->GetRaytracingAccelerationStructurePrebuildInfo(
                        &bottomLevelInputs, &bottomLevelPrebuildInfo);
                }

                // create blas.
                (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
                     D3D12_HEAP_FLAG_NONE,
                     &CD3DX12_RESOURCE_DESC::Buffer(
                         bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
                     D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
                     nullptr,  // optimized clear.
                     IID_PPV_ARGS(&gd->BLASes[idx])));

                // TODO: since we are in a lambda, init fail check is only going to fail
                //  to out of this func, but will not return any further.
                INIT_FAIL_CHECK();

                // TODO: we could create one scratch buffer that is large enough to hold
                //  for all BLAS and TLAS.
                //
                //  create scratch.
                UINT64 scratchSize = bottomLevelPrebuildInfo.ScratchDataSizeInBytes;
                (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
                     D3D12_HEAP_FLAG_NONE,
                     &CD3DX12_RESOURCE_DESC::Buffer(scratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                     nullptr,  // optimized clear.
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
                    bottomLevelBuildDesc.DestAccelerationStructureData = gd->BLASes[idx]->GetGPUVirtualAddress();
                }

                // record the building.
                cmd->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0, nullptr);
                cmd->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(gd->BLASes[idx]));
            };

            for (int i = 0; i < geoCount; i++) fnCreateBlasFromSingleGeometryDesc(i);
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
            topLevelPrebuildInfo                                                = {};
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS topLevelInputs = {};
            {
                topLevelInputs.Type        = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
                topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
                topLevelInputs.NumDescs    = tlasInstanceCount;

                device->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
            }

            // alloc tlas.
            (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
                 D3D12_HEAP_FLAG_NONE,
                 &CD3DX12_RESOURCE_DESC::Buffer(
                     topLevelPrebuildInfo.ResultDataMaxSizeInBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
                 D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
                 nullptr,  // optimized clear.
                 IID_PPV_ARGS(&gd->tlas)));
            INIT_FAIL_CHECK();

            // instantiate blas instances.
            {
                size_t                         bSize = tlasInstanceCount * sizeof(D3D12_RAYTRACING_INSTANCE_DESC);
                D3D12_RAYTRACING_INSTANCE_DESC instanceDescs[tlasInstanceCount] = {};

                int baseInstance       = 0;
                int instanceDescOffset = 0;
                int totalInstanceCount = 0;

                assert(g_world.planeCount == 1);

                for (int i = 0; i < g_world.planeCount; i++) {
                    auto &inst = instanceDescs[baseInstance + i];

                    // TODO: our application does not currently support arbitrary planes.
                    // we only support the inf plane with normal pointing straight up.
                    inst.Transform[0][0] = inst.Transform[1][1] = inst.Transform[2][2] = 1;

                    inst.InstanceID = i;

                    // TODO: maybe we care about different blas having different instance
                    // masks, but for now, 0xff for all blas is what we do.
                    inst.InstanceMask = 0xFF;  // bitwise OR with what given on TraceRay
                                               // side, and if zero, ignore.

                    inst.AccelerationStructure               = gd->BLASes[baseInstance]->GetGPUVirtualAddress();
                    inst.InstanceContributionToHitGroupIndex = baseInstance;

                    totalInstanceCount++;
                }

                baseInstance++;
                instanceDescOffset += g_world.planeCount;

                for (int i = 0; i < g_world.sphereCount; i++) {
                    auto &inst = instanceDescs[instanceDescOffset + i];

                    // TODO: our application does not support arbitrary radii spheres,
                    //  although, that would be relatively trivial to support.
                    inst.Transform[0][0] = inst.Transform[1][1] = inst.Transform[2][2] = 1;
                    // setup translation.
                    //  NOTE: the transform is setup as 3 float4's contiguous in memory.
                    //  where if you were to write this on a sheet of paper, you would
                    //  be looking at a matrix of 4 columns and 3 rows.
                    inst.Transform[0][3] = g_world.spheres[i].p.x;
                    inst.Transform[1][3] = g_world.spheres[i].p.y;
                    inst.Transform[2][3] = g_world.spheres[i].p.z;
                    // memcpy(&inst.Transform[3], &g_world.spheres[i].p, sizeof(float)*3);

                    inst.InstanceID = i;

                    // TODO: maybe we care about different blas having different instance
                    // masks, but for now, 0xff for all blas is what we do.
                    inst.InstanceMask = 0xFF;  // bitwise OR with what given on TraceRay
                                               // side, and if zero, ignore.

                    inst.AccelerationStructure               = gd->BLASes[baseInstance]->GetGPUVirtualAddress();
                    inst.InstanceContributionToHitGroupIndex = baseInstance;

                    totalInstanceCount++;
                }

                assert(totalInstanceCount == tlasInstanceCount);

                void *data;
                gd->tlasInstances = ae::DX::AllocUploadBuffer(gd->d3dDevice, bSize, &data);

                if (data) {
                    memcpy(data, &instanceDescs, bSize);
                } else {
                    AELoggerLog("oops");
                }

                gd->tlasInstances->Unmap(0,
                    nullptr  // entire subres modified.
                );
            }

            // create the required scratch buffer.
            UINT64 scratchSize = topLevelPrebuildInfo.ScratchDataSizeInBytes;

            // TODO: creating a buffer with default args like this might be
            // something common that we want to pull out into AE.
            (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
                 D3D12_HEAP_FLAG_NONE,
                 &CD3DX12_RESOURCE_DESC::Buffer(scratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
                 D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                 nullptr,  // optimized clear.
                 IID_PPV_ARGS(&gd->tlasScratch)));
            INIT_FAIL_CHECK();

            auto cmd = getRayCmdList(gd->commandList);

            // Top Level Acceleration Structure desc
            D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
            {
                topLevelInputs.InstanceDescs                       = gd->tlasInstances->GetGPUVirtualAddress();
                topLevelBuildDesc.Inputs                           = topLevelInputs;
                topLevelBuildDesc.DestAccelerationStructureData    = gd->tlas->GetGPUVirtualAddress();
                topLevelBuildDesc.ScratchAccelerationStructureData = gd->tlasScratch->GetGPUVirtualAddress();
            }

            cmd->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);
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
        (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
             D3D12_HEAP_FLAG_NONE,
             &CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_R8G8B8A8_UNORM,
                 winInfo.width,
                 winInfo.height,
                 1,
                 0,
                 1,
                 0,
                 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
             D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
             nullptr,  // optimized clear.
             IID_PPV_ARGS(&gd->gpuTex)));
        INIT_FAIL_CHECK();

        // create the texture for CPU side memcpy to backbuffer, for blit to
        // window.
        D3D12_HEAP_PROPERTIES hPROP = {};
        hPROP.Type                  = D3D12_HEAP_TYPE_CUSTOM;
        hPROP.MemoryPoolPreference  = D3D12_MEMORY_POOL_L0;  // system RAM.

        // write back here is a cache protocol in which when the CPU writes to
        // the page (the virt mem mapped to the heap), this is written to cache
        // instead of whatever backs the cache (the heap). write back is a fair
        // protocol since the CPU does not require write access.
        hPROP.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_WRITE_BACK;

        (hr = gd->d3dDevice->CreateCommittedResource(&hPROP,
             D3D12_HEAP_FLAG_CREATE_NOT_ZEROED,
             &CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_R8G8B8A8_UNORM,
                 winInfo.width,
                 winInfo.height,
                 1,
                 0,
                 1,
                 0,
                 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
             D3D12_RESOURCE_STATE_COMMON,  /// initial access.
             nullptr,                      // optimized clear.
             IID_PPV_ARGS(&gd->cpuTex)));
        INIT_FAIL_CHECK();

        hr = gd->cpuTex->Map(0, NULL, nullptr);
        INIT_FAIL_CHECK();

        // create the buffer for storing information about the scene.
        {
            (hr = gd->d3dDevice->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
                 D3D12_HEAP_FLAG_CREATE_NOT_ZEROED,
                 &CD3DX12_RESOURCE_DESC::Buffer(sceneBufferSize),
                 D3D12_RESOURCE_STATE_COPY_DEST,  /// initial access.
                 nullptr,                         // optimized clear.
                 IID_PPV_ARGS(&gd->sceneBuffer)));
            INIT_FAIL_CHECK();

            // write to the upload buffer.
            {
                void *data               = nullptr;
                uBuffer.curr().size      = sceneBufferSize;
                uBuffer.curr().src       = ae::DX::AllocUploadBuffer(gd->d3dDevice,
                    uBuffer.curr().size,
                    (void **)&data);  // begins mapped.
                uBuffer.curr().dst       = gd->sceneBuffer;
                uBuffer.curr().initState = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;

                if (data) {
                    dxr_world w = SCENE_TO_DXR_WORLD();

                    memset(data, 0, sceneBufferSize);
                    memcpy(data, &w, sizeof(w));

                } else {
                    AELoggerError("oopsy");
                }

                uBuffer.curr().src->Unmap(0,  // subres
                    nullptr                   // entire subresource was modified.
                );

                uBuffer.iter();
            }
        }

        // create the CBV_SRV_UAV descriptor heap + descriptors.
        {
            D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
            srvHeapDesc.NumDescriptors             = 4;  // TODO: can we get some static verification that this is same
                                                         // as above (where we define root sig?)
            srvHeapDesc.Type  = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
            srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
            (hr = gd->d3dDevice->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&gd->descHeap)));
            INIT_FAIL_CHECK();

            D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
            uavDesc.Format                           = DXGI_FORMAT_R8G8B8A8_UNORM;
            uavDesc.ViewDimension                    = D3D12_UAV_DIMENSION_TEXTURE2D;

            CD3DX12_CPU_DESCRIPTOR_HANDLE heapHandle(gd->descHeap->GetCPUDescriptorHandleForHeapStart());

            gd->d3dDevice->CreateUnorderedAccessView(gd->gpuTex,
                nullptr,  // no counter.
                &uavDesc,
                heapHandle  // where to write the descriptor.
            );

            int off = gd->d3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
            heapHandle.Offset(1, off);

            gd->d3dDevice->CreateUnorderedAccessView(gd->cpuTex,
                nullptr,  // no counter.
                &uavDesc,
                heapHandle  // where to write the descriptor.
            );

            heapHandle.Offset(1, off);

            D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
            cbvDesc.BufferLocation                  = gd->sceneBuffer->GetGPUVirtualAddress();
            cbvDesc.SizeInBytes                     = sceneBufferSize;

            gd->d3dDevice->CreateConstantBufferView(&cbvDesc,
                heapHandle  // where to write the descriptor.
            );

            heapHandle.Offset(1, off);

            // TODO: there is no longer a need to be writing this descriptor.
            //       alternatively, we could go back to this approach instead of
            //       setting it in the root parameter. that approach was valid but we
            //       did not see the fruits since lots of other things were incorrect.
            //
            //  create the scene accel structure descriptor.
            D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc          = {};
            srvDesc.Shader4ComponentMapping                  = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
            srvDesc.ViewDimension                            = D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE;
            srvDesc.RaytracingAccelerationStructure.Location = gd->tlas->GetGPUVirtualAddress();

            gd->d3dDevice->CreateShaderResourceView(
                // gd->tlas,
                nullptr,
                &srvDesc,
                heapHandle);
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
            hr = (gd->fence->SetEventOnCompletion(gd->fenceValue, gd->fenceEvent));
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
        if (ub.src) ub.src->Release();
        ub.src = nullptr;
    }

    // record the compute work.
    {
        auto cmd = gd->commandList;

        // NOTE(handmade_gpu): this sort of API doesn't make sense to me.
        // if the pipeline already contains the signature, why must I bind both
        // here?
        cmd->SetPipelineState(gd->computePipelineState);  // contains the monolithic shader.
        cmd->SetComputeRootSignature(gd->rootSig);

        // bind heap and point the compute root sig to that heap.
        ID3D12DescriptorHeap *ppHeaps[] = {gd->descHeap};
        cmd->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);
        cmd->SetComputeRootDescriptorTable(0, gd->descHeap->GetGPUDescriptorHandleForHeapStart());

        cmd->ResourceBarrier(1,
            &CD3DX12_RESOURCE_BARRIER::Transition(
                gd->cpuTex, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS));

        cmd->Dispatch(ae::math::div_ceil(winInfo.width, 16),
            ae::math::div_ceil(winInfo.height, 16),
            1);  // using the pipeline.

        cmd->ResourceBarrier(1,
            &CD3DX12_RESOURCE_BARRIER::Transition(
                gd->cpuTex, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COMMON));

        hr = (cmd->Close());
        INIT_FAIL_CHECK();
    }

#undef INIT_FAIL_CHECK
}

#endif // #if AUTOMATA_ENGINE_DX12_BACKEND

dxr_world SCENE_TO_DXR_WORLD()
{
    dxr_world w = {};

    // TODO: right now we only support planes pointing up.
    // and because that, I am being lazy here and only init one plane info
    // in DXR world.
    w.planes[0].d = 0;
    // plane is pointing straight up!
    w.planes[0].n[1]     = 1;  // this gives something that is NOT 0x1, since it is a float.
    w.planes[0].matIndex = 1;

    for (int i = 0; i < g_world.sphereCount; i++) {
        w.spheres[i].r        = g_world.spheres[i].r;
        w.spheres[i].matIndex = g_world.spheres[i].matIndex;
    }

    for (int i = 0; i < g_world.materialCount; i++) {
        auto &mat              = g_world.materials[i];
        w.materials[i].scatter = mat.scatter;
        memcpy(&w.materials[i].refColor, &mat.refColor, sizeof(float) * 3);
        memcpy(&w.materials[i].emitColor, &mat.emitColor, sizeof(float) * 3);
    }

    // generate the cone for shooting rays from a pixel.
    for (int i = 0; i < g_raysPerPixel; i++) {
        w.rands[i].xy[0] = RandomBilateral();
        w.rands[i].xy[1] = RandomBilateral();
    }

    return w;
}
