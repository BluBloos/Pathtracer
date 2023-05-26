#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
#include <automata_engine.hpp>

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
        if (hitMatIndex) { // if there is any material at all.
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
unsigned int raysPerPixel = 256;
image_32_t image = {};

// TODO(Noah): Right now, the image is upside-down. Do we fix this on the application side
// or is this something that we can fix on the engine side?
void visualizer( ae::game_memory_t *gameMemory ) {
    memcpy((void *)gameMemory->backbufferPixels, image.pixelPointer,
        sizeof(uint32_t) * gameMemory->backbufferWidth * gameMemory->backbufferHeight);
}

void automata_engine::HandleWindowResize(game_memory_t *gameMemory, int nw, int nh) { }

void automata_engine::PreInit(game_memory_t *gameMemory) {
    ae::defaultWinProfile = AUTOMATA_ENGINE_WINPROFILE_NORESIZE;
    ae::defaultWindowName = "Raytracer";
    ae::defaultWidth = 1280;
    ae::defaultHeight = 720;
}


typedef struct texel {
    int width;
    int height;
    uint32_t xPos;
    uint32_t yPos;
} texel_t;

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
    std::tuple<texel_t *, uint32_t> *ptr_to_tuple = 
        (std::tuple<texel_t *, uint32_t> *)lpParameter;
    texel_t *texels = std::get<0>(*ptr_to_tuple);
    for (uint32_t i = 0; i < std::get<1>(*ptr_to_tuple); i++) {
        texel_t texel = texels[i];
        unsigned int *out = image.pixelPointer + texel.yPos * image.width + texel.xPos;
        // Raytracer works by averaging all colors from all rays shot from this pixel.
        for (unsigned int y = texel.yPos; y < (texel.height + texel.yPos); y++) {
            float filmY = -1.0f + 2.0f * (float)y / (float)image.height;
            for (unsigned int x = texel.xPos; x < (texel.width + texel.xPos); x++) {
                float filmX = -1.0f + 2.0f * (float)x / (float)image.width;
                v3 color = {};
                float contrib = 1.0f / (float)raysPerPixel;
                for (unsigned int rayIndex = 0; rayIndex < raysPerPixel; rayIndex++) {
                    float offX = filmX + (RandomBilateral() * halfPixW);
                    float offY = filmY + (RandomBilateral() * halfPixH);
                    v3 filmP = filmCenter + (offX * halfFilmW * cameraX) + (offY * halfFilmH * cameraY);
                    v3 rayOrigin = cameraP;
                    v3 rayDirection = Normalize(filmP - cameraP);
                    color = color + contrib * RayCast(&world, rayOrigin, rayDirection);
                }
                v4 BMPColor = {
                    255.0f * LinearToSRGB(color.r),
                    255.0f * LinearToSRGB(color.g),
                    255.0f * LinearToSRGB(color.b), 
                    255.0f
                }; 
                unsigned int BMPValue = BGRAPack4x8(BMPColor);
                *out++ = BMPValue; // ARGB
            }
            out += image.width - texel.width;
        }
        // TODO(Noah): It seems that the thread continues even after we close the window??
        // (we had prints that were showing) it could be the terminal doing a buffering thing,
        // and just being slow. OR, it could be that the threads are for reals still alive?? 
        // If it is the second one, this is a cause for concern.
    }
    ExitThread(0);
}

// TODO: once a thread completes work it should steal texels from other threads still working.
//
// on a per-thread basis, we could maybe introduce cpu SIMD.

DWORD WINAPI master_thread(_In_ LPVOID lpParameter) {
#define THREAD_COUNT 7
#define THREAD_GROUP_SIZE 32
#define PIXELS_PER_TEXEL (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)
    uint32_t maxTexelsPerThread = (uint32_t)ceilf((float)(image.width * image.height) / 
        (float)(PIXELS_PER_TEXEL * THREAD_COUNT));
    AELoggerLog("maxTexelsPerThread: %d", maxTexelsPerThread);
    {
        HANDLE threadHandles[THREAD_COUNT];
        uint32_t xPos = 0;
        uint32_t yPos = 0;
        // TODO(Noah): Could do entire image as BSP tree -> assign threads to these regions.
        // then break up these regions into texels.
        texel_t *texels = nullptr;
        std::tuple<texel_t *, uint32_t> texelParams[THREAD_COUNT];
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
                if (texel.xPos >= 0 && (texel.xPos + texel.width <= image.width) &&
                    texel.yPos >= 0 && (texel.yPos + texel.height <= image.height)
                ) {
                    if (isPartialTexel) j--; // NOTE(Noah): This is hack ...
                    StretchyBufferPush(texels, texel);
                } else {
                    AELoggerWarn("found invalid texel:");
                    AELoggerLog("with x: %d", texel.xPos);
                    AELoggerLog("with y: %d", texel.yPos);
                }
            }
        }
        // NOTE(Noah): The reason we split up the for-loop is because texels base addr
        // is not stable until we have finished pushing (this is due to stretchy buff logic).
        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            texelParams[i] = std::make_tuple(
                texels + i * maxTexelsPerThread,
                (i + 1 < THREAD_COUNT) ? maxTexelsPerThread :
                StretchyBufferCount(texels) - (THREAD_COUNT - 1) * maxTexelsPerThread
            );
            threadHandles[i] = CreateThread(
                nullptr,
                0, // default stack size.
                render_thread,
                (LPVOID)&texelParams[i],
                0, // thread runs immediately after creation.
                nullptr
            );
        }
        // wait for all threads to complete.
        for (uint32_t i = 0; i < THREAD_COUNT; i++) {
            WaitForSingleObject(threadHandles[i], INFINITE);
        }
        StretchyBufferFree(texels);
    }
    
    WriteImage(image, "test.bmp");    
    printf("Done. Image written to test.bmp\n");
    ExitThread(0);
}

struct game_data {
    ID3D12Device *d3dDevice = NULL;
    ID3D12Debug *debugController = NULL;

    // TODO: this is only temporary.
    ID3D12PipelineState *computePipelineState = NULL;
    ID3D12RootSignature *computeRootSig = NULL;
    ID3D12Resource *uavTexture = NULL;
    ID3D12DescriptorHeap *srvHeap = NULL;
    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptor;

    // things needed for command buffer submission.
    ID3D12CommandQueue *commandQueue = NULL;
    ID3D12CommandAllocator *commandAllocator = NULL;
    ID3D12GraphicsCommandList *commandList = NULL;
};

static game_data *getGameData(ae::game_memory_t *gameMemory) {
    return (game_data *)gameMemory->data;
}

void automata_engine::Init(game_memory_t *gameMemory) {
    printf("Doing stuff...\n");

    auto gd = getGameData(gameMemory);

    HRESULT hr;
#define INIT_FAIL_CHECK()                                                      \
  if (hr != S_OK) {                                                            \
    exit(hr);                                                                  \
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
            exit(-1);
        defer(COM_RELEASE(hardwareAdapter));

        (hr = D3D12CreateDevice(hardwareAdapter,
                                D3D_FEATURE_LEVEL_12_0, // minimum feature level
                                IID_PPV_ARGS(&gd->d3dDevice)));
        INIT_FAIL_CHECK();
    }

    // create compute pipeline state for postprocess
    {

#if !defined(_DEBUG)
        // Enable better shader debugging with the graphics debugging tools.
        UINT compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
        UINT compileFlags = 0;
#endif

        // set the compute root sig
        {

            // Determine supported root sig version.
            D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData = {};
            featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1;
            if (FAILED(gd->d3dDevice->CheckFeatureSupport(
                    D3D12_FEATURE_ROOT_SIGNATURE, &featureData,
                    sizeof(featureData)))) {
                featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;
            }

            CD3DX12_DESCRIPTOR_RANGE1 ranges[1];
            CD3DX12_ROOT_PARAMETER1 rootParameters[1];

            ranges[0].Init(
                D3D12_DESCRIPTOR_RANGE_TYPE_UAV,
                1, // num descriptors.
                0, // BaseShaderRegister: map to register(t0) in HLSL.
                0, // RegisterSpace: map to register(t0, space0) in HLSL.
                D3D12_DESCRIPTOR_RANGE_FLAG_DATA_VOLATILE // descriptor static,
                                                          // data pointed to is
                                                          // not.
            );

            rootParameters[0].InitAsDescriptorTable(
                1, &ranges[0], D3D12_SHADER_VISIBILITY_ALL);

            CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
            rootSignatureDesc.Init_1_1(_countof(rootParameters),
                                       rootParameters);

            // TODO: could make a func called "serialize root sig".
            ID3DBlob *signature;
            ID3DBlob *error;
            defer(COM_RELEASE(signature));
            defer(COM_RELEASE(error));

            if ((hr = D3DX12SerializeVersionedRootSignature(
                     &rootSignatureDesc, featureData.HighestVersion, &signature,
                     &error)) == S_OK) {
                (hr = gd->d3dDevice->CreateRootSignature(
                     0, signature->GetBufferPointer(),
                     signature->GetBufferSize(),
                     IID_PPV_ARGS(&gd->computeRootSig)));
            }
            INIT_FAIL_CHECK();
        }
        // end compute root sig create.

        // note that the blob is only temporary since it gets compiled into the
        // pipeline obj.
        ID3DBlob *computeShader = nullptr;

        const wchar_t *shaderFilePath = L"res\\shader.hlsl";
        hr = ae::DX::compileShader(shaderFilePath, "main", "cs_5_0",
                                   compileFlags, &computeShader);
        INIT_FAIL_CHECK();

        D3D12_COMPUTE_PIPELINE_STATE_DESC computePipelineDesc = {};
        computePipelineDesc.pRootSignature = gd->computeRootSig;
        computePipelineDesc.CS = CD3DX12_SHADER_BYTECODE(computeShader);

        hr = (gd->d3dDevice->CreateComputePipelineState(
            &computePipelineDesc, IID_PPV_ARGS(&gd->computePipelineState)));
        INIT_FAIL_CHECK();
    }

    game_window_info_t winInfo = automata_engine::platform::getWindowInfo();

    // create the texture for use as an UAV.
    // NOTE: we are actually creating a buffer, but we conceptualize this as a
    // texture.

    {
        // call below creates both the resource and the heap.
        (hr = gd->d3dDevice->CreateCommittedResource(
             &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
             D3D12_HEAP_FLAG_NONE,
             &CD3DX12_RESOURCE_DESC::Tex2D(
                 DXGI_FORMAT_R8G8B8A8_UNORM, winInfo.width, winInfo.height, 1,
                 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS),
             D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr,
             IID_PPV_ARGS(&gd->uavTexture)));
        INIT_FAIL_CHECK();
    }

    // create the UAV descriptor heap + descriptor.
    {
        D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
        srvHeapDesc.NumDescriptors = 1;
        srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        (hr = gd->d3dDevice->CreateDescriptorHeap(&srvHeapDesc,
                                                  IID_PPV_ARGS(&gd->srvHeap)));
        INIT_FAIL_CHECK();

        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
        uavDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;

        CD3DX12_CPU_DESCRIPTOR_HANDLE uavHandle(
            gd->srvHeap->GetCPUDescriptorHandleForHeapStart());
        gd->d3dDevice->CreateUnorderedAccessView(
            gd->uavTexture,
            nullptr, // no counter.
            &uavDesc,
            uavHandle // where to write the descriptor.
        );
    }

    // create what we need so that we can submit commands to the GPU.
    {
        // queue.
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        (hr = gd->d3dDevice->CreateCommandQueue(
             &queueDesc, IID_PPV_ARGS(&gd->commandQueue)));
        INIT_FAIL_CHECK();

        // command allocator.
        (hr = gd->d3dDevice->CreateCommandAllocator(
             D3D12_COMMAND_LIST_TYPE_DIRECT,
             IID_PPV_ARGS(&gd->commandAllocator)));
        INIT_FAIL_CHECK();

        // command list.
        (hr = gd->d3dDevice->CreateCommandList(
             0, // nodemask, this is for single GPU scenarios.
             D3D12_COMMAND_LIST_TYPE_DIRECT,
             gd->commandAllocator, // how the device allocates commands for this
                                   // list.
             nullptr, // initial pipeline state.
             IID_PPV_ARGS(&gd->commandList)));
        INIT_FAIL_CHECK();
    }

    // record the compute work.
    {
        auto cmd = gd->commandList;
        cmd->SetPipelineState(gd->computePipelineState);
        cmd->SetComputeRootSignature(gd->computeRootSig);
        cmd->SetComputeRootDescriptorTable(
            0, gd->srvHeap->GetGPUDescriptorHandleForHeapStart());
        cmd->Dispatch(ae::math::div_ceil(winInfo.width, 16),
                      ae::math::div_ceil(winInfo.height, 16), 1);
        hr = (cmd->Close());
        INIT_FAIL_CHECK();
    }

#undef INIT_FAIL_CHECK

    image = AllocateImage(winInfo.width, winInfo.height);
    materials[0].emitColor = V3(0.3f, 0.4f, 0.5f);
    materials[1].refColor = V3(0.5f, 0.5f, 0.5f);
    materials[2].refColor = V3(0.7f, 0.25f, 0.3f);
    materials[3].refColor = V3(0.0f, 0.8f, 0.0f);
    materials[3].scatter = 1.0f;
    planes[0].n = V3(0,0,1);
    planes[0].d = 0; // plane on origin
    planes[0].matIndex = 1;
    spheres[0].p = V3(0,0,0);
    spheres[0].r = 1.0f;
    spheres[0].matIndex = 2;
    spheres[1].p = V3(-3,-2,0);
    spheres[1].r = 1.0f;
    spheres[1].matIndex = 2;
    spheres[2].p = V3(-2,0,2);
    spheres[2].r = 1.0f;
    spheres[2].matIndex = 3;
    world.materialCount = ARRAY_COUNT(materials);
    world.materials = materials;
    world.planeCount = ARRAY_COUNT(planes);
    world.planes = planes;
    world.sphereCount = ARRAY_COUNT(spheres);
    world.spheres = spheres;
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
                 master_thread, nullptr,
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
    COM_RELEASE(gd->computeRootSig);

    COM_RELEASE(gd->srvHeap);
    COM_RELEASE(gd->uavTexture);

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
