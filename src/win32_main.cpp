#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
#include <automata_engine.h>
#include <windows.h>
#include <gist/github/nc_stretchy_buffers.h>

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
    
3. Ultimate desired demo = get a monkey mesh in there (that is made of glass)
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
                float t = (plane.d - Dot(plane.n, rayOrigin)) / denom;
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = plane.matIndex;
                    nextNormal = plane.n;
                }
            } 
        }
        // Triangle intersection test
        for (
            unsigned int meshIndex = 0;
            meshIndex < world->meshCount;
            meshIndex++
        ) {
            mesh_t &mesh = world->meshes[meshIndex];
            assert( mesh.pointCount % 3 == 0 );
            for (
                unsigned int triIndex = 0;
                triIndex*3 < mesh.pointCount;
                triIndex++
            ) {
                v3 *points = &mesh.points[triIndex*3];
                v3 A=points[0];
                v3 B=points[1];
                v3 C=points[2];
                v3 n = Normalize( Cross( B-A, C-A ) );
                float t=RayIntersectTri(rayOrigin, rayDirection, minHitDistance, A,B,C, n);
                // hit.
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = mesh.matIndex;
                    nextNormal = n;
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
            material_t mat = world->materials[hitMatIndex];
            result = result + Hadamard(attenuation, mat.emitColor);
            break;
        }
    }
    return result;   
}

static world_t world = {};
// populate word with floor and spheres.
static material_t materials[5] = {};
static plane_t planes[1] = {};
static sphere_t spheres[3] = {};
static mesh_t meshes[1]={};
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
void visualizer(game_memory_t *gameMemory) {
    memcpy((void *)gameMemory->backbufferPixels, image.pixelPointer,
        sizeof(uint32_t) * gameMemory->backbufferWidth * gameMemory->backbufferHeight);
}

void automata_engine::HandleWindowResize(game_memory_t *gameMemory, int nw, int nh) { }
void automata_engine::PreInit(game_memory_t *gameMemory) {
    ae::defaultWinProfile = AUTOMATA_ENGINE_WINPROFILE_NORESIZE;
    ae::defaultWindowName = "Raytracer";
}
void automata_engine::Close(game_memory_t *gameMemory) { }

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

// TODO(Noah): There are certainly ways that we can make this Raytracer perform better overall!
// We can literally shift the entire thing over to execution on the GPU via compute.
// 
// Or we can say, "some threads finish all their texels, while others are still working". We are wasting
// potential good work! we need the master thread to notice and assign those lazy threads more work.
//
// Finally, on a per-thread basis, we could introduce SIMD intrinsics / instructions to see if we can get
// some more throughput there ...

DWORD WINAPI master_thread(_In_ LPVOID lpParameter) {
#define THREAD_COUNT 7
#define THREAD_GROUP_SIZE 32
#define PIXELS_PER_TEXEL (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)
    uint32_t maxTexelsPerThread = (uint32_t)ceilf((float)(image.width * image.height) / 
        (float)(PIXELS_PER_TEXEL * THREAD_COUNT));
#if 0
    PlatformLoggerLog("maxTexelsPerThread: %d", maxTexelsPerThread);
#endif
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
                    texel.height = max(texel.height, 0);
                    isPartialTexel = true;
                }
                if (xPos >= image.width) {
                    if (xPos > image.width) {
                        texel.width -= xPos - image.width;
                        texel.width = max(texel.width, 0);
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
#if 0
                    PlatformLoggerWarn("found invalid texel:");
                    PlatformLoggerLog("with x: %d", texel.xPos);
                    PlatformLoggerLog("with y: %d", texel.yPos);
#endif
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

void automata_engine::Init(game_memory_t *gameMemory) {
    printf("Doing stuff...\n");
    game_window_info_t winInfo = automata_engine::platform::getWindowInfo();
    image = AllocateImage(winInfo.width, winInfo.height);    
    materials[0].emitColor = V3(0.3f, 0.4f, 0.5f);
    materials[1].refColor = V3(0.5f, 0.5f, 0.5f);
    materials[2].refColor = V3(0.7f, 0.25f, 0.3f);
    materials[3].refColor = V3(0.0f, 0.8f, 0.0f);
    materials[3].scatter = 1.0f;
    materials[4].refColor = V3(0.3f, 0.25f, 0.7f);
    planes[0].n = V3(0,0,1);
    planes[0].d = 0; // plane on origin
    planes[0].matIndex = 1;
    spheres[0].p = V3(0,0,0);
    spheres[0].r = 1.0f;
    spheres[0].matIndex = 2;
    spheres[1].p = V3(-3,-2,0);
    spheres[1].r = 1.0f;
    spheres[1].matIndex = 4;
    spheres[2].p = V3(-2,0,2);
    spheres[2].r = 1.0f;
    spheres[2].matIndex = 3;
    meshes[0].pointCount=3;
    meshes[0].points=(v3*)malloc(sizeof(v3)*meshes[0].pointCount);
    meshes[0].points[0] = { -3.0,-1.0,0.5 };
    meshes[0].points[1] = { -4.0,-3.0,0.5 };
    meshes[0].points[2] = { -2.0,-3.0,0.5 };    
    meshes[0].matIndex = 3;
    world.materialCount = ARRAY_COUNT(materials);
    world.materials = materials;
    world.planeCount = ARRAY_COUNT(planes);
    world.planes = planes;
    world.sphereCount = ARRAY_COUNT(spheres);
    world.spheres = spheres;
    world.meshCount = ARRAY_COUNT(meshes);
    world.meshes  =meshes;
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
    // print infos.
    {
        PlatformLoggerLog("camera located at (%f,%f,%f)\n", cameraP.x,cameraP.y,cameraP.z);
        PlatformLoggerLog("cameraX: (%f,%f,%f)\n", cameraX.x,cameraX.y,cameraX.z);
        PlatformLoggerLog("cameraY: (%f,%f,%f)\n", cameraY.x,cameraY.y,cameraY.z);
        PlatformLoggerLog("cameraZ: (%f,%f,%f)\n", cameraZ.x,cameraZ.y,cameraZ.z);
        PlatformLoggerLog(
            "cameraX and Y define the plane where the image plane is embedded.\n"
            "rays are shot originating from the cameraP and through the image plane(i.e. each pixel).\n"
            "the camera has a local coordinate system which is different from the world coordinate system.\n");
    }
    automata_engine::bifrost::registerApp("raytracer_vis", visualizer);
    CreateThread(
        nullptr,
        0, // default stack size.
        master_thread,
        nullptr,
        0, // thread runs immediately after creation.
        nullptr
    );    
}

