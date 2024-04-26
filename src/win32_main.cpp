#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
#include <automata_engine.h>
#include <windows.h>
#include <gist/github/nc_stretchy_buffers.h>

#define CGLTF_IMPLEMENTATION
#include "external/cgltf.h"

#define NC_DS_IMPLEMENTATION
#include "nc_ds.h"

#define MAX_BOUNCE_COUNT 4
#define THREAD_COUNT 1
#define THREAD_GROUP_SIZE 32
#define RAYS_PER_PIXEL 4              // for antialiasing. 
#define RENDER_EQUATION_TAP_COUNT 8
#define MIN_HIT_DISTANCE float(1e-5)
#define WORLD_SIZE 5.0f
#define LEVELS 6

static HANDLE masterThreadHandle;

static v3 RayCast(world_t *world, v3 o, v3 d, int depth);
void visualizer(game_memory_t *gameMemory);
DWORD WINAPI render_thread(_In_ LPVOID lpParameter);
DWORD WINAPI master_thread(_In_ LPVOID lpParameter);
rtas_node_t GenerateAccelerationStructure(world_t *world);
void LoadGltf();

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

// helper function to make it a single line.
bool RayIntersectsWithAABB(v3 rayOrigin, v3 rayDirection, float minHitDistance, aabb_t box)
{
    bool exitedEarly;
    int faceHitIdx;
    float t=doesRayIntersectWithAABB2(rayOrigin, rayDirection, minHitDistance, box, &exitedEarly, &faceHitIdx);
    return t!=minHitDistance;
}

v3 brdf(material_t mat);

static v3 RayCast(world_t *world, v3 o, v3 d, int depth) {
    
    float tolerance = TOLERANCE, minHitDistance = MIN_HIT_DISTANCE;
    
    
    
    //v3 attenuation = V3(1, 1, 1);

#if 0
    int tapCount=1,tapIdxStart=0;
    // bounceCount == 4.
    // 4 iterations.
    // first iter = generates RENDER_EQUATION_TAP_COUNT rays.
    // next iter  = generates RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT rays.
    // third iter = generates RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT.
    // final iter = no arrays generated.
    constexpr int maxPossibleRays = RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT +
        (RENDER_EQUATION_TAP_COUNT)*RENDER_EQUATION_TAP_COUNT +
        RENDER_EQUATION_TAP_COUNT;

    static v3 origins[maxPossibleRays];
    static v3 directions[maxPossibleRays];
    static v3 radiances[maxPossibleRays];

    struct HitState /* HitState is the "structure tag" */ {
        int matIdx;
        float theta[RENDER_EQUATION_TAP_COUNT];
    };

    // the "origins" and "directions" store the outgoing rays from a particular surface point.
    // the "states" array stores the incoming rays at the same point where the outgoing rays emit from.
    static struct HitState states[maxPossibleRays]={};

    //static int isLive[maxPossibleRays]={};

    origins[0] = o;
    directions[0] = d;
    memset(states, 0xDEADBEEF, sizeof(states));
    //isLive[0]=1;
#else
    v3 radiance = {};
    if (depth>=MAX_BOUNCE_COUNT) return radiance;
#endif
    
    //for (int bounceCount = 0; bounceCount < MAX_BOUNCE_COUNT; ++bounceCount) {
        // NOTE: we use newTapCount idea because rays should only become "live" on the next bounce.
        //int newTapCount=tapCount;


        //for (int tapIdx=tapIdxStart; tapIdx < tapCount; tapIdx++) {
            v3 rayOrigin,rayDirection;

#if 0
            
            rayOrigin=origins[tapIdx];
            rayDirection=directions[tapIdx];
            //if (!isLive[tapIdx]) continue;
#else
            rayOrigin=o;
            rayDirection=d;
#endif

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
            // intersection test with the triangles in the world (via an acceleration structure).
            {
                rtas_node_t &rtas = world->rtas;

                static thread_local  stack_t<rtas_node_t*> nodes={};
                if (rtas.triangleCount && RayIntersectsWithAABB(rayOrigin, rayDirection, minHitDistance, rtas.bounds))
                    nc_spush(nodes, &rtas);

                mesh_t &mesh = world->meshes[0];

                while(nc_ssize(nodes))
                {
                    rtas_node_t *r = nc_spop(nodes);
                    if (r->children)
                        for (int i=0;i<nc_sbcount(r->children);i++)
                        {
                            rtas_node_t *c = &r->children[i];
                            if (c->triangleCount && RayIntersectsWithAABB(rayOrigin, rayDirection, minHitDistance, c->bounds))
                                nc_spush(nodes, c);
                        }
                    else // leaf
                        for (int i=0;i<r->triangleCount;i++)
                        {
                            int triIndex = r->triangles[i];
                            v3 *points = &mesh.points[triIndex*3];
                            v3 A=points[0];
                            v3 B=points[1];
                            v3 C=points[2];
                            v3 n = Normalize( Cross( B-A, C-A ) );
                            float t=RayIntersectTri(rayOrigin, rayDirection, minHitDistance, A,B,C, n);
                            // hit.
                            if ((t > minHitDistance) && (t < hitDistance)) {
                                hitDistance = t;
    #if 1
                                hitMatIndex = r->bounds.matIndex; // debug.
    #else
                            hitMatIndex = mesh.matIndex;
    #endif
                                nextNormal = n;
                            }
                        }
                }
            }
            // AABB intersection test
            for (
                unsigned int aabbIndex = 0;
                aabbIndex < world->aabbCount;
                aabbIndex++
            ) {
                aabb_t box = world->aabbs[aabbIndex];

                bool exitedEarly;
                int faceHitIdx;
                // NOTE: the faceNormals array was copied directly from within doesRayIntersectWithAABB2.
                // this is some garbage and not clean code. 
                constexpr v3 faceNormals[] = {
                    // front, back, left, right, top, bottom.
                    {0.f,0.f,-1.f}, {0.f,0.f,1.f}, {-1.f,0.f,0.f}, {1.f,0.f,0.f}, {0.f,1.f,0.f}, {0.f,-1.f,0.f}
                };
                float t=doesRayIntersectWithAABB2(rayOrigin, rayDirection, minHitDistance, box, &exitedEarly, &faceHitIdx);

                // check hit.
                if ((t > minHitDistance) && (t < hitDistance)) {
                    hitDistance = t;
                    hitMatIndex = box.matIndex;
                    nextNormal = faceNormals[faceHitIdx];
                }
            }
            material_t mat = world->materials[hitMatIndex];
            if (hitMatIndex) { 

                float ks,kd,theta,lastTheta,NdotL;

                //material_t mat = world->materials[hitMatIndex];

                // the line below accumulates the radiance, considering the outgoing ray from this point,
                // thus, we need the material and incident angle at where this ray will arrive!
                //radiance += cos(lastTheta) * Hadamard(incomingRadiance, brdf(lastMat));

                // whereas the accumulate due to emission is via material at point for this outgoing ray.
                //radiance += mat.emitColor;
                
                // NOTE: so this approach is similar to the discrete volume rendering integral.
                // and its like we are using the local illumination model of Phong.
                // so you get that product term of Ks (phong) or opacity(volume rendering).
                //
                // radiance = radiance + Hadamard(attenuation, mat.emitColor);
                // attenuation = Hadamard(attenuation, mat.refColor);
                
                rayOrigin = rayOrigin + hitDistance * rayDirection;
                v3 pureBounce = rayDirection - 2.0f * Dot(nextNormal, rayDirection) * nextNormal;

                // spawn the new rays.
                for (int i=0;i<RENDER_EQUATION_TAP_COUNT;i++)
                {
                    v3 randomBounce = Normalize(
                        nextNormal + V3(
                            RandomBilateral(),
                            RandomBilateral(),
                            RandomBilateral()
                        )
                    );
                    rayDirection = Normalize(Lerp(pureBounce, randomBounce, mat.roughness));
                    
#if 0
                    if (bounceCount<MAX_BOUNCE_COUNT-1) { 
                        origins[newTapCount] = rayOrigin;
                        directions[newTapCount] = rayDirection;
                        isLive[newTapCount]=true;
                        newTapCount++;
                        NdotL=theta = Dot(nextNormal,rayDirection);
                        states[tapIdx].theta[i] = theta;
                        //isLive[tapIdx]=false; // kill the parent ray.
                    } else {
                        // since the next ray will not accumulate, accumulation needs to happen here.
                    }
                } // end for
            }/*else { // skybox hit.
                material_t mat = world->materials[0];
                //radiance = radiance + Hadamard(attenuation, mat.emitColor);
                radiance += mat.emitColor;
                //isLive[tapIdx]=false;
            }*/
#else
                    NdotL=theta = Dot(nextNormal,rayDirection);

                    if (NdotL>0.f)
                    radiance += (1.f/float(RENDER_EQUATION_TAP_COUNT)) * 
                        cos(theta) * Hadamard(RayCast(world,rayOrigin,rayDirection,depth+1), brdf(mat));
                
                } // end for
                //radiance += mat.emitColor;
            } /*else {
                // skybox hit.
                
                //radiance = radiance + Hadamard(attenuation, mat.emitColor);
                
                //isLive[tapIdx]=false;
            } */// end if.
            radiance += mat.emitColor;
#endif
            
            //states[tapIdx].matIdx=hitMatIndex;
        //}
        //tapIdxStart=tapCount;
        //tapCount=newTapCount;
    //}

// premature optimization is the root of all evil?
#if 0
    // accumulate the radiance, going backwards through the data.
    // we reduce the tree-structured data as we go through.
    memset(radiances,0,sizeof(radiances));
    int stride=RENDER_EQUATION_TAP_COUNT*RENDER_EQUATION_TAP_COUNT;
    for (int bounceCount = 0; bounceCount < MAX_BOUNCE_COUNT-1; ++bounceCount) {
        for (int j=0;j<RENDER_EQUATION_TAP_COUNT;j++)
            {
                int matIndexOut,matIndexIn,idxOut,idxIn;
                float thetaIn;
                material_t matOut,matIn;

                idxOut=(j*stride)/RENDER_EQUATION_TAP_COUNT;
                idxIn=(j*stride);
                if (idxOut==idxIn) continue;//don't double count radiance.

                matIndexOut=states[idxOut];
                matIndexIn=states[idxIn].matIndex;
                if (matIndexIn==0xDEADBEEF) continue;//skip early terminated rays.
                
                matOut = world->materials[matIndexOut];
                matIn = world->materials[matIndexIn];

                radiances[idxIn] += matIn.emitColor;

                if (bounceCount==0) continue; // last level has no incoming rays.
                thetaIn = states[idxOut].theta[j];
                radiances[idxOut] += cos(thetaIn) * Hadamard(radiances[idxIn], brdf(matOut));            
            }
        stride/=RENDER_EQUATION_TAP_COUNT;
    }
#endif

    // divide by tapCount is OK here and agrees with rendering eq due to linearity of integrals.
    return radiance;
}

constexpr int octtreeDebugMaterialCount = (1<<LEVELS)*(1<<LEVELS)*(1<<LEVELS);

static world_t world = {};
// populate word with floor and spheres.
static material_t materials[5 + octtreeDebugMaterialCount ] = {};
static plane_t planes[1] = {};
static sphere_t spheres[3] = {};
static mesh_t meshes[1]={};
static aabb_t aabbs[1]={};
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
unsigned int raysPerPixel = RAYS_PER_PIXEL;
image_32_t image = {};

// TODO(Noah): Right now, the image is upside-down. Do we fix this on the application side
// or is this something that we can fix on the engine side?
void visualizer(game_memory_t *gameMemory) {
    memcpy((void *)gameMemory->backbufferPixels, image.pixelPointer,
        sizeof(uint32_t) * gameMemory->backbufferWidth * gameMemory->backbufferHeight);

    DWORD result = WaitForSingleObject( masterThreadHandle, 0);
    if (result == WAIT_OBJECT_0) {
//        setGlobalRunning
        automata_engine::setGlobalRunning(false);// platform::GLOBAL_RUNNING=false;
    }
    else {
        // the thread handle is not signaled - the thread is still alive
    }
    
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
                    color = color + contrib * RayCast(&world, rayOrigin, rayDirection,0);
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
    materials[1].roughness = 1.f;
    //materials[1].emitColor = V3(0.3f, 0.0f, 0.0f);
    materials[2].refColor = V3(0.7f, 0.25f, 0.3f);
    materials[2].roughness = 1.f;
    materials[3].refColor = V3(0.0f, 0.8f, 0.0f);
    materials[3].roughness = 0.0f;
    materials[4].refColor = V3(0.3f, 0.25f, 0.7f);
    materials[4].roughness = 1.f;
    { // generate debug materials for occtree voxels.
        int s=1<<LEVELS;
        for (int i=0;i<s;i++)
        for (int j=0;j<s;j++)
        for (int k=0;k<s;k++)
        {
            materials[5 + i * s * s + j * s + k].refColor = V3( s/(float)i,s/(float)j,s/(float)k );
            materials[5 + i * s * s + j * s + k].roughness=1.f;
        }
    }
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
    aabbs[0]=MakeAABB(v3 {}, v3 {1,1,1});
    aabbs[0].matIndex = 4;
    meshes[0].points=nullptr;
    meshes[0].matIndex = 4;
    // load gltf scene.
    LoadGltf();
    meshes[0].pointCount = nc_sbcount(meshes[0].points);
    world.materialCount = ARRAY_COUNT(materials);
    world.materials = materials;
    world.planeCount = ARRAY_COUNT(planes);
    world.planes = planes;
    world.sphereCount = ARRAY_COUNT(spheres);
    world.spheres = spheres;
    //world.aabbCount = ARRAY_COUNT(aabbs);
    world.aabbs = aabbs;
    world.meshCount = ARRAY_COUNT(meshes);
    world.meshes = meshes;
    world.rtas = GenerateAccelerationStructure(&world);
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
    masterThreadHandle=CreateThread(
        nullptr,
        0, // default stack size.
        master_thread,
        nullptr,
        0, // thread runs immediately after creation.
        nullptr
    );    
}

void AdoptChildren(rtas_node_t &node, rtas_node_t B);

rtas_node_t GenerateAccelerationStructure(world_t *world)
{
    rtas_node_t accel;

    float sep = WORLD_SIZE / float(1<<LEVELS);
    int leavesCount, nodesCount, halfLeavesCount;
    leavesCount = ceil(WORLD_SIZE / sep);
    assert(leavesCount%2==0);
    halfLeavesCount = leavesCount >> 1;
    nodesCount=leavesCount * leavesCount * leavesCount;

    // scratch space.
    static rtas_node_t *nodes=(rtas_node_t *)malloc(nodesCount*sizeof(rtas_node_t));

    auto ma=[&](int z,int y,int x) -> rtas_node_t &  {
        return nodes[z*leavesCount*leavesCount+y*leavesCount+x];
    };

    auto jackma=[=](int z,int y,int x) -> aabb_t {
        aabb_t box = MakeAABB(
            // NOTE: the beauty here is that since the clamping to integer boundary
            // went in an uneven way above, that means that here adding 0.5f indeed
            // gets us to center, regardless of if we are talking about an AABB on
            // either side.
            v3 {
                ( x-halfLeavesCount + .5f )*sep,
                ( y-halfLeavesCount + .5f )*sep,
                ( z-halfLeavesCount + .5f )*sep
            }, //origin.
            v3 {sep/2.f,sep/2.f,sep/2.f}    //halfdim.
        );
        return box;
    };

    // Triangle intersection test
    for (
        unsigned int meshIndex = 0;
        meshIndex < world->meshCount;
        meshIndex++
    ) {
        mesh_t &mesh = world->meshes[meshIndex];
        assert( mesh.pointCount % 3 == 0 );
        for (
            int triIndex = 0;
            triIndex*3 < mesh.pointCount;
            triIndex++
        ) {
            v3 *points = &mesh.points[triIndex*3];

            v3 vxMax,vxMin,vyMax,vyMin,vzMax,vzMin;
            int xMax,xMin,yMax,yMin,zMax,zMin;
            xMax=yMax=zMax=-leavesCount-1;
            xMin=yMin=zMin=leavesCount+1;

            for(int i=0;i<3;i++)
            {
                v3 A=points[i];
                // locate leaf.

                int x,y,z,index;
                //NOTE: we add halfleaf count since the positions may have been negative.
    #if 0
                x= int( A.x>0.f ? floor(A.x/sep) : ceil(A.x/sep) )+halfLeavesCount;
                y= int( A.y>0.f ? floor(A.y/sep) : ceil(A.y/sep) )+halfLeavesCount;
                z= int( A.z>0.f ? floor(A.z/sep) : ceil(A.z/sep) )+halfLeavesCount;
    #else
                // we have to take the floor always there will be a "singularity" at zero,
                // where we lose the information about what grid space we were in. 
                // plus, this means the largest negative number will be -halfLeavesCount.
                // -halfLeavesCount+halfLeavesCount will map to zero!
                x= int( floor(A.x/sep) )+halfLeavesCount;
                y= int( floor(A.y/sep) )+halfLeavesCount;
                z= int( floor(A.z/sep) )+halfLeavesCount;
    #endif

                assert(x>=0);
                assert(y>=0);
                assert(z>=0);

                if (x<xMin)xMin=x,vxMin=A;
                if (y<yMin)yMin=y,vyMin=A;
                if (z<zMin)zMin=z,vzMin=A;
                if (x>xMax)xMax=x,vxMax=A;
                if (y>yMax)yMax=y,vyMax=A;
                if (z>zMax)zMax=z,vzMax=A;                

                index=z*leavesCount*leavesCount + y * leavesCount + x;
                
                assert(index>=0&&index<nodesCount && 
                    "triangle is out of the world bounds!\n"
                    "either extend the world bounds or move the triangle.");

                /*
                the swiss cheese issue is where the voxels get so small that all of A B and C verts are not in it,
                but the triangle is still intersecting with the AABB.

                solution:                                      construct the triangle edges and intersect those with AABB.
                bigger brain solution:                         realize that the fundamental issue is the poor AABB. we don't actually want
                                                               there to be voxels so small not a single triangle vertex is within it.
                the universe cannot contain my brian solution: realize that the above is a pipe dream and won't happen
                                                               for an octree and generalized geometry.
                oh wait, I'm not smart!              solution: you can't just do the lines because triangles are solid things. in fact,
                                                               what we are doing here is the rasterization algorithm but in 3d.
                                                               
                                                               feels like: we can get the bounds (in voxel coords) of the 3d planar triangle
                                                               and do a plane collision with each AABB in volume given by the voxel extent.
                                                               --- I know this to not work, from just a single negative example I ideated. 

                                                               brute force: do the plane collision with bounds, for all collision points, check
                                                               if inside the triangle.

                                                               sloppy joe: y=mx+b (plane) with a width (use math to iter across indices).

                                                               a not so sloppy joe after all: take the above and relate it to precise raymarch.
                                                               when iter, "whichever is shorter". based on slope. the gradient of the plane is
                                                               thus what we care about, that's a v3.
                                                                   
                */
                //bool FiniteLineIntersectsWithAABB(v3 A, v3 B, float minHitDistance, aabb_t box);
                
#if 0
                int triangleCount=ma(z,y,x).triangleCount;
                bool goodToGo=true;
                for (int j=0;j<3 && (triangleCount - 1 - j>=0);j++)//check for duplicates.
                {
                    if (ma(z,y,x).triangles[triangleCount-1-j]==triIndex)
                    goodToGo=false;
                }
                if (goodToGo)
                {
                    nc_sbpush( ma(z,y,x).triangles,triIndex );
                    ma(z,y,x).triangleCount++;
                }
#endif
            }

            // iterate through the barycentric coordinates.
            v3 a,b;
            //=vxMax-vxMin;
            //v3 b=vyMax-vyMin;
            //float zxGrad,zyGrad;
            //zxGrad=a.z;
            //zyGrad=b.z;
            v3 A=points[0];
            v3 B=points[1];
            v3 C=points[2];
            //v3 n = Normalize( Cross( B-A, C-A ) );
            a=B-A;
            b=C-A;

            for (int z = zMin; z <= zMax; z++)
            for (int y = yMin; y <= yMax; y++)
            for (int x = xMin; x <= xMax; x++)
            {
#if 0
                // check for reject.
                float beta1,alpha1;
                alpha1=(x-xMin)/float(xMax-xMin);
                beta1=(y-yMin)/float(yMax-yMin);

                if (alpha1+beta1>=1.f) continue;
                
                // check Z reject.
                float alpha2,beta2;
                alpha2=(x-xMin+1)/float(xMax-xMin);
                beta2=(y-yMin+1)/float(yMax-yMin);
                float zBegin=vxMin.z+alpha1*zxGrad+beta1*zyGrad;
                float zEnd=vxMin.z+alpha2*zxGrad+beta2*zyGrad;
                if (  float(z)>zEnd || float(z)<zBegin ) continue;                
#endif

                nc_sbpush( ma(z,y,x).triangles,triIndex );
                ma(z,y,x).triangleCount++;
            }
        }
    }

    for (int z = 0; z < (leavesCount); z++)
    for (int y = 0; y < (leavesCount); y++)
    for (int x = 0; x < (leavesCount); x++)
    {
        aabb_t box = jackma(z,y,x);
        box.matIndex = z * leavesCount * leavesCount + y * leavesCount + x;
        ma(z,y,x).bounds=box;
    }

    // build up the tree.
    for (int i=0;i< LEVELS; i++)
    {
        // each time we go through, we shall reduce the total space.
        // this will simplify things and keep the cache locality better.
        for (int z = 0; z < (leavesCount >> i); z += 2)
        for (int y = 0; y < (leavesCount >> i); y += 2)
        for (int x = 0; x < (leavesCount >> i); x += 2)
        {
            rtas_node_t r={};

            // consider the 8 children.
            auto x0y0z0 = ma(z,y,x);
            auto x1y0z0 = ma(z,y, x + 1);
            auto x0y1z0 = ma(z, y + 1,x);
            auto x1y1z0 = ma(z, y + 1, x + 1);
            auto x0y0z1 = ma(z+1,y,x);
            auto x1y0z1 = ma(z+1, y, x + 1);
            auto x0y1z1 = ma(z+1, y + 1,x);
            auto x1y1z1 = ma(z+1, y + 1,x+1);

            int triangleCount = 
                x0y0z0.triangleCount+
                x1y0z0.triangleCount+
                x0y1z0.triangleCount+
                x1y1z0.triangleCount+
                x0y0z1.triangleCount+
                x1y0z1.triangleCount+
                x0y1z1.triangleCount+
                x1y1z1.triangleCount;

            if (triangleCount) {
                AdoptChildren(r,x0y0z0);
                AdoptChildren(r,x1y0z0);
                AdoptChildren(r,x0y1z0);
                AdoptChildren(r,x1y1z0);
                AdoptChildren(r,x0y0z1);
                AdoptChildren(r,x1y0z1);
                AdoptChildren(r,x0y1z1);
                AdoptChildren(r,x1y1z1);
                r.triangleCount = triangleCount;
            }

            r.bounds = AABBFromCube(x0y0z0.bounds.min, sep * float(1<<(i + 1)) );
            ma(x >> 1, y >> 1, z >> 1) = r;
        }
    }

    accel = ma(0, 0, 0);

    free(nodes);

    return accel;
}

void AdoptChildren(rtas_node_t &node, rtas_node_t B)
{
    nc_sbpush(node.children,B);
}

void LoadGltf()
{
        do {
        const char *gltfFilePath="res\\mario.glb";
        cgltf_options opt={};
        cgltf_data *data; // contains URIs for buffers and images.
        cgltf_result result = cgltf_parse_file(&opt, gltfFilePath, &data);

        if (result != cgltf_result_success)
            break;
        
        do {
            if (cgltf_validate(data) != cgltf_result_success)
                break;

            opt={};
            result=cgltf_load_buffers(&opt, data, gltfFilePath);

            if (result != cgltf_result_success)
                break;

            static stack_t<cgltf_node*> stack={};

            for (int i=0;i<data->scenes_count;i++)
            {
                cgltf_scene &scene=data->scenes[i];
                for (int j=0;j<scene.nodes_count;j++)
                {
                    cgltf_node *node=scene.nodes[j];
                    nc_spush(stack,node);
                }
            }

            // process the nodes.
            while( nc_ssize(stack) )
            {
                cgltf_node *node=nc_spop(stack);

                if (node->mesh)
                {
                    cgltf_mesh *mesh = node->mesh;
                    for (int i=0;i<mesh->primitives_count;i++)
                    {
                        cgltf_primitive prim=mesh->primitives[i];
                        if (prim.type==cgltf_primitive_type_triangles)
                        {
                            //cgltf_material mat = prim.material;
                            cgltf_accessor *indices = prim.indices;

                            cgltf_float *posData=nullptr;
                            int *indexData;
                            cgltf_size float_count;
                            for (int j=0;j<prim.attributes_count;j++)
                            {
                                cgltf_attribute attr = prim.attributes[j];
                                if (attr.type==cgltf_attribute_type_position)
                                {
                                    assert(posData==nullptr);

                                    cgltf_size floats_per_element = cgltf_num_components(attr.data->type);
                                	float_count = attr.data->count * floats_per_element;

                                    posData=(cgltf_float*)malloc(sizeof(cgltf_float)*float_count);
                                    cgltf_accessor_unpack_floats(attr.data, posData, float_count);
                                }

                            }
                            
                            if (indices) {

                                cgltf_size ints_per_element = cgltf_num_components(indices->type);
                                cgltf_size int_count = indices->count * ints_per_element;
                                indexData = (int*)malloc(sizeof(int) * int_count);
                                cgltf_accessor_unpack_indices(indices, indexData, sizeof(int), int_count);

                                // process the position and indices data to generate the triangles.
                                assert(int_count % 3 == 0);
                                for (int j = 0;j < int_count;j += 3)
                                {
                                    int a, b, c;
                                    a = indexData[j] * 3;
                                    b = indexData[j + 1] * 3;
                                    c = indexData[j + 2] * 3;

                                    v3 v1 = { posData[a] ,posData[a+1],posData[a+2] };
                                    v3 v2 = { posData[b],posData[b + 1],posData[b + 2] };
                                    v3 v3 = { posData[c],posData[c + 1],posData[c + 2]};

                                    nc_sbpush(meshes[0].points, v1);
                                    nc_sbpush(meshes[0].points, v2);
                                    nc_sbpush(meshes[0].points, v3);
                                }

                                free(indexData);
                            }
                            else {
                                for (int j = 0;j < float_count;j += 3)
                                {
                                    v3 v = { posData[j],posData[j+1],posData[j+2] };
                                    nc_sbpush(meshes[0].points, v);
                                }
                            }

                            free(posData);
                        }
                    }
                }

                for (int i=0;i<node->children_count;i++)
                {
                    cgltf_node *child=node->children[i];
                    nc_spush(stack,child);
                }
            }

        } while(0);

        cgltf_free(data);
    } while(0);
}

// bidirectional reflectance distribution function.
v3 brdf(material_t mat)
{

    return V3(1.f,1.f,1.f);
}