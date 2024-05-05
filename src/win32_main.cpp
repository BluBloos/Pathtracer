#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
#include <automata_engine.h>
#include <windows.h>
#include <gist/github/nc_stretchy_buffers.h>

#define CGLTF_IMPLEMENTATION
#include "external/cgltf.h"

//#define STB_IMAGE_IMPLEMENTATION, already defined in the engine translation unit.
#include "stb_image.h"

#define NC_DS_IMPLEMENTATION
#include "nc_ds.h"

#define MAX_BOUNCE_COUNT 4
#define MAX_THREAD_COUNT 16
#define THREAD_GROUP_SIZE 32
#define RAYS_PER_PIXEL_MAX 512              // for antialiasing.
#define RENDER_EQUATION_MAX_TAP_COUNT 16
#define MIN_HIT_DISTANCE float(1e-5)
#define WORLD_SIZE 5.0f
#define LEVELS 6
#define IMAGE_FOCAL_LENGTH 5.f
#define N_AIR 1.003f
#define STATIC_MATERIAL_COUNT 5
#define DYNAMIC_MATERIAL_MAX_COUNT 10
// here, we use a 35mm=3.5cm camera lens disc.
#define LENS_RADIUS 0.035f // scene units are 1 = 1m.
#define FOCAL_LENGTH 0.098f

static v3 TonemapPass(v3 pixel);
static v3 RayCast(world_t *world, v3 o, v3 d, int depth);
static ray_payload_t RayCastIntersect(world_t *world, const v3 &rayOrigin, const v3 &rayDirection);
void visualizer(game_memory_t *gameMemory);
DWORD WINAPI render_thread(_In_ LPVOID lpParameter);
DWORD WINAPI master_thread(_In_ LPVOID lpParameter);
rtas_node_t GenerateAccelerationStructure(world_t *world);
void LoadGltf();
void LoadBespokeTextures();
bool FindRefractionDirection( const v3 &rayDir, v3 N, float nglass, v3 &refractionDir );
v3 brdf_diff(material_t &mat,v3 surfPt);
v3 brdf_specular(material_t &mat, v3 surfPoint, v3 normal, v3 L, v3 V, v3 H);
v3 SampleTexture(texture_t tex, v2 uv);
v3 BespokeSampleTexture(texture_t tex, v2 uv);

float Schlick(float F0, float cosTheta);
v3 SchlickMetal(float F0, float cosTheta, float metalness, v3 surfaceColor);
float MaskingShadowing(v3 normal, v3 L, v3 V, v3 H, float roughness);
float GGX(v3 N, v3 H, float roughness);

constexpr bool use_pinhole=true;
static world_t g_world = {};
static light_t g_lights[2]={};
constexpr int octtreeDebugMaterialCount = (1<<LEVELS)*(1<<LEVELS)*(1<<LEVELS);
static int g_dynamicMaterialCount;
static material_t g_materials[STATIC_MATERIAL_COUNT + octtreeDebugMaterialCount + DYNAMIC_MATERIAL_MAX_COUNT] = {};
static plane_t g_planes[1] = {};
static sphere_t g_spheres[3] = {};
static mesh_t g_meshes[1]={};
static aabb_t g_aabbs[1]={};
static texture_t g_textures[4]={};

void AddDynamicMaterial(material_t mat)
{
    assert(g_dynamicMaterialCount<DYNAMIC_MATERIAL_MAX_COUNT);
    g_materials[STATIC_MATERIAL_COUNT+octtreeDebugMaterialCount+g_dynamicMaterialCount++]=mat;
}

int MaterialsCount()
{
    return STATIC_MATERIAL_COUNT+octtreeDebugMaterialCount+g_dynamicMaterialCount;
}

static v3 g_cameraP = {};
static v3 g_cameraZ = {};
static v3 g_cameraX = {};
static v3 g_cameraY = {};
static float g_filmDist = 0.1f;//10cm.
static float g_filmW = 0.1f;
static float g_filmH = 0.1f;
static float g_halfFilmW;
static float g_halfFilmH;
static v3 g_filmCenter;
float g_halfPixW;
float g_halfPixH;
image_32_t g_image = {};

static HANDLE g_masterThreadHandle;
static int g_tc; // thread count.
static int g_sc; // sample count (render equation tap count).
static int g_pp; // rays per pixel.
// flags for enablement.
static bool g_bNormals;
static bool g_bMetalness;
static bool g_bRoughness;

// https://learn.microsoft.com/en-us/cpp/c-runtime-library/argc-argv-wargv?view=msvc-170&redirectedfrom=MSDN
extern int __argc;
extern char ** __argv;

/*
TODO:
APPLICATION:
X render proper orientation at runtime.
/ save proper orientation.
    - make it granular where we can flip either X or Y, or both.
/ Add cmdline processing.
    X thread count.
    - output image filepath; dynamically find extension and output based on that.
    - allow for disable DOF.
- usage/help print if supply -h.
- use stb_image_write to support .PNG and .JPG.
RENDERING FEATURES:
/ tonemapping from HDR to 0->1 range.
    - do more than a cursory read of https://64.github.io/tonemapping/ and integrate real camera response.
/ add light sources: directional, point, area.
    X directional.
    X point.
    - to do area lights, e.g. a triangle light, we will need to uniformly sample the light.
    the approach here is rejection sampling. for N samples, require 2*N (but this will vary
    depending on the geometry); it's related to the area ratios.
    X add shadow rays.
- "god rays" and fog, both via volumetric light transport.
/ add GLTF loading.
    X load triangles.
    X load materials with constant color.
    - load materials with textures.
X add rendering of triangle geometry.
/ accelerate triangle geometry rendering via occtree.
/ approximate the rendering equation.
    - uniform sampling in hemisphere.
    / proper implementation of a BRDF model (GGX).
    - specular antialiasing:
        - despite shaky theory, can use Toksvig equation.
/ refraction
    - different wavelengths refract differently.
/ textures for PBR materials.
    X diffuse.
    - bump map.
    - roughness.
    - metalness.
    - mipmapping.
X diffuse and specular interaction with surfaces via fresnel equations.
- subsurface scattering.
/ physical camera modelling (e.g. lens).
    X depth of field
    - exposure
- antialiasing:
    - use statified sampling.
- accelerate and improve quality with denoising.
- importance sampling.
- add early ray termintation via russian roulette.
PERFORMANCE:
- Use compute shaders.
- "Some threads finish all their texels, while others are still working" - we are wasting potential good work!
   We need the master thread to notice and assign those lazy threads more work.
- SIMD?
*/

/*
FUTURE WORK:
- add support for anisotropic BRDFs (would allow for rendering e.g. brushed metal).
- support very rough metals via multiple bounce surface reflection in the BRDF; currently such surfaces (and others?)
  rendered via current system are too dark.
- add BRDF models for cloth.
- add support for materials with nanogeometry , where geometric optics model breaks down (e.g. thin films).
- there's a whole can of worms when it comes to specular antialiasing. Open it!
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

static void WriteImage(image_32_t image, char *fileName, bool bFlip=false) {
    void *pixels;
    unsigned int outputPixelSize = GetTotalPixelSize(image);
    pixels = malloc(outputPixelSize);
    if (pixels==NULL) { fprintf(stderr, "[ERROR] Unable to write output file,malloc error.\n");return; }
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
        if (bFlip) {
            for ( unsigned int *pi=image.pixelPointer+image.width*image.height-1, *pp=(unsigned int*)pixels;pi>=image.pixelPointer; )
                *pp++=*pi--;
            fwrite(pixels, outputPixelSize, 1, fileHandle);
        } else fwrite(image.pixelPointer, outputPixelSize, 1, fileHandle);
        fclose(fileHandle);
    }
    else {
        fprintf(stderr, "[ERROR] Unable to write output file,fopen error\n");
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

static ray_payload_t RayCastIntersect(world_t *world, const v3 &rayOrigin, const v3 &rayDirection) {
    float tolerance = TOLERANCE, minHitDistance = MIN_HIT_DISTANCE;
    ray_payload_t p={};
    p.hitDistance=FLT_MAX;
    p.hitMatIndex=0;
    float &hitDistance = p.hitDistance;
    unsigned int &hitMatIndex = p.hitMatIndex;
    v3 &nextNormal = p.normal;

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

    // floor intersection test.
    for (
        unsigned int planeIndex = 0;
        planeIndex < world->planeCount;
        planeIndex++
    ) {
        plane_t plane = world->planes[planeIndex];
        float t=RayIntersectPlane(rayOrigin, rayDirection, plane.n, plane.d, minHitDistance);
        if ((t > minHitDistance) && (t < hitDistance)) {
            hitDistance = t;
            hitMatIndex = plane.matIndex;
            nextNormal = plane.n;
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
#if 0
                        hitMatIndex = r->bounds.matIndex; // debug.
#else
                    hitMatIndex = mesh.matIndices[triIndex*3];
#endif
                        nextNormal = n;
                    }
                }
        }
    }
            
    // AABB intersection test.
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

    return p;
}

static v3 RayCast(world_t *world, v3 o, v3 d, int depth)
{
    float tolerance = TOLERANCE, minHitDistance = MIN_HIT_DISTANCE;
    v3 rayOrigin, rayDirection;

    v3 radiance = {};
    if (depth>=MAX_BOUNCE_COUNT) return radiance;

    rayOrigin=o;
    rayDirection=d;

    ray_payload_t p;
    float &hitDistance = p.hitDistance;
    unsigned int &hitMatIndex = p.hitMatIndex;
    v3 &nextNormal = p.normal;
    p=RayCastIntersect(world, rayOrigin, rayDirection);
      
    material_t mat = world->materials[hitMatIndex];
    do {
    if (hitMatIndex) {

        float cosTheta,NdotL,NdotV,F0,metalness;
        v3 halfVector,N,L,V,pureBounce,brdfTerm,ks_local,kd_local,r3;

        cosTheta=(Dot(nextNormal, rayDirection));
        cosTheta=(cosTheta>0.f)?Dot(-1.f*nextNormal, rayDirection):cosTheta;

        F0 = Square((N_AIR-mat.ior)/(N_AIR+mat.ior)); // NOTE: need to change when support refraction again.

        int tapCount = g_sc;
        float tapContrib = 1.f / float(tapCount + world->lightCount);

        rayOrigin = rayOrigin + hitDistance * rayDirection;
        pureBounce = rayDirection - 2.0f * cosTheta * nextNormal;

        N=Dot(nextNormal,rayDirection)<0.f ? nextNormal:-1.f*nextNormal;
        V=-rayDirection;

        // Via the material mapping (might be UVs or some other function), sample the texture.
        v2 uv = {rayOrigin.x,rayOrigin.y};//for now.

        { // find the metalness.
            if (mat.metalnessIdx==0 || !g_bMetalness) {
                metalness=mat.metalness;
            } else {
                texture_t tex=g_textures[mat.metalnessIdx-1];
                r3 = BespokeSampleTexture(tex, uv );
                metalness=r3.x;
            }
        }

        // find the normal.
        if (g_bNormals && mat.normalIdx!=0){
            texture_t tex=g_textures[mat.normalIdx-1];
            N = BespokeSampleTexture(tex, uv );
            // NOTE: this currently only works for the ground plane, since it's normal happens to be up!
            N = Normalize(2.f*N - V3(1.f,1.f,1.f));
        }

        if (Dot(N, V)<=0.f) break;

        // cast the shadow ray(s).
        for (unsigned int i=0;i<world->lightCount;i++) {
            light_t &light=world->lights[i];
            float hitThreshold=FLT_MAX,attenuation=1.f;
            switch(light.kind){
                case LIGHT_KIND_DIRECTIONAL:
                {
                    L=-1.f*light.direction;
                } break;
                case LIGHT_KIND_POINT:
                {
                    L=light.position-rayOrigin;
                    hitThreshold=Magnitude(L);
                    L=(1.f/hitThreshold)*L;//normalize.
                    attenuation=powf(2.7f,-0.1f*hitThreshold);
                } break;
                case LIGHT_KIND_TRIANGLE:
                break;
            }

            halfVector =(1.f/Magnitude(L+V)) * (L+V);
            cosTheta=Dot(halfVector,L);

            if ((NdotL=Dot(N, L))>0.f && attenuation>0.f) {
                assert(cosTheta>=0.f);
                ks_local = SchlickMetal(F0,cosTheta,metalness,mat.metalColor);
                kd_local=V3(1.f,1.f,1.f)-ks_local;
                for (int i = 0;i < 3;i++) assert(ks_local.E[i] >= 0.f && ks_local.E[i] <= 1.f);
                kd_local = Lerp(kd_local, V3(0,0,0), metalness); // metal surfaces have a very high absorption!
                brdfTerm = Hadamard(kd_local, brdf_diff(mat,rayOrigin))+
                    Hadamard(ks_local, brdf_specular(mat,rayOrigin, N, L, V, halfVector ));

                auto payload=RayCastIntersect(world,rayOrigin,L);
                if (payload.hitMatIndex==0 || (payload.hitDistance>hitThreshold&&payload.hitDistance>minHitDistance) ) {
                    //radiance += tapContrib * NdotL * Hadamard( attenuation*light.radiance, brdfTerm );
                    radiance += NdotL * Hadamard( attenuation*light.radiance, brdfTerm );//correct coeff for importance sampling, i.e. inclusion of 1/p(x) term.
                }
            }
        }

        // spawn the new rays.
        for (int i=0;i<tapCount;i++)
        {
            v3 randomBounce = Normalize(
                (N) + V3(
                    RandomBilateral(),
                    RandomBilateral(),
                    RandomBilateral()
                )
            );
            //L = Normalize(Lerp(pureBounce, randomBounce, 0.f));
            L = pureBounce;

            halfVector =(1.f/Magnitude(L+V)) * (L+V);
            cosTheta=Dot(halfVector,L);

            if ((NdotL=Dot(N, L))>0.f)//incoming light is in hemisphere.
            {
                assert(cosTheta>0.f);

                ks_local = SchlickMetal(F0,cosTheta,metalness,mat.metalColor);
                kd_local=V3(1.f,1.f,1.f)-ks_local;
                for(int j=0;j<3;j++) assert(ks_local.E[j] >= 0.f && ks_local.E[j] <= 1.f);
                kd_local = Lerp(kd_local, V3(0,0,0), metalness); // metal surfaces have a very high absorption!
                brdfTerm = Hadamard(kd_local, brdf_diff(mat,rayOrigin))+
                    Hadamard(ks_local, brdf_specular(mat,rayOrigin, N, L, V, halfVector ));

                radiance += tapContrib * NdotL * Hadamard(RayCast(world,rayOrigin,L,depth+1), brdfTerm);
            }
        } // END FOR.

        // How to think about the refraction ray?
        // it doesn't really have anything to do with the BRDF.
        // it's simply where we continue along the same ray path,
        // but there's a bend in the path along the way (due to the interface).
        // and we treat the geometry as "participating media", where
        // there is some absorption when transporting through a translucent
        // material.
        
        // coming up through a transparent surface
        // with the reflection ray calculated as 'Iout' above.  The blend
        // should be 'opacity' of the reflected ray and '1-opacity' of the
        // refracted ray.

        // disable refraction for now.
#if 0
        if (mat.alpha < 1.0f) { // not completely opaque

            v3 refractionDirection;
            if (FindRefractionDirection(rayDirection, nextNormal, mat.ior, refractionDirection))
                radiance += kd*(1.f-mat.alpha) * RayCast(world, rayOrigin, refractionDirection, depth);
        }
#endif

    } // END IF.
    } while(false);

    radiance += mat.emitColor;

    return radiance;
}

// TODO(Noah): Right now, the image is upside-down. Do we fix this on the application side
// or is this something that we can fix on the engine side?
void visualizer(game_memory_t *gameMemory) {
    memcpy((void *)gameMemory->backbufferPixels, g_image.pixelPointer,
        sizeof(uint32_t) * gameMemory->backbufferWidth * gameMemory->backbufferHeight);

    DWORD result = WaitForSingleObject( g_masterThreadHandle, 0);
    if (result == WAIT_OBJECT_0) {
//        setGlobalRunning
        automata_engine::setGlobalRunning(false);// platform::GLOBAL_RUNNING=false;
    }
    else {
        // the thread handle is not signaled - the thread is still alive
    }
}

void automata_engine::HandleWindowResize(game_memory_t *gameMemory, int nw, int nh) { 
    // nothing to see here, folks.
}

void automata_engine::Close(game_memory_t *gameMemory) { 
    // nothing to see here, folks.
}

void automata_engine::PreInit(game_memory_t *gameMemory) {
    ae::defaultWinProfile = AUTOMATA_ENGINE_WINPROFILE_NORESIZE;
    ae::defaultWindowName = "Raytracer";

    g_tc=MAX_THREAD_COUNT;
    g_sc=8;
    g_pp=4;

    g_bNormals=true;
    g_bMetalness=true;
    g_bRoughness=true;

    // process cmdline args.
    for( char **argv=__argv; *argv; argv++ )
        if ( *argv[0]=='-' )
            for( char c; c=*((argv[0])++); ) {
                switch( c ) {
                    case 't':
                        g_tc=max(0,min(MAX_THREAD_COUNT,atoi(argv[0])));
                        break;
                    case 's':
                        g_sc=max(0,min(RENDER_EQUATION_MAX_TAP_COUNT,atoi(argv[0])));
                        break;
                    case 'p':
                        g_pp=max(0,min(RAYS_PER_PIXEL_MAX,atoi(argv[0])));
                        break;
                    case 'n':
                        g_bNormals=false;
                        break;
                    case 'm':
                        g_bMetalness=false;
                        break;
                    case 'r':
                        g_bRoughness=false;
                        break;
                    default:
                        // nothing to see here, folks.
                        break;
                }
            }
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
    std::tuple<texel_t *, uint32_t> *ptr_to_tuple = 
        (std::tuple<texel_t *, uint32_t> *)lpParameter;
    texel_t *texels = std::get<0>(*ptr_to_tuple);
    for (uint32_t i = 0; i < std::get<1>(*ptr_to_tuple); i++) {
        texel_t texel = texels[i];
        unsigned int *out = g_image.pixelPointer + texel.yPos * g_image.width + texel.xPos;
        // Raytracer works by averaging all colors from all rays shot from this pixel.
        for (unsigned int y = texel.yPos; y < (texel.height + texel.yPos); y++) {
            float filmY = -1.0f + 2.0f * (float)y / (float)g_image.height;
            for (unsigned int x = texel.xPos; x < (texel.width + texel.xPos); x++) {
                float filmX = -1.0f + 2.0f * (float)x / (float)g_image.width;
                v3 color = {};

                if ( use_pinhole ) {
                    float contrib = 1.0f / (float)g_pp;
                    for (unsigned int rayIndex = 0; rayIndex < g_pp; rayIndex++) {
                        float offX = filmX + (RandomBilateral() * g_halfPixW);
                        float offY = filmY + (RandomBilateral() * g_halfPixH);
                        v3 filmP = g_filmCenter + (offX * g_halfFilmW * g_cameraX) + (offY * g_halfFilmH * g_cameraY);
                        v3 rayOrigin = g_cameraP;
                        v3 rayDirection = Normalize(filmP - g_cameraP);
                        color = color + contrib * RayCast(&g_world, rayOrigin, rayDirection,0);
                    }
                } else // if not the pinhole model, we use a more physical camera model with a real aperature and lens.
                {
                    float contrib = 1.0f / (float)g_pp/(float)g_pp;

                    // the poisson disk samples
                    const int NUM_SAMPLES = 12;     // the number of samples to take.
                    //const float PI = 3.14159265359f; // the value of PI.
                    static const v2 poissonDisk[NUM_SAMPLES] = {
                        V2(0.0, 0.0),
                        V2(-0.94201624, -0.39906216),
                        V2(0.94558609, -0.76890725),
                        V2(-0.094184101, -0.92938870),
                        V2(0.34495938, 0.29387760),
                        V2(-0.91588581, 0.45771432),
                        V2(-0.81544232, -0.87912464),
                        V2(-0.38277543, 0.27676845),
                        V2(0.97484398, 0.75648379),
                        V2(0.44323325, -0.97511554),
                        V2(0.53742981, -0.47373420),
                        V2(-0.26496911, -0.41893023)
                    };

                    for (unsigned int rayIndex = 0; rayIndex < g_pp; rayIndex++) { // integrate across image sensor.
                        
                        float offX = filmX + (RandomBilateral() * g_halfPixW);
                        float offY = filmY + (RandomBilateral() * g_halfPixH);
                        v3 filmP = g_filmCenter + (offX * g_halfFilmW * g_cameraX) + (offY * g_halfFilmH * g_cameraY);
                        v3 rayOrigin = g_cameraP;
                        v3 rayDirection = Normalize(g_cameraP-filmP);

                        // intersect with focal plane.
                        // https://computergraphics.stackexchange.com/questions/246/how-to-build-a-decent-lens-camera-objective-model-for-path-tracing
                        // 1/f = 1/v + 1/b.
                        float focalPlaneDist = 1.f/(1.f/FOCAL_LENGTH - 1.f/g_filmDist);
                        v3 planePoint,N= g_cameraZ,focalPoint;
                        planePoint = g_cameraP + g_cameraX + focalPlaneDist*N;
                        float d = Dot(N,planePoint);

                        float t=RayIntersectPlane(rayOrigin, rayDirection, N, d, MIN_HIT_DISTANCE);
                        assert(t!=MIN_HIT_DISTANCE);

                        focalPoint=rayOrigin+t*rayDirection;

                        // sample poisson disk point.
                        for (unsigned int rayIndex2 = 0; rayIndex2 < g_pp; rayIndex2++) {

                            v2 diskSample=poissonDisk[(rayIndex2*rayIndex)%NUM_SAMPLES];
                            v3 rayOriginDisk,rayDirectionDisk;
                            rayOriginDisk = rayOrigin + diskSample.x*LENS_RADIUS*g_cameraX + diskSample.y*LENS_RADIUS*g_cameraY;
                            rayDirectionDisk=Normalize(focalPoint-rayOriginDisk);

                            color = color + contrib * RayCast(&g_world, rayOriginDisk, rayDirectionDisk,0);
                        }
                    }
                }

                color=TonemapPass(color);

                v4 BMPColor = {
                    255.0f * LinearToSRGB(color.r),
                    255.0f * LinearToSRGB(color.g),
                    255.0f * LinearToSRGB(color.b), 
                    255.0f
                }; 
                unsigned int BMPValue = BGRAPack4x8(BMPColor);
                *out++ = BMPValue; // ARGB
            }
            out += g_image.width - texel.width;
        }
        // TODO(Noah): It seems that the thread continues even after we close the window??
        // (we had prints that were showing) it could be the terminal doing a buffering thing,
        // and just being slow. OR, it could be that the threads are for reals still alive?? 
        // If it is the second one, this is a cause for concern.
    }
    ExitThread(0);
}

DWORD WINAPI master_thread(_In_ LPVOID lpParameter) {

#define PIXELS_PER_TEXEL (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)

    uint32_t maxTexelsPerThread = (uint32_t)ceilf((float)(g_image.width * g_image.height) / 
        (float)(PIXELS_PER_TEXEL * g_tc));
#if 0
    PlatformLoggerLog("maxTexelsPerThread: %d", maxTexelsPerThread);
#endif

    {
        HANDLE threadHandles[MAX_THREAD_COUNT];
        uint32_t xPos = 0;
        uint32_t yPos = 0;
        // TODO(Noah): Could do entire image as BSP tree -> assign threads to these regions.
        // then break up these regions into texels.
        texel_t *texels = nullptr;
        std::tuple<texel_t *, uint32_t> texelParams[MAX_THREAD_COUNT];
        for (uint32_t i = 0; i < g_tc; i++) {
            for (uint32_t j = 0; j < maxTexelsPerThread; j++) {
                texel_t texel;
                texel.width = THREAD_GROUP_SIZE;
                texel.height = THREAD_GROUP_SIZE;
                texel.xPos = xPos;
                texel.yPos = yPos;
                xPos += THREAD_GROUP_SIZE;
                bool isPartialTexel = false;
                if (texel.yPos + texel.height > g_image.height) {
                    texel.height -= (texel.yPos + texel.height) - g_image.height;
                    texel.height = max(texel.height, 0);
                    isPartialTexel = true;
                }
                if (xPos >= g_image.width) {
                    if (xPos > g_image.width) {
                        texel.width -= xPos - g_image.width;
                        texel.width = max(texel.width, 0);
                        isPartialTexel = true;
                    }
                    xPos = 0;
                    yPos += THREAD_GROUP_SIZE;
                }
                if (texel.xPos >= 0 && (texel.xPos + texel.width <= g_image.width) &&
                    texel.yPos >= 0 && (texel.yPos + texel.height <= g_image.height)
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
        for (uint32_t i = 0; i < g_tc; i++) {
            texelParams[i] = std::make_tuple(
                texels + i * maxTexelsPerThread,
                (i + 1 < g_tc) ? maxTexelsPerThread :
                StretchyBufferCount(texels) - (g_tc - 1) * maxTexelsPerThread
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
        for (uint32_t i = 0; i < g_tc; i++) {
            WaitForSingleObject(threadHandles[i], INFINITE);
        }
        StretchyBufferFree(texels);
    }

#undef PIXELS_PER_TEXEL
    
    bool bFlip=true;
    WriteImage(g_image, "test.bmp",bFlip);
    printf("Done. Image written to test.bmp\n");
    ExitThread(0);
}

void automata_engine::Init(game_memory_t *gameMemory) {
    printf("Doing stuff...\n");
    game_window_info_t winInfo = automata_engine::platform::getWindowInfo();
    g_image = AllocateImage(winInfo.width, winInfo.height);    
    
    g_materials[0].emitColor = V3(65/255.f,108/255.f,162/255.f);//sky.

    g_lights[0].kind = LIGHT_KIND_DIRECTIONAL;
    g_lights[0].direction = Normalize(V3(1.f,1.f,-1.f));
    g_lights[0].radiance = 1.5f *V3(1.f,1.f,1.f); //g_materials[0].emitColor;

    //g_materials[1].albedo = V3(0.5f, 0.5f, 0.5f);
    g_materials[1].albedoIdx=1;
    g_materials[1].metalnessIdx=2;
    g_materials[1].roughnessIdx=3;
    g_materials[1].normalIdx=4;
    g_materials[1].metalColor=V3(0.562f,0.565f,0.578f);//iron in air.

    g_materials[2].albedo = V3(0.7f, 0.25f, 0.3f);
    //g_materials[2].ior=2.417f; // diamond.
    g_materials[2].roughness = 0.f;
    //g_materials[2].alpha=0.0f; // diamond.
    g_materials[3].albedo = V3(0.0f, 0.8f, 0.0f);
    g_materials[3].roughness = 0.0f;
    g_materials[3].ior=1.31f; // ice.    
    g_materials[4].albedo = V3(0.3f, 0.25f, 0.7f);
    g_materials[4].ior=1.544f; // fused silica; a pure form of glass.    
    { // generate debug g_materials for occtree voxels.
        int s=1<<LEVELS;
        for (int i=0;i<s;i++)
        for (int j=0;j<s;j++)
        for (int k=0;k<s;k++)
        {
            g_materials[5 + i * s * s + j * s + k].albedo = V3( s/(float)i,s/(float)j,s/(float)k );
            g_materials[5 + i * s * s + j * s + k].roughness=1.f;
        }
    }

    g_lights[1].kind = LIGHT_KIND_POINT;
    g_lights[1].position = V3(2,-5,2);
    g_lights[1].radiance = 1.5f *V3(0.0f, 0.8f, 0.0f);

    g_planes[0].n = V3(0,0,1);
    g_planes[0].d = 0; // plane on origin
    g_planes[0].matIndex = 1;
    g_spheres[0].p = V3(0,0,0);
    g_spheres[0].r = 1.0f;
    g_spheres[0].matIndex = 2;
    g_spheres[1].p = V3(-1,-5,0);
    g_spheres[1].r = 1.0f;
    g_spheres[1].matIndex = 4;
    g_spheres[2].p = V3(-2,0,2);
    g_spheres[2].r = 1.0f;
    g_spheres[2].matIndex = 3;
    g_aabbs[0]=MakeAABB(v3 {}, v3 {1,1,1});
    g_aabbs[0].matIndex = 4;
    g_meshes[0].points=nullptr;
    LoadBespokeTextures();
    //LoadGltf();
    g_meshes[0].pointCount = nc_sbcount(g_meshes[0].points);
    g_world.lightCount = ARRAY_COUNT(g_lights);
    g_world.lights = g_lights;
    g_world.materialCount = (unsigned int)MaterialsCount();
    g_world.materials = g_materials;
    g_world.planeCount = ARRAY_COUNT(g_planes);
    g_world.planes = g_planes;
    g_world.sphereCount = ARRAY_COUNT(g_spheres);
    g_world.spheres = g_spheres;
    //g_world.aabbCount = ARRAY_COUNT(g_aabbs);
    g_world.aabbs = g_aabbs;
    g_world.meshCount = ARRAY_COUNT(g_meshes);
    g_world.meshes = g_meshes;
    g_world.rtas = GenerateAccelerationStructure(&g_world);
    // define camera and characteristics
    g_cameraP = V3(0, -10, 1); //  /* go back 10 and up 1 */ : V3(0, 10, 1); /* look down negative Z for physical camera */ 
    g_cameraZ = (use_pinhole)? Normalize(g_cameraP) : -Normalize(g_cameraP);
    g_cameraX = Normalize(Cross(V3(0,0,1), g_cameraZ));
    g_cameraY = Normalize(Cross(g_cameraZ, g_cameraX));
    if (g_image.width > g_image.height) {
        g_filmH = g_filmW * (float)g_image.height / (float)g_image.width;
    } else if (g_image.height > g_image.width) {
        g_filmW = g_filmH * (float)g_image.width / (float)g_image.height;
    }
    if (!use_pinhole)
        g_filmDist=1.f/(1.f/FOCAL_LENGTH-1.f/IMAGE_FOCAL_LENGTH);
    g_halfFilmW = g_filmW / 2.0f;
    g_halfFilmH = g_filmH / 2.0f;
    g_filmCenter = g_cameraP - g_filmDist * g_cameraZ;
    g_halfPixW = 1.0f / g_image.width;
    g_halfPixH = 1.0f / g_image.height;
    // print infos.
    {
        PlatformLoggerLog("camera located at (%f,%f,%f)\n", g_cameraP.x,g_cameraP.y,g_cameraP.z);
        PlatformLoggerLog("g_cameraX: (%f,%f,%f)\n", g_cameraX.x,g_cameraX.y,g_cameraX.z);
        PlatformLoggerLog("g_cameraY: (%f,%f,%f)\n", g_cameraY.x,g_cameraY.y,g_cameraY.z);
        PlatformLoggerLog("g_cameraZ: (%f,%f,%f)\n", g_cameraZ.x,g_cameraZ.y,g_cameraZ.z);
        PlatformLoggerLog(
            "g_cameraX and Y define the plane where the image plane is embedded.\n"
            "rays are shot originating from the g_cameraP and through the image plane(i.e. each pixel).\n"
            "the camera has a local coordinate system which is different from the world coordinate system.\n");
    }
    automata_engine::bifrost::registerApp("raytracer_vis", visualizer);
    g_masterThreadHandle=CreateThread(
        nullptr,
        0, // default stack size.
        master_thread,
        nullptr,
        0, // thread runs immediately after creation.
        nullptr
    );
}

rtas_node_t GenerateAccelerationStructure(world_t *world)
{
    rtas_node_t accel;
    void AdoptChildren(rtas_node_t &node, rtas_node_t B);

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


                // helper function to make it a single line.
                bool FiniteLineIntersectsWithAABB(v3 A, v3 B, float minHitDistance, aabb_t box)
                {
                    v3 dir = B-A;
                    float len=Magnitude(dir);
                    bool exitedEarly;
                    int faceHitIdx;
                    float t=doesRayIntersectWithAABB2(A, B, minHitDistance, box, &exitedEarly, &faceHitIdx);
                    return t!=minHitDistance && t <= len;
                }

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
                            // create material from gltf data.
                            cgltf_material *mat = prim.material;
                            int matIdx=1;
                            if (mat->has_pbr_metallic_roughness) {
                                cgltf_pbr_metallic_roughness *metalrough = &mat->pbr_metallic_roughness;
                                if (metalrough->base_color_texture.texture == NULL) {//support textureless materials.
                                    material_t Mat={};
                                    Mat.albedo.x=metalrough->base_color_factor[0];
                                    Mat.albedo.y=metalrough->base_color_factor[1];
                                    Mat.albedo.z=metalrough->base_color_factor[2];
                                    AddDynamicMaterial(Mat);// NOTE: we don't care about duplicates.
                                    matIdx = MaterialsCount() - 1;
                                }
                            }

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

                                    nc_sbpush(g_meshes[0].points, v1);
                                    nc_sbpush(g_meshes[0].points, v2);
                                    nc_sbpush(g_meshes[0].points, v3);
                                    nc_sbpush(g_meshes[0].matIndices, matIdx);
                                    nc_sbpush(g_meshes[0].matIndices, matIdx);
                                    nc_sbpush(g_meshes[0].matIndices, matIdx);
                                }

                                free(indexData);
                            }
                            else {
                                for (int j = 0;j < float_count;j += 3)
                                {
                                    v3 v = { posData[j],posData[j+1],posData[j+2] };
                                    nc_sbpush(g_meshes[0].points, v);
                                    nc_sbpush(g_meshes[0].matIndices, matIdx);
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

v3 brdf_diff(material_t &mat, v3 surfPoint)
{
    float piTerm=1.f/PI;
    if (mat.albedoIdx==0) {
        return piTerm*mat.albedo;
    } else {
        texture_t tex=g_textures[mat.albedoIdx-1];
        // Via the material mapping (might be UVs or some other function), sample the texture.
        v2 uv = {surfPoint.x,surfPoint.y};//for now.
        return piTerm*BespokeSampleTexture(tex, uv );
    }
    return piTerm*V3(1.f,1.f,1.f);
}

v3 brdf_specular(material_t &mat, v3 surfPoint, v3 normal, v3 L, v3 V, v3 H)
{
    float roughness;
    // Via the material mapping (might be UVs or some other function), sample the texture.
    v2 uv = {surfPoint.x,surfPoint.y};//for now.

    //float scaling=1.f/255.f;

    if (mat.roughnessIdx==0 || !g_bRoughness) {
        roughness=mat.roughness;
    } else {
        texture_t tex=g_textures[mat.roughnessIdx-1];
        v3 r3 = BespokeSampleTexture(tex, uv );
        roughness=r3.x;
    }

    v3 spec=
        /*HammonMaskingShadowing()**
        SchlickMetal(float F0, float cosTheta, metalness, 
            mat.metalColor //for now.
        );*/
        GGX(normal, H, roughness)*
        MaskingShadowing(normal, L, V, H, roughness)*
        (0.25f/fabsf(Dot(normal,L))/fabsf(Dot(normal,V)))*
        V3(1.f,1.f,1.f);
    return spec;
}

// Finds the refraction direction of a ray that is *arriving* in
// direction 'rayDir' at an air/"nglass" interface with outward-pointing
// normal 'N'.
//
// Returns true if a value is returned in 'refractionDir'.  Returns
// false if there's total internal reflection (hence, no refraction).
bool FindRefractionDirection( const v3 &rayDir, v3 N, float nglass, v3 &refractionDir )
{
    float nair, n1, n2, theta1, theta2, LHS;
    
    nair = 1.008f;

    if ((Dot(N,rayDir)) < 0) {
        // air-to-glass transition.
        n1 = nair;
        n2 = nglass;
        N = -1.f * N;
    }
    else {
        // glass-to-air transition.
        n1 = nglass;
        n2 = nair;
    }

    theta1 = acos(Dot(N,rayDir));
    
    // check for TIR.
    LHS = n1 / n2 * sin(theta1);
    if (LHS > 1.f) {
        return false;
    }
    theta2 = asin(LHS);

    v3 M = Cross(N, Cross(rayDir, N));
    M = Normalize(M);

    refractionDir = cos(theta2) * N + LHS * M;

    return true;
}

// ACES, approximation by Krzysztof Narkowicz.
static v3 TonemapPass(v3 color) {    
    const float a = 2.51f;
    const float b = 0.03f;
    const float c = 2.43f;
    const float d = 0.59f;
    const float e = 0.4f;
    color = Clamp( HadamardDiv( Hadamard(color, a*color + V3(b,b,b)) , V3(e,e,e) + Hadamard(color,c*color+V3(d,d,d)) ), 
        V3(0.0f, 0.0f, 0.0f), V3(1.0f, 1.0f, 1.0f));
    return color;
}

v3 BespokeSampleTexture(texture_t tex, v2 uv) {
    uv=v2{uv.x*float(tex.width)*0.5f,uv.y*float(tex.height)*0.5f};
    return SampleTexture(tex,uv);
}

v3 SampleTexture(texture_t tex, v2 uv) {
    //right now we only support bilinear filtering.
    int x1,x2,y1,y2;
    float s,t;
    uv={abs(uv.x),abs(uv.y)};
    x1=uv.x;
    y1=uv.y;
    s=uv.x-float(x1);
    t=uv.y-float(y1);
    // NOTE: I am aware of the precision loss for large floating point values.
    // when so far away from the camera, it's like, whatever dude.
    s = min(1.f, max(s, 0.f));
    t = min(1.f, max(t, 0.f));
    x1=x1%tex.width;
    x2=(x1+1)%tex.width;
    y1=y1%tex.height;
    y2=(y1+1)%tex.height;
    v3 top,bottom;

    assert(x1>=0&&x1<tex.width);
    assert(x2>=0&&x2<tex.width);
    assert(y1>=0&&y1<tex.height);
    assert(y2>=0&&y2<tex.height);
    assert(s>=0.f&&s<=1.f);
    assert(t>=0.f&&t<=1.f);

    top=Lerp(tex.data[y1*tex.width+x1],tex.data[y1*tex.width+x2],s);
    bottom=Lerp(tex.data[y2*tex.width+x1],tex.data[y2*tex.width+x2],s);
    return Lerp(top,bottom,t);
}

void LoadBespokeTextures(){

    void LoadTexture(texture_t *texOut,const char *filePath);
    //    // ... process data if not NULL ...
    //    // ... x = width, y = height, n = # 8-bit components per pixel ...
    //    // ... replace '0' with '1'..'4' to force that many components per pixel
    //    // ... but 'n' will always be the number that it would have been if you said 0
    //   

    LoadTexture(&g_textures[0], "res\\rusty-metal_albedo.png");
    LoadTexture(&g_textures[1], "res\\rusty-metal_metallic.png");
    LoadTexture(&g_textures[2], "res\\rusty-metal_roughness.png");
    LoadTexture(&g_textures[3], "res\\rusty-metal_normal-ogl.png");
}

void LoadTexture(texture_t *texOut,const char *filePath) {
    int x,y,n;
    unsigned int *data = (unsigned int *)stbi_load(filePath, &x, &y, &n, 4);
    if (data!=NULL){
        texOut->width=x;
        texOut->height=y;
        texOut->data=(v3*)malloc(sizeof(v3)*x*y);
        for (int Y=0;Y<y;Y++)
        for (int X=0;X<x;X++) {
            unsigned int pixel=data[Y*x+X];
            texOut->data[Y*x+X].x=float(pixel&0xFF)/255.f;
            texOut->data[Y*x+X].y=float((pixel>>8)&0xFF)/255.f;
            texOut->data[Y*x+X].z=float((pixel>>16)&0xFF)/255.f;
        }
        stbi_image_free(data);
    }
}

/*
the macro BRDF is derived from microfacet theory - a bunch of tiny little flat surfaces, each acting as a perfect
fresnel mirror. We use GGX(or the Trowbridge-Reitz distribution func) and the Smith joint masking-shadowing.
*/
// the real-time rendering book physically-based shading chapter has all the wonderful details.
// there is also of course: https://learnopengl.com/PBR/Theory.
v3 SchlickMetal(float F0, float cosTheta, float metalness, v3 surfaceColor) {
    v3 vF0  = V3(F0,F0,F0);
    vF0      = Lerp(vF0, surfaceColor, metalness);//surfaceColor depends on the metal.
    return vF0 + powf(1.f-cosTheta,5.f)*(V3(1.f,1.f,1.f)-vF0);
}

float Schlick(float F0, float cosTheta) {
    return F0+(1.f-F0)*powf(1.f-cosTheta,5.f);
}

float GGX(v3 N, v3 H, float roughness)
{
    float a2     = roughness*roughness*roughness*roughness;//Burley parameterization(Disney principled shading model).
    a2=max(1e-2,a2);// don't let roughness go to zero!

    float NdotH  = Dot(N, H);
    float denom  = (1.0f + NdotH*NdotH * (a2 - 1.0f));
    denom        = PI * denom * denom;

    return a2 / denom;
}

float MaskingShadowing(v3 normal, v3 L, v3 V, v3 H, float roughness){
    float a     = roughness*roughness ;//Burley parameterization(Disney principled shading model).
    a=max(1e-1,a);// don't let roughness go to zero!
    float Lambda(v3 N, v3 s,float a);
    if (Dot(H,V)<=0.f) return 0.f;
    return 1.f/(1.f+Lambda(normal,V,a)+Lambda(normal,L,a));
}
float Lambda(v3 N, v3 s,float a){
    return 0.5f*((a/Dot(N,s))-1.f);
}
