#include "inf_forge.h" // for the windowing/platform code.

#include <stdio.h>
#include <stdlib.h>
#include "ray.hpp"
#include <windows.h>

#define CGLTF_IMPLEMENTATION
#include "external/cgltf.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define NC_DS_IMPLEMENTATION
#include "nc_ds.h"

// debug macro interface.
#define DEBUG_MIDDLE_PIXEL 0
#define DEBUG_JUST_COSINE  0
#define DEBUG_JUST_IMPORTANT_LIGHT 0

typedef enum class debug_render_kind {
    regular,
    primary_ray_normals,
    bounce_count,
    termination_condition,
    variance
} debug_render_kind_t;


// PROGRAM SETUP.
void ParseArgs();
void PrintHelp();

// SCENE SETUP.
void LoadWorld(world_kind_t kind,camera_t *c);
void DefineCamera(camera_t *c);
void LoadGltf();
void LoadBespokeTextures();
rtas_node_t GenerateAccelerationStructure(world_t *world);

// PROGRAM FLOW FUNCTIONS.
DWORD WINAPI RenderThread(_In_ LPVOID lpParameter);
DWORD WINAPI MasterThread(_In_ LPVOID lpParameter);
void RenderTexel( unsigned int *pixel_pointer, texel_t texel );

// GEOMETRICAL HELPER FUNCTIONS.
bool FindRefractionDirection( const v3 &rayDir, v3 N, float nglass, v3 &
    refractionDir );
void BuildOrthonormalBasisFromW(v3 w, v3 *a, v3 *b, v3 *c);
float RaySphereIntersect(v3 origin, v3 direction, float minHitDistance,     
    sphere_t sphere, v3 *normal);
v3 RandomToSphere(sphere_t sphere, v3 from);
v3 RandomHalfVectorGGX(float roughness);
v3 RandomCosineDirectionHemisphere();
v3 RandomDirectionHemisphere();
static ray_payload_t RayCastIntersect(world_t *world, const v3 &rayOrigin, 
    const v3 &rayDirection);

// MISC HELPER FUNCTIONS.
void WriteDIBImage(image_32_t image, const char *fileName);
image_32_t AllocateImage(unsigned int width, unsigned int height);
mipchain_t GenerateMipmapChain(texture_t tex);
/*
beware to the unitiated - this function takes "uv",but the value should range 
from 0->size, not 0->1. 
*/
v3 SampleTexture(texture_t tex, v2 uv);
v3 BespokeSampleTexture(texture_t tex, v2 uv);

// CORE PATH TRACER ALGORITHM.
v3 BrdfDiff(material_t &mat,v3 surfPt);
v3 BrdfSpecular(material_t &mat, v3 surfPoint, v3 normal, v3 L, v3 V, v3 H, 
    float roughness);
static v3 TonemapPass(v3 pixel);
v3 SchlickMetal(float F0, float cosTheta, float metalness, v3 surfaceColor);
float HammonMaskingShadowing(v3 N, v3 L, v3 V, float roughness);
float GGX(v3 N, v3 H, float roughness);
float BurleyParameterization(float roughness);
static v3 RayCast(world_t *world, v3 o, v3 d, int depth);
bool EffectivelySmooth(float roughness);
bool IsNotEmissive(const material_t& m);


// GLOBALS
#define MAX_BOUNCE_COUNT 4
#define MAX_THREAD_COUNT 16
#define THREAD_GROUP_SIZE 32
#define RAYS_PER_PIXEL_MAX 1000              // for antialiasing.
#define MIN_HIT_DISTANCE float(1e-4)
#define WORLD_SIZE 5.0f
#define LEVELS 6
#define N_AIR 1.003f
#define FIXED_FOCAL_LENGTH 0.098f
#define MIN_ROUGHNESS float(0.01f)

static world_t g_world = {};
static camera_t g_camera = {};
static material_t *g_materials;
static light_t *g_lights = {};
static plane_t *g_planes = {};
static sphere_t *g_spheres = {};
static quad_t *g_quads = {};
static mesh_t *g_meshes = {};
static aabb_t *g_aabbs = {};
static mipchain_t g_textures[4]={};
image_32_t g_image = {};
static world_kind_t g_worldKind;
static HANDLE g_masterThreadHandle;

static int g_tc; // thread count.
static int g_sc; // sample count (render equation tap count).
static int g_pp; // sqrt(rays per pixel).

// flags for enablement.
static bool g_bNormals;
static bool g_bMetalness;
static bool g_bRoughness;
static bool g_use_pinhole;

// https://learn.microsoft.com/en-us/cpp/c-runtime-library/argc-argv-wargv?view=msvc-170&redirectedfrom=MSDN
extern int __argc;
extern char ** __argv;

constexpr debug_render_kind_t g_debugRenderKind = 
    debug_render_kind_t::regular;

auto MallocDeleter = [](auto* ptr) { free(ptr); };

/*
Hierarchy of work:
- features first with verification by looking at images.
- verify correctness more rigorously (tests). 
- then performance.
- then code readability and simpleness.
*/

/*
TODO:
APPLICATION:
X render proper orientation at runtime.
/ save proper orientation.
    - make it granular where we can flip either X or Y, or both.
/ Add cmdline processing.
    X thread count.
    - output image filepath; dynamically find extension and output based on that.
    X allow for disable DOF.
X usage/help print if supply -h.
- use stb_image_write to support .PNG and .JPG.
RENDERING FEATURES:
X tonemapping from HDR to 0->1 range.
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
X approximate the rendering equation.
    X uniform sampling in hemisphere.
    X proper implementation of a BRDF model (GGX).
/ refraction
    - different wavelengths refract differently.
/ textures for PBR materials.
    X diffuse.
    - bump map.
    / normal map.
        - support normal maps applied to surface where the normal is not pointing directly up.
    X roughness.
    X metalness.
    - mipmapping.
X diffuse and specular interaction with surfaces via fresnel equations.
/ physical camera modelling (e.g. lens).
    X depth of field
    - exposure
X use statified sampling.
- accelerate and improve quality with denoising.
/ importance sampling.
    X cosine sample the unit hemisphere.
- add early ray termination via russian roulette.
CLEANER CODE:
- Add instance transforms, which implies we will refactor for a hittable class kind of idea,and reduce our multiple
  loops to just one!
PERFORMANCE:
- Use compute shaders.
- "Some threads finish all their texels, while others are still working" - we are wasting potential good work!
   We need the master thread to notice and assign those lazy threads more work.
- vector instructions -> SIMD.
*/

/*
FUTURE WORK:
- do more than a cursory read of https://64.github.io/tonemapping/ and integrate real camera response.
- subsurface scattering models to support e.g. skin.
- add support for anisotropic BRDFs (would allow for rendering e.g. brushed metal).
- support very rough metals via multiple bounce surface reflection in the BRDF; currently such surfaces (and others?)
  rendered via current system are too dark.
- add BRDF models for cloth.
- add support for materials with nanogeometry , where geometric optics model breaks down (e.g. thin films).
- there's a whole can of worms when it comes to specular antialiasing. Open it!
    - despite shaky theory, can use Toksvig equation.
- I could read Lee et al. 2017 to determine that I don't need to bilinear filter the texture maps.
- look into ray differentials for generically determine the gradients required to select the mip level. 
*/


int main() 
{
    ParseArgs();

    const int w = 1280;
    const int h = 720;

    IF_create_window_info_t info = {};
    info.width = w;
    info.height = h;
    info.title = "Pathtracer";
    info.flags = IF_CREATE_WINDOW_NORESIZE;

    IF_window_handle_t window = IF_create_window_ex(&info);

    IF_rect_t ca = IF_get_window_clientarea(window);

    g_image = AllocateImage(ca.width, ca.height);

    LoadWorld(g_worldKind, &g_camera);

    // define camera and characteristics
    DefineCamera(&g_camera);

    // spawn the render thread.
    g_masterThreadHandle = CreateThread(
        nullptr,
        0, // default stack size.
        MasterThread,
        nullptr,
        0, // thread runs immediately after creation.
        nullptr
    );

    // handle window messages and visualize the render while it's doing the 
    // thing.
    int i = 0;
    IF_winmsg_t msg;
    while ( IF_win_poll_message(window, &msg) ) // poll_message will not block.
    {

        // handle the message.
        if (msg.isvalid) switch(msg.code) {

        }

        IF_texture_info_t backbufferinfo;
        backbufferinfo.format = IF_TEXTURE_FORMAT_RGBA8;
        backbufferinfo.width = g_image.width;
        backbufferinfo.height = g_image.height;

        IF_blit_to_window_surface( window, g_image.pixelPointer, 
            &backbufferinfo );

        DWORD result = WaitForSingleObject( g_masterThreadHandle, 0);
        if (result == WAIT_OBJECT_0) break;
        else {
            // the thread handle is not signaled - the thread is still alive
        }

    }

    IF_close_window(window);

    return 0;
}

// okay, this is really bad from, but depending on the template argument (which Pdf to eval),
// the func signature is going to change. in COSINE_PDF we pass dir in tangent space.
// whereas in PdfValue we pass dir in global.

constexpr int COSINE_PDF=0;
constexpr int TO_SPHERE_PDF=1;

template<int Pdf>
float PdfValue(v3 dir, sphere_t sphere={}, v3 from=V3(0.f,0.f,0.f))
{
    switch(Pdf) {
        case COSINE_PDF: return max(0.f,Dot(V3(0,0,1), dir)/PI); break;
        default: assert(false);//not supported.
    }

    return 0.f;
}

template<>
float PdfValue<TO_SPHERE_PDF>(v3 dir, sphere_t sphere, v3 from)
{
    // 0 if direction doesn't intersect with sphere.
    float minHitDistance = MIN_HIT_DISTANCE;
    v3 N;

    if (RaySphereIntersect(from, dir, minHitDistance, sphere, &N)
        <= minHitDistance) return 0.f;

    // https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#samplinglightsdirectly.
    float cosThetaMax = sqrt(1.f - sphere.r*sphere.r/MagnitudeSquared( from - 
        sphere.p ));
    float solid_angle = 2.f*PI*(1.f-cosThetaMax);

    return  1.f / solid_angle;
}

static unsigned int GetTotalPixelSize(image_32_t image) {
    return image.height * image.width * sizeof(unsigned int);
}

static image_32_t AllocateImage(unsigned int width, unsigned int height)
{
    image_32_t newImage = {};

    newImage.width = width;
    newImage.height = height;
    unsigned int totalPixelSize = GetTotalPixelSize(newImage);
    newImage.pixelPointer = (unsigned int *)malloc(totalPixelSize);

    return newImage;
}

void WriteDIBImage(image_32_t image, const char *fileName) 
{
    void *pixels;
    unsigned int outputPixelSize = GetTotalPixelSize(image);
    pixels = malloc(outputPixelSize);

    if (pixels==NULL) { 
        fprintf(stderr, "[ERROR] Unable to write output file,malloc error.\n");return; 
    }

    bitmap_header_t header = {};
    header.FileType = 0x4D42;   
    header.FileSize = sizeof(bitmap_header_t) + outputPixelSize;
    header.BitmapOffset = sizeof(header); 
    header.size = 40;    
    header.Width = image.width;  
    // NOTE: with a positive value for Height, the image is a Bottom-up DIB,
    // where the pixel data starts with the bottom row and progresses to the
    // top.
    header.Height = image.height;
    header.Planes = 1;          
    header.BitsPerPixel = 32;    

    FILE *fileHandle = fopen(fileName, "wb");

    if(fileHandle) {
        fwrite(&header, sizeof(header), 1, fileHandle);
        fwrite(image.pixelPointer, outputPixelSize, 1, fileHandle);
        fclose(fileHandle);
    }
    else {
        fprintf(stderr, "[ERROR] Unable to write output file,fopen error\n");
    }
}

// helper function to make it a single line.
bool RayIntersectsWithAABB(v3 rayOrigin, v3 rayDirection, float minHitDistance, 
    aabb_t box)
{
    bool exitedEarly;
    int faceHitIdx;

    float t = RayIntersectWithAABB2(rayOrigin, rayDirection, minHitDistance, 
        box, &exitedEarly, &faceHitIdx);

    return t != minHitDistance;
}

static ray_payload_t RayCastIntersect(world_t *world, const v3 &rayOrigin, 
    const v3 &rayDirection)
{
    float tolerance = TOLERANCE, minHitDistance = MIN_HIT_DISTANCE;
    ray_payload_t p = {};
    p.hitDistance = FLT_MAX;
    p.hitMatIndex = 0;
    float &hitDistance = p.hitDistance;
    unsigned int &hitMatIndex = p.hitMatIndex;
    v3 &nextNormal = p.normal;

    // sphere intersection test.
    for (
        unsigned int sphereIndex= 0;
        sphereIndex < nc_sbcount(world->spheres);
        sphereIndex++
    ) {
        sphere_t sphere = world->spheres[sphereIndex];
        float t;
        v3 N;

        if ((t = RaySphereIntersect(rayOrigin, rayDirection, minHitDistance, 
            sphere, &N)) > minHitDistance && t < hitDistance) 
        {
            hitDistance = t;
            hitMatIndex = sphere.matIndex;
            nextNormal=N;
        }
    }

    // quad intersection test.
    for (
        unsigned int quadIndex = 0;
        quadIndex < nc_sbcount(world->quads);
        quadIndex++
    ) {
        quad_t quad = world->quads[quadIndex];
        v3 N = Normalize(Cross(quad.u,quad.v));
        
        // TODO: this is a hack to account for our cornell box scene.
        float minHitDistance = 0.02;

        float t = RayIntersectPlanarShape<PLANAR_QUAD>(rayOrigin, rayDirection, 
            minHitDistance, quad.point, quad.u, quad.v);

        if ((t > minHitDistance) && (t < hitDistance)) {
            hitDistance = t;
            hitMatIndex = quad.matIndex;
            nextNormal = N;
        }
    }

    // floor intersection test.
    for (
        unsigned int planeIndex = 0;
        planeIndex < nc_sbcount(world->planes);
        planeIndex++
    ) {
        plane_t plane = world->planes[planeIndex];
        float t = RayIntersectPlane(rayOrigin, rayDirection, plane.n, plane.d, 
            minHitDistance);

        if ((t > minHitDistance) && (t < hitDistance)) {
            hitDistance = t;
            hitMatIndex = plane.matIndex;
            nextNormal = plane.n;
        }
    }

    // intersection test with the triangles in the world (via an acceleration structure).
    {
        rtas_node_t &rtas = world->rtas;
        static thread_local  stack_t<rtas_node_t*> nodes = {};
 
        if (rtas.triangleCount && RayIntersectsWithAABB(rayOrigin, 
            rayDirection, minHitDistance, rtas.bounds)) nc_spush(nodes, &rtas);

        mesh_t &mesh = world->meshes[0];

        while(nc_ssize(nodes))
        {
            rtas_node_t *r = nc_spop(nodes);
            if (r->children)
                for (int i = 0; i < nc_sbcount(r->children); i++)
                {
                    rtas_node_t *c = &r->children[i];

                    if (c->triangleCount && RayIntersectsWithAABB(rayOrigin, 
                        rayDirection, minHitDistance, c->bounds))
                    nc_spush(nodes, c);
                }
            else // leaf
                for (int i=0;i<r->triangleCount;i++)
                {
                    int triIndex = r->triangles[i];
                    v3 *points = &mesh.points[triIndex*3];

                    v3 A,B,C,u,v;
                    A = points[0];
                    B = points[1];
                    C = points[2];

                    v3 n = Normalize( Cross( u=B-A, v=C-A ) );

                    float t = 
                        RayIntersectPlanarShape<PLANAR_TRIANGLE>(rayOrigin, 
                            rayDirection, minHitDistance, A, u, v);

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
        aabbIndex < nc_sbcount(world->aabbs);
        aabbIndex++
    ) {
        aabb_t box = world->aabbs[aabbIndex];

        bool exitedEarly;
        int faceHitIdx;
        // NOTE: the faceNormals array was copied directly from within RayIntersectWithAABB2.
        // this is some garbage and not clean code. 
        constexpr v3 faceNormals[] = {
            // front, back, left, right, top, bottom.
            {0.f,0.f,-1.f}, {0.f,0.f,1.f}, {-1.f,0.f,0.f}, {1.f,0.f,0.f}, {0.f,1.f,0.f}, {0.f,-1.f,0.f}
        };
        float t = RayIntersectWithAABB2(rayOrigin, rayDirection, 
            minHitDistance, box, &exitedEarly, &faceHitIdx);

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
    if (depth >= MAX_BOUNCE_COUNT) return radiance;

    rayOrigin = o;
    rayDirection = d;

    ray_payload_t p;
    float &hitDistance = p.hitDistance;
    unsigned int &hitMatIndex = p.hitMatIndex;
    v3 &nextNormal = p.normal;

    p = RayCastIntersect(world, rayOrigin, rayDirection);
      
    material_t mat = world->materials[hitMatIndex];
    v3 N=nextNormal;

    bool bHitSky = (hitMatIndex==0);
    bool bHitLight = !IsNotEmissive(mat);
    bool bIsTerminalRay = (depth == MAX_BOUNCE_COUNT - 1);

    float NdotL, NdotV;

    do {

        // NOTE: We terminate at emissive materials since the photons originate 
        // from these and we are actually tracing
        // backwards.
        if (bHitSky || bHitLight) break;

        float cosTheta,F0,metalness,roughness;
        v3 halfVector,L,V,pureBounce,brdfTerm,ks_local,kd_local,r3,tangentX,
            tangentY,tangentZ;

        cosTheta = (Dot(nextNormal, rayDirection));
        cosTheta = (cosTheta>0.f) ? Dot(-1.f*nextNormal, rayDirection) : 
            cosTheta;

        F0 = Square((N_AIR-mat.ior)/(N_AIR+mat.ior)); // NOTE: need to change 
        // when support refraction again.

        int tapCount = g_sc;
        //float tapContrib = 1.f / float(tapCount + nc_sbcount(world->lights));
        float tapContrib = 1.f/tapCount;

        rayOrigin = rayOrigin + hitDistance * rayDirection;
        pureBounce = rayDirection - 2.0f * cosTheta * nextNormal;

        V = -rayDirection;

        // Via the material mapping (might be UVs or some other function), sample the texture.
        v2 uv = {rayOrigin.x,rayOrigin.y};//for now.

        { // find the metalness.
            if (mat.metalnessIdx==0 || !g_bMetalness) {
                metalness = mat.metalness;
            } else {
                mipchain_t chain = g_textures[mat.metalnessIdx-1];
                texture_t tex = chain.mips[0];
                r3 = BespokeSampleTexture( tex, uv );
                metalness = r3.x;
            }
        }

        if (mat.roughnessIdx==0 || !g_bRoughness) {
            roughness = mat.roughness;
        } else {
            mipchain_t chain = g_textures[mat.roughnessIdx-1];
            texture_t tex = chain.mips[0];
            v3 r3 = BespokeSampleTexture(tex, uv );
            roughness = r3.x;
        }

#if 1
        // find the normal.
        if (g_bNormals && mat.normalIdx!=0){
            mipchain_t chain = g_textures[mat.normalIdx-1];
            texture_t tex = chain.mips[0];
            N = BespokeSampleTexture(tex, uv );
            // NOTE: this currently only works for the ground plane, since it's normal happens to be up!
            N = Normalize(2.f*N - V3(1.f,1.f,1.f));
        }
#endif

        // if we do not support refraction, this should never occur.
        NdotV = Dot(N, V);
        if ((NdotV)<=0.f) break;

        // Define the local tangent space using the normal.
        BuildOrthonormalBasisFromW( N, &tangentX, &tangentY, &tangentZ );

        // use monte carlo estimator.
        const bool bJustCosine=DEBUG_JUST_COSINE || 
            (g_worldKind==WORLD_RAYTRACING_ONE_WEEKEND);
        constexpr bool bJustImportance=DEBUG_JUST_IMPORTANT_LIGHT;
        assert( !(bJustImportance && bJustCosine) &&
             "they can't both be true." );

        do {
            bool bSpecular = RandomUnilateral()>0.5f;
            float px;

            // the code below is somewhat nuanced. we use a correction weight. what is the "correction weight"?
            // well, the total brdf term is specular+diffuse. so, we can split that integral into two,
            // because integrals are linear. thus, what was a single statistical estimator before is
            // split into two estimators. so, the probabilistic bSpecular choice here has nothing to do
            // with PDF mixtures. it selects which estimator we will add a sample from. finally, the correction
            // weight multiplies by two since the 1/N term that occurs at the end isn't aware of the funny
            // business that we're doing here.

            if (bSpecular && EffectivelySmooth(roughness)) {
                L = pureBounce;
                px = 1.f;
            }
            else if (!bSpecular) {

                bool bSampleCosine = RandomUnilateral()>0.5f;

                //hittable_t importantLight; // Three cases. 1: sample sphere. 2. sample square light. 3. sample nothing,because there is no light.
                //assert(incomingLight)

                sphere_t importantLight = world->spheres[0];//for now, this 
                //only works for some scenes.

                v3 rDir,lobeBounce;

                if ( !bJustImportance && (bSampleCosine || bJustCosine) )
                {
                    rDir=RandomCosineDirectionHemisphere();   
                } 
                else if ( !bJustCosine && (!bSampleCosine || bJustImportance) ) 
                {
                    v3 direction = importantLight.p - rayOrigin;
                    rDir = RandomToSphere(importantLight, rayOrigin);
                    BuildOrthonormalBasisFromW( direction, &tangentX, &
                        tangentY, &tangentZ );
                }

                if (rDir==V3(0,0,0)) continue; // e.g., rayOrigin is for some reason inside the sphere.
                
                lobeBounce = Normalize(rDir.x * tangentX + rDir.y * 
                    tangentY + rDir.z * tangentZ);
                L = lobeBounce;
                halfVector = (1.f/Magnitude(L+V)) * (L+V);

                if ( !bJustCosine && !bJustImportance ) {
                    // Now that we are using a pdf mixture, it's important to retain the cosTheta term from rendering equation.
                    px = 0.5f * PdfValue<COSINE_PDF>(Normalize(rDir)) + 
                        0.5f * PdfValue<TO_SPHERE_PDF>(lobeBounce,
                            importantLight, rayOrigin);
                }

                if ( bJustCosine ) {
                    px = PdfValue<COSINE_PDF>(Normalize(rDir));
                }

                if ( bJustImportance ) {
                    px = PdfValue<TO_SPHERE_PDF>(lobeBounce,importantLight,rayOrigin);
                }

                if (px==0.f) continue;

            } else {
                // reflection equation from: https://schuttejoe.github.io/post/ggximportancesamplingpart1/
                v3 rDir = RandomHalfVectorGGX(roughness);
                halfVector = Normalize( rDir.x * tangentX + rDir.y * 
                    tangentY + rDir.z * N);
                L = 2.f * (Dot(V, halfVector)) * halfVector - V;
                px = 1.f;
            }

            if ( (NdotL=Dot(N, L))>0.f )  //incoming light is in hemisphere.
            {
                // NOTE: the difference here is maybe a little bit subtle. when the surface is perfectly smooth, we
                // don't require microfacet theory. thus, we won't be using the half vector.

                if (EffectivelySmooth(roughness)) {
                    ks_local = SchlickMetal(F0,NdotL,metalness,mat.metalColor);
                    //ks_local = SchlickMetal(F0,cosTheta,metalness,mat.metalColor);
                } 
                else if ( ((Dot(halfVector,V) > 0.f) && 
                    (cosTheta = Dot(halfVector,L)) >0.f) )
                {
                    ks_local = SchlickMetal(F0,cosTheta,metalness,
                        mat.metalColor);
                } else {
                    break;
                }
                
                kd_local = V3(1.f,1.f,1.f) - ks_local;
                for (int j=0;j<3;j++) assert(ks_local.E[j] >= 0.f && 
                    ks_local.E[j] <= 1.f);

                // metals (conductors) have special properties w.r.t. light.
                // when the EM wave arrives at a conductor interface, the wave goes to zero quite quickly at a characteristic
                // length-scale called the skin-depth.
                // (effectively, is completely absorbed and there is no scattering => less diffuse color).
                kd_local = Lerp(kd_local, V3(0,0,0), metalness);
                
                if (bSpecular && EffectivelySmooth(roughness)) {
                    // if the surface is perfectly smooth, there is no need for microfacet theory and the
                    // brdf is given by the dirac delta in the perfect fresnel ref dir;
                    brdfTerm = ks_local;
                    //and therefore,
                    // we don't need an estimator either. We just take the term.
                } else if (bSpecular) {
                    // NOTE: for specular, the 1/p(x) term is baked into BrdfSpecular.
                    brdfTerm = Hadamard(ks_local, BrdfSpecular(mat,rayOrigin, 
                        N, L, V, halfVector, roughness ) );
                } else {
                    brdfTerm = NdotL * Hadamard(kd_local, BrdfDiff(mat,rayOrigin));
                }

                if constexpr (g_debugRenderKind == 
                    debug_render_kind_t::regular || g_debugRenderKind == 
                    debug_render_kind_t::variance) 
                {
                    // NOTE: since we sample by cos(theta), the NdotL term goes away by the 1/p(x) term.
                    radiance += 2.f * (1.f/px) * Hadamard(RayCast(world,
                        rayOrigin,L,depth+1), brdfTerm);
                }

                if constexpr (g_debugRenderKind == 
                    debug_render_kind_t::bounce_count || g_debugRenderKind == 
                    debug_render_kind_t::termination_condition) 
                {
                    radiance += RayCast(world,rayOrigin,L,depth+1);
                }
            }
            break;
        } while(true); // END spawning the bounce rays.
        
    } while(false); // END do while structure.

    if constexpr (
        g_debugRenderKind == debug_render_kind_t::regular ||
        g_debugRenderKind == debug_render_kind_t::variance
    ) radiance += mat.emitColor;

    if constexpr (g_debugRenderKind == debug_render_kind_t::bounce_count) {
        float bounceContrib = 1.f / float(MAX_BOUNCE_COUNT);
        radiance += V3(bounceContrib,bounceContrib,bounceContrib);
    }

    if constexpr (g_debugRenderKind == debug_render_kind_t::primary_ray_normals)
        radiance = 0.5f * N + V3(0.5,0.5,0.5);

    if constexpr (g_debugRenderKind == 
        debug_render_kind_t::termination_condition
    ) {
        if (bHitSky)
            radiance = V3(0,0,1);
        else if (bHitLight)
            radiance = V3(0,1,0);
        else if (bIsTerminalRay)
            radiance = V3(1,0,0);
        else if (NdotV <= 0.f)
            radiance = V3(1,1,0);
    }

    return radiance;
}

bool IsNotEmissive(const material_t& m){
    return (m.emitColor==V3(0,0,0));
}

typedef struct {
    texel_t texel;
    bool done;
    bool shouldquit;
} render_thread_data_t;

// NOTE: apparently these intrinsics are deprecated but what the hell, I want
// to program in C, not C++ !?
#define WriteGlobals() _WriteBarrier(), MemoryBarrier()
#define PrepareGlobals() _ReadBarrier(), MemoryBarrier()

// NOTE: 
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
DWORD WINAPI RenderThread(_In_ LPVOID lpParameter)
{
    render_thread_data_t *pdata = (render_thread_data_t *)lpParameter;

    PrepareGlobals();

    if (pdata)
    while (!pdata->shouldquit)
    {
        PrepareGlobals(); if ( !pdata->done )
        {
            texel_t texel = pdata->texel;
            unsigned int *out =
                g_image.pixelPointer + texel.yPos * g_image.width + texel.xPos;

            RenderTexel( out, texel );

            WriteGlobals();

            pdata->done = true;
        }
    }

    ExitThread(0);
}

render_thread_data_t g_renderdata[MAX_THREAD_COUNT];

DWORD WINAPI MasterThread(_In_ LPVOID lpParameter) {

#define PIXELS_PER_TEXEL (THREAD_GROUP_SIZE * THREAD_GROUP_SIZE)

    HANDLE threadHandles[MAX_THREAD_COUNT];

    texel_t *texels = nullptr;


    for (int y = 0; y < g_image.height; y += THREAD_GROUP_SIZE)
    for (int x = 0; x < g_image.width; x += THREAD_GROUP_SIZE)
    {
        texel_t texel;
        texel.width = THREAD_GROUP_SIZE;
        texel.height = THREAD_GROUP_SIZE;
        texel.xPos = x;
        texel.yPos = y;

        // clip the edges if needed.
        texel.width = min((texel.xPos + THREAD_GROUP_SIZE), g_image.width) - 
            texel.xPos;
        texel.height = min((texel.yPos + THREAD_GROUP_SIZE), g_image.height) - 
            texel.yPos;

        nc_sbpush(texels, texel);
    }

    int threadcount = min(g_tc, nc_sbcount(texels));

    // spawn the render threads.
    //
    // NOTE: The reason we split up the for-loop is because texels base addr
    // is not stable until we have finished pushing (this is due to stretchy 
    // buff logic).
    for (int i = 0; i < threadcount; i++)
    {
        render_thread_data_t data;

        texel_t texel = texels[i];
        data.texel = texel;
        data.done = false;
        data.shouldquit = false;
        g_renderdata[i] = data;

        WriteGlobals();

        threadHandles[i] = CreateThread(
            nullptr,
            0, // default stack size.
            RenderThread,
            (LPVOID)&g_renderdata[i],
            0, // thread runs immediately after creation.
            nullptr
        );
    }

    // when render threads are done their texel, give them another one
    // until there are no texels left.
    for (int i = threadcount; i < nc_sbcount(texels); i++ )
    {
        texel_t texel = texels[i];

        for (int j = 0; j < threadcount; j++) {

            render_thread_data_t *pdata = &g_renderdata[j];

            PrepareGlobals(); if (pdata->done) {

                pdata->texel = texels[i];

                WriteGlobals();

                pdata->done = false;

                i++;
                break;
            }
        }

        i--; // if no threads were complete their work we need to wait.
        //std::this_thread::yield();
    }

    // signal for all threads to complete.
    for (int i = 0; i < threadcount; i++) {

        render_thread_data_t *pdata = &g_renderdata[i];

        if (pdata->done) {
            g_renderdata[i].shouldquit = true;
            WriteGlobals();
        } else {
            i--; // cannot continue until this thread is complete.
        }
    }

    // wait for all threads to complete.
    for (int i = 0; i < threadcount; i++) {
        WaitForSingleObject(threadHandles[i], INFINITE);
    }

    nc_sbfree(texels);

#undef PIXELS_PER_TEXEL
    
    WriteDIBImage(g_image, "test.bmp");
    printf("Done. Image written to test.bmp.\n");
    ExitThread(0);
}

// render a texl on the film plane.
void RenderTexel( unsigned int *out, texel_t texel )
{
    // Raytracer works by averaging all colors from all rays shot from this 
    // pixel.

    for (unsigned int y = texel.yPos; y < (texel.height + texel.yPos); y++)
    {
        for ( unsigned int x = texel.xPos; x < (texel.width + texel.xPos); x++) 
        {
            
            // frustrum vals are values in the range [-1,1].
            // the frustrum plane is that from the camera and in the direction
            // of the scene.
            const float frustrumY = -1.0f + 2.0f * (float)y / (float)g_image.
                height;
            const float frustrumX = -1.0f + 2.0f * (float)x / (float)g_image.
                width;

            v3 color = {};
            v3 radiance = {};

#if DEBUG_MIDDLE_PIXEL
            if ((y != (g_image.height/2)) || (x!= (g_image.width/2))) 
                continue;
#endif

            constexpr bool bVarRender = g_debugRenderKind == 
                debug_render_kind_t::variance;
            
            /* 
            how does the decltype thing work? well, lambda syntax is 
            shorthand for defining a new functor type.
            we use decltype to get that functor type back for giving to 
            the template, as required. 
            */

            std::unique_ptr<v3, decltype(MallocDeleter)> 
                vListManager { bVarRender ? (v3*)malloc( g_pp *g_pp * 
                    sizeof(v3)) : nullptr };
            v3 *vList = vListManager.get();

            // pinhole does not have depth of field.
            if ( g_camera.use_pinhole ) {

                v3 pinhole = g_camera.pos; 
                
                float contrib;
                v3 rayDirection, frustrumP;

                // cast a number of rays per pixel in a statified manner.
                contrib = 1.0f / (float)g_pp / (float)g_pp;
                for (int i = 0;i < g_pp;i++) {
                    for (int j = 0;j < g_pp;j++) {

                        float llpixelX = frustrumX - 1.f * g_camera.
                            halfFilmPixelW;
                        float llpixelY = frustrumY - 1.f * g_camera.
                            halfFilmPixelH;
                        float stepX = 1.f / g_pp * g_camera.halfFilmPixelW*2.f;
                        float stepY = 1.f / g_pp * g_camera.halfFilmPixelH*2.f;

                        float xStep = llpixelX + float(i) / g_pp * g_camera.
                            halfFilmPixelW + stepX*0.5f;
                        float yStep = llpixelY + float(j) / g_pp * g_camera.
                            halfFilmPixelH + stepY*0.5f;
                        
                        xStep += (RandomUnilateral() - 0.5f) * stepX;
                        yStep += (RandomUnilateral() - 0.5f) * stepY;

                        frustrumP = g_camera.frustrumCenter + 
                            ( xStep * g_camera.halfFilmWidth * g_camera.axisX) +
                            ( yStep * g_camera.halfFilmHeight * g_camera.axisY);

                        // towards the film means towards the scene.
                        rayDirection = Normalize(frustrumP - pinhole);

                        radiance = RayCast(&g_world, pinhole, rayDirection,0);

                        if (IsNaN(radiance)) {j--; continue;} // try again.
                        color = color + contrib * radiance;

                        if constexpr (bVarRender) vList[i * g_pp + j] = 
                            radiance;
                    }
                }

                if constexpr (bVarRender) {
                    v3 var=V3(0,0,0);
                    for (int i=0; i < g_pp * g_pp; i++){
                        var = var + contrib * Hadamard( vList[i]-color, vList[i]-color );
                    }
                    color=var;
                }

            } 
            // if not the pinhole model, we use a more physical camera model 
            // with a real aperature and lens.
            else 
            {
                assert(bVarRender == false && "not supported");

                float contrib = 1.0f / (float)g_pp/(float)g_pp;

                // the poisson disk samples
                // the number of samples to take.
                const int NUM_SAMPLES = 12;     

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

                v3 lensCenter = g_camera.pos;

                for (unsigned int rayIndex = 0; rayIndex < g_pp; rayIndex++) { // integrate across image sensor.
                    
                    float offX = frustrumX + (RandomBilateral() * g_camera.
                        halfFilmPixelW);
                    float offY = frustrumY + (RandomBilateral() * g_camera.
                        halfFilmPixelH);

                    v3 frustrumP = g_camera.frustrumCenter + 
                        (offX * g_camera.halfFilmWidth * g_camera.axisX) + 
                        (offY * g_camera.halfFilmHeight * g_camera.axisY);

                    v3 rayDirection = Normalize(frustrumP - lensCenter);

                    // intersect with focal plane.
                    // https://computergraphics.stackexchange.com/questions/246/how-to-build-a-decent-lens-camera-objective-model-for-path-tracing
                    // 1/f = 1/v + 1/b.
                    float focalPlaneDist = 1.f/(
                        1.f/FIXED_FOCAL_LENGTH - 1.f/g_camera.focalLength);
                    v3 planePoint, N = -g_camera.axisZ, focalPoint;
                    planePoint = 
                        lensCenter + g_camera.axisX + focalPlaneDist * N;
                    float d = Dot(N, planePoint);

                    float t = RayIntersectPlane( lensCenter, rayDirection, 
                        N, d, MIN_HIT_DISTANCE);

                    assert(t > MIN_HIT_DISTANCE);

                    focalPoint = lensCenter + t * rayDirection;

                    // sample poisson disk point.
                    for (
                        unsigned int rayIndex2 = 0;
                        rayIndex2 < g_pp; rayIndex2++) 
                    {

                        v2 diskSample = poissonDisk[
                            (rayIndex2 * rayIndex) % NUM_SAMPLES];

                        v3 rayOriginDisk, rayDirectionDisk;

                        rayOriginDisk = lensCenter + diskSample.x * 
                            g_camera.aperatureRadius * g_camera.axisX + 
                            diskSample.y * g_camera.aperatureRadius * 
                            g_camera.axisY;

                        rayDirectionDisk = Normalize(focalPoint -   
                            rayOriginDisk);

                        radiance = RayCast(&g_world, rayOriginDisk,
                            rayDirectionDisk, 0);

                        if (IsNaN(radiance)) {rayIndex2--;continue;}//try again.
                        color = color + contrib * radiance;
                    }
                }
            }

            if constexpr (g_debugRenderKind == debug_render_kind_t::regular)
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
}

rtas_node_t GenerateAccelerationStructure(world_t *world)
{
    rtas_node_t accel;
    void AdoptChildren(rtas_node_t &node, rtas_node_t B);

    // early exit if the scene does not contain meshes.
    if (nc_sbcount(world->meshes) == 0)
    {
        accel.triangleCount = 0;
        return accel;
    }

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
        meshIndex < nc_sbcount(world->meshes);
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
                    float t=RayIntersectWithAABB2(A, B, minHitDistance, box, &exitedEarly, &faceHitIdx);
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
    nc_sbpush(g_meshes,mesh_t{});//load empty.

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
                                    nc_sbpush(g_materials, Mat);
                                    matIdx = nc_sbcount(g_materials) - 1;
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

v3 BrdfDiff(material_t &mat, v3 surfPoint)
{
    float piTerm=1.f/PI;
    if (mat.albedoIdx==0) {
        return piTerm*mat.albedo;
    } else {
        mipchain_t chain=g_textures[mat.albedoIdx-1];
        texture_t tex=chain.mips[0];
        // Via the material mapping (might be UVs or some other function), sample the texture.
        v2 uv = {surfPoint.x,surfPoint.y};//for now.
        return piTerm*BespokeSampleTexture(tex, uv );
    }
    return piTerm*V3(1.f,1.f,1.f);
}

v3 BrdfSpecular(material_t &mat, v3 surfPoint, v3 normal, v3 L, v3 V, v3 H, float roughness)
{
    // Via the material mapping (might be UVs or some other function), sample the texture.
    v2 uv = {surfPoint.x,surfPoint.y};//for now.

    //float scaling=1.f/255.f;

    v3 spec = HammonMaskingShadowing(normal, L, V, roughness)*
        (fabsf(Dot(H,L))/fabsf(Dot(normal,L))/fabsf(Dot(H,normal)))*V3(1.f,1.f,1.f);
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

    void LoadTexture(mipchain_t *texOut,const char *filePath);
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

void LoadTexture(mipchain_t *texOut,const char *filePath) {
    int x,y,n;
    unsigned int *data = (unsigned int *)stbi_load(filePath, &x, &y, &n, 4);
    if (data!=NULL){
        texture_t tex;
        tex.width=x;
        tex.height=y;
        tex.data=(v3*)malloc(sizeof(v3)*x*y);
        for (int Y=0;Y<y;Y++)
        for (int X=0;X<x;X++) {
            unsigned int pixel=data[Y*x+X];
            tex.data[Y*x+X].x=float(pixel&0xFF)/255.f;
            tex.data[Y*x+X].y=float((pixel>>8)&0xFF)/255.f;
            tex.data[Y*x+X].z=float((pixel>>16)&0xFF)/255.f;
        }
        stbi_image_free(data);
        *texOut=GenerateMipmapChain(tex);
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

float GGX(v3 N, v3 H, float roughness)
{
    float a2     = BurleyParameterization(roughness);

    float NdotH  = Dot(N, H);
    float denom  = (1.0f + NdotH*NdotH * (a2 - 1.0f));
    denom        = PI * denom * denom;

    // TODO: what's the proper thing here?
    if (denom==0.f) return 1.f;

    return a2 / denom;
}

// presented at GDC!
float HammonMaskingShadowing(v3 N, v3 L, v3 V, float roughness){
    float a2     = BurleyParameterization(roughness);
    // we know that both NdotV and NdotL are positive and nonzero!
    float NdotV=Dot(N,V);
    float NdotL=Dot(N,L);
    float numerator = 2.f*NdotL*NdotV;
    float denom=NdotV * sqrt(a2 + (1.f-a2)*NdotL*NdotL) + NdotL * sqrt(a2 + (1-a2)*NdotV*NdotV);
    return numerator/denom;
}

bool EffectivelySmooth(float roughness){
    if (roughness<MIN_ROUGHNESS)return true;
    return false;
}

void LoadWorld(world_kind_t kind, camera_t *c)
{
    plane_t MakeGroundPlane(unsigned int mat);
    void AddSky(v3 color);
    void AddSunDirectionalLight();

    light_t light;
    plane_t plane;
    material_t material;
    sphere_t sphere;
    quad_t quad;

    // init the camera params.
    c->use_pinhole=g_use_pinhole;
    c->pos=V3(0, -10, 1); // go back 10 and up 1.
    c->fov=45.f;//degrees,obviously.
    c->focalDistance=5.f;
    c->aperatureRadius=0.035f;
    c->target={};//origin of space,duh.
    
    switch(kind) {
        case WORLD_DEFAULT: {
            AddSky(V3(65/255.f,108/255.f,162/255.f));
            AddSunDirectionalLight();

            unsigned int planeMat = nc_sbcount(g_materials);
            material={.albedoIdx=1,.metalnessIdx=2,.metalColor=V3(0.562f,0.565f,0.578f),.roughnessIdx=3,.normalIdx=4};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(0,0,-1000),.r = 1000,.matIndex = planeMat};
            nc_sbpush(g_spheres,sphere);
            
            LoadBespokeTextures();

            unsigned int mat = nc_sbcount(g_materials);
            material={.albedo = V3(0.7f, 0.25f, 0.3f),.roughness = 0.f};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(0,0,0),.r = 1.0f,.matIndex = mat};
            nc_sbpush(g_spheres,sphere);

            mat = nc_sbcount(g_materials);
            material={.albedo = V3(0.0f, 0.8f, 0.0f),.metalness=0.8f,
                .metalColor=V3(0.562f,0.565f,0.578f),
                .roughness = 0.0f,};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(-2,0,2),.r = 1.0f,.matIndex = mat};
            nc_sbpush(g_spheres,sphere);

            mat = nc_sbcount(g_materials);
            material={.albedo = V3(0.3f, 0.25f, 0.7f),.roughness=0.f};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(-1,-5,0),.r = 1.0f,.matIndex = mat};
            nc_sbpush(g_spheres,sphere);

            c->fov=30.f;//degrees,obviously.
        }
            break;
        case WORLD_CORNELL_BOX: {
            AddSky(V3(0.f,0.f,0.f));

            int left,/*->*/right,bottom,/*->*/top,front,/*->*/back;
            left=0;
            right=800;
            bottom=0;
            top=555;
            front=0;
            back=555;

            unsigned int red = nc_sbcount(g_materials);
            material={.albedo=(V3(.65, .05, .05))};
            nc_sbpush(g_materials,material);
            unsigned int white = nc_sbcount(g_materials);
            material={.albedo=(V3(.73, .73, .73))};
            nc_sbpush(g_materials,material);
            unsigned int green = nc_sbcount(g_materials);
            material={.albedo=(V3(.12, .45, .15))};
            nc_sbpush(g_materials,material);
            unsigned int light = nc_sbcount(g_materials);
            material={.albedo=V3(0,0,0)/*black body radiation source*/,
                .emitColor=V3(15.f,15.f,15.f)};
            nc_sbpush(g_materials,material);

            // right wall.
            quad={.point=V3(right,bottom,front),.u=V3(0,0,top-bottom),.v= V3(0,back-front,0), .matIndex=green}; // Z cross Y equals -X.
            nc_sbpush(g_quads,quad);

            // left wall.
            quad={.point=V3(left,bottom,front),.u=V3(0,back-front,0), .v=V3(0,0,top-bottom), .matIndex=red}; // Y cross Z equals X.
            nc_sbpush(g_quads,quad);

            sphere={.p=V3((right-left)/2.f,(back-front)/2.f,(top-bottom)/2.f),.r=65,.matIndex= light};
            nc_sbpush(g_spheres,sphere);

            // ceiling.
            quad={.point=V3(left,front,top), .u=V3(0,back-front,0), .v=V3(right-left,0,0), .matIndex= white};
            nc_sbpush(g_quads,quad);

            // back face wall.
            // the normal of this wall points towards the camera center, so it is in negative Y direction.
            quad={.point=V3(left,back,bottom),.u=V3(right-left,0,0),.v=V3(0,0,top-bottom)  ,.matIndex= white};
            nc_sbpush(g_quads,quad);

            // floor.
            quad={.point=V3(left,bottom,front),.u= V3(right-left,0,0),.v=  V3(0,back-front,0),.matIndex= white};
            nc_sbpush(g_quads,quad);

            // cam.aspect_ratio      = 1.0;
            // cam.image_width       = 600;
            //cam.samples_per_pixel = 200;
            //cam.max_depth         = 50;
            c->fov =40;
            c->pos = V3((right-left)/2.f, front-800, (top-bottom)/2.f );
            c->target   = V3((right-left)/2.f, front, (top-bottom)/2.f);
            // cam.defocus_angle = 0;
        } break;
        // try to roughly match: https://cdn-images-1.medium.com/v2/resize:fit:800/1*IBg4O5MyKVmwyA2DhoBBVA.jpeg.
        case WORLD_BRDF_TEST: {
            AddSky(V3(65/255.f,108/255.f,162/255.f));
            AddSunDirectionalLight();

            unsigned int planeMat = nc_sbcount(g_materials);
            material={.albedo = V3(0.5f, 0.5f, 0.5f)};
            nc_sbpush(g_materials, material);//ground material.
            nc_sbpush(g_planes, MakeGroundPlane(planeMat));

            for (int i=0;i<11;i++)
            for (int j=0;j<11;j++) {
                unsigned int newMat=nc_sbcount(g_materials);
                v3 center = V3(i/2.f, 11/2.f-j/2.f, 0.2);
                v3 color = V3(1.0f,0.782f,0.344f);
                material={.albedo = color, .metalness=i/10.f, .metalColor=color,
                    .roughness=j/10.f};
                nc_sbpush(g_materials,material);
                sphere={.p = center,.r = 0.2f,.matIndex = newMat};
                nc_sbpush(g_spheres,sphere);
            }

            c->target=V3(2.5,2.5,0);
            c->pos = V3(2.5,7,2);
            c->fov=50.f;
            c->focalDistance=10.f;
        }
            break;
        case WORLD_MARIO: {
            AddSky(V3(65/255.f,108/255.f,162/255.f));
            AddSunDirectionalLight();

            unsigned int planeMat = nc_sbcount(g_materials);
            material={.albedo = V3(0.5f, 0.5f, 0.5f)};
            nc_sbpush(g_materials, material);//ground material.
            nc_sbpush(g_planes, MakeGroundPlane(planeMat));

            LoadGltf();
            g_meshes[0].pointCount = nc_sbcount(g_meshes[0].points);

            { // generate debug g_materials for occtree voxels.
                int s=1<<LEVELS;
                for (int i=0;i<s;i++)
                for (int j=0;j<s;j++)
                for (int k=0;k<s;k++)
                {
                    //material={};
                    //g_materials[materialsBefore + i * s * s + j * s + k].albedo = V3( s/(float)i,s/(float)j,s/(float)k );
                    //g_materials[materialsBefore + i * s * s + j * s + k].roughness=1.f;
                }
            }

            // adjust camera.
            c->target=V3(0,0,1);
            c->pos=V3(-5, -5, 1); // go back 10 and up 1.
            c->fov=30.f;
        }
            break;
        case WORLD_RAYTRACING_ONE_WEEKEND: {

            // NOTE: I copied the code from https://raytracing.github.io/books/RayTracingInOneWeekend.html#wherenext?/afinalrender
            // and adapted it. some variable names therefore do not follow the same convention as the rest of this codebase.

            AddSky(V3(1.f,1.f,1.f));

            //make the ground plane.
            unsigned int ground_material=nc_sbcount(g_materials);
            material={.albedo = V3(0.5f, 0.5f, 0.5f)};
            nc_sbpush(g_materials,material);

            sphere={.p = V3(0,0,-1000),.r = 1000,.matIndex = ground_material};
            nc_sbpush(g_spheres,sphere);

            for (int a = -11; a < 11; a++) {
                for (int b = -11; b < 11; b++) {
                    float choose_mat = RandomUnilateral();
                    v3 center=V3(a + 0.9*RandomUnilateral(), b + 0.9*RandomUnilateral(), 0.2);

                    if (Magnitude(center - V3(4, 0, 0.2)) > 0.9) {
                        unsigned int newMat=nc_sbcount(g_materials);
                        
                        if (choose_mat < 0.8) {
                            // diffuse
                            material={.albedo = Hadamard( RandomV3(),RandomV3() )};
                            nc_sbpush(g_materials,material);
                        } else 
                        //if (choose_mat < 0.95)
                        {
                            // metal
                            material={
                                .metalness  = RandomUnilateral(),
                                .metalColor =  0.5f*RandomV3()+V3(0.5f,0.5f,0.5f),                                
                                .roughness = 1.f-material.metalness};
                            nc_sbpush(g_materials, material);
                        }
                        // we'll get the glass in later.
                        /*else {
                            // glass
                            sphere_material = make_shared<dielectric>(1.5);
                        }*/

                        sphere={.p = center,.r = 0.2f,.matIndex = newMat};
                        nc_sbpush(g_spheres,sphere);
                    }
                }
            }

            /*
            auto material1 = make_shared<dielectric>(1.5);
            world.add(make_shared<sphere>(V3(0, 1, 0), 1.0, material1));
            */

            unsigned int material2=nc_sbcount(g_materials);
            material={.albedo=V3(0.4, 0.2, 0.1)};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(-4, 0, 1),.r = 1.f,.matIndex = material2};
            nc_sbpush(g_spheres,sphere);

            unsigned int material3=nc_sbcount(g_materials);
            material={.metalness=1.f,.metalColor=V3(0.7, 0.6, 0.5),.roughness=0.f};
            nc_sbpush(g_materials,material);
            sphere={.p = V3(4, 0, 1),.r = 1.f,.matIndex = material3};
            nc_sbpush(g_spheres,sphere);

            //adjust camera.
            //cam.samples_per_pixel = 500;
            //cam.max_depth         = 50;
            //cam.defocus_angle = 0.6;
            c->use_pinhole=false;
            c->target=V3(0,0,0);
            c->pos = V3(13,3,2);
            c->fov=20.f;
            c->focalDistance=10.f;
        }
        break;
    }

    g_world.lights = g_lights;
    g_world.materials = g_materials;
    g_world.planes = g_planes;
    g_world.quads = g_quads;
    g_world.spheres = g_spheres;
    g_world.meshes = g_meshes;
    g_world.rtas = GenerateAccelerationStructure(&g_world);
}

void AddSky(v3 color){
    material_t material={.emitColor = color};
    nc_sbpush(g_materials,material);
}

void AddSunDirectionalLight(){
    // sun directional light.
    //light={.kind = LIGHT_KIND_DIRECTIONAL, .direction = Normalize(V3(1.f,1.f,-1.f)), .radiance = 1.5f *V3(1.f,1.f,1.f)};
    //nc_sbpush(g_lights,light);
    material_t material;
    sphere_t sphere;

    unsigned int light = nc_sbcount(g_materials);
    material={.albedo=V3(0,0,0)/*black body radiation source*/,
        .emitColor=V3(15.f,15.f,15.f)};
    nc_sbpush(g_materials,material);

    sphere={.p = V3(2000, 2000, 2000),.r = 1000.f,.matIndex = light};
    nc_sbpush(g_spheres, sphere);
}

plane_t MakeGroundPlane(unsigned int mat){
    // ground plane.
    plane_t plane={.n = V3(0,0,1),.d = 0, // plane on origin
        .matIndex = mat};
    return plane;
}

void PrintHelp() {

    // print the usage first.
    printf ( "usage: Pathtracer.exe [options]\n" );

    // print description.
    printf ( "\nPhysically-based Path Tracer capable of rendering various geometrical shapes, including triangles.\n" );

    printf ( "Written by Noah J. Cabral.\n\n" );

    printf ( "optional arguments:\n" );

    printf("\tt<int>                        - Set the number of threads to use.\n");
    printf("\tp<int>                        - Set the rays to shoot per pixel.\n");
    printf("\tw<int>                        - Set the world number to load. Possible options:\n"
           "\t\t1:\tDefault scene.\n"
           "\t\t2:\tMetal-roughness test.\n"
           "\t\t3:\tCornell box.\n"
           "\t\t4:\tRay Tracing in One Weekend book cover.\n"
           "\t\t5:\tMario N64 model.\n"
    );
    
    printf("\td                             - Enable depth of field via thin-lens approximation.\n");
    printf("\tn                             - Disable loading normal map textures.\n");
    printf("\tm                             - Disable loading metalness material textures.\n");
    printf("\tr                             - Disable loading roughness material textures.\n");
    printf("\th                             - Print this help menu.\n");
    
}

void ParseArgs() {
    g_tc=MAX_THREAD_COUNT;
    g_sc=8;
    g_pp=4;

    g_bNormals=true;
    g_bMetalness=true;
    g_bRoughness=true;
    g_use_pinhole=true;

    g_worldKind=WORLD_DEFAULT;
    
    for( char **argv=__argv; *argv; argv++ )
        if ( *argv[0]=='-' )
            for( char c; c=*((argv[0])++); ) {
                switch( c ) {
                    case 't': g_tc=max(0,min(MAX_THREAD_COUNT,atoi(argv[0])));
                        break;
                    case 'p': g_pp=max(0,min(RAYS_PER_PIXEL_MAX,atoi(argv[0])));
                        break;
                    case 'n': g_bNormals=false;
                        break;
                    case 'm': g_bMetalness=false;
                        break;
                    case 'r': g_bRoughness=false;
                        break;
                    case 'h': PrintHelp(); exit(0);
                        break;
                    case 'w': g_worldKind=(world_kind_t)max(0,min(WORLD_KIND_COUNT-1,atoi(argv[0])-1));
                        break;
                    case 'd': g_use_pinhole=false;
                        break;
                    case '-':
                        break; // skip but do not give warning.
                    default:
                        printf("Warning: invalid program arugment -%c\n", c);
                        break;
                }
            }
}

void DefineCamera(camera_t *c) {

    // By this point, the "user set" parameters are:
    // pos, use_pinhole, fov, focalDistance, aperatureRadius, and target.

    c->axisZ = Normalize(c->pos - c->target);
    c->axisX = Normalize(Cross(V3(0,0,1), c->axisZ));
    c->axisY = Normalize(Cross(c->axisZ, c->axisX));

    if (!c->use_pinhole)
        c->focalLength=1.f/(1.f/FIXED_FOCAL_LENGTH-1.f/c->focalDistance);
    else
        c->focalLength=FIXED_FOCAL_LENGTH;
    
    c->filmWidth=tan(DEG_TO_RAD*c->fov)*2.f*c->focalLength;
    c->filmHeight=c->filmWidth;
    
    //account for aspect ratio.
    if (g_image.width > g_image.height) {
        c->filmHeight = c->filmWidth * (float)g_image.height / (float)g_image.width;
    } else if (g_image.height > g_image.width) {
        c->filmWidth = c->filmHeight * (float)g_image.width / (float)g_image.height;
    }

    c->halfFilmWidth  = c->filmWidth / 2.0f;
    c->halfFilmHeight = c->filmHeight / 2.0f;

    // the frustrumCenter is offset from the camera in the direction
    // towards the target; i.e. the camera viewing direction.
    c->frustrumCenter = c->pos - c->focalLength * c->axisZ;

    // NOTE: This indeed looks odd at first glance. This is correct. Check the usage.
    // Where it's used, we're working in a stretched film space by factor 2.
    c->halfFilmPixelW = 1.0f / g_image.width;
    c->halfFilmPixelH = 1.0f / g_image.height;

    // print infos.
    {
        printf("DefineCamera():\n===\n");
        printf("camera located at c->pos = (%f,%f,%f)\n", c->pos.x,c->pos.y,
            c->pos.z);
        printf("Distance between the lens and the film plane: %f\n", 
            c->focalLength);
        printf("c->axisX: (%f,%f,%f)\n", c->axisX.x,c->axisX.y,c->axisX.z);
        printf("c->axisY: (%f,%f,%f)\n", c->axisY.x,c->axisY.y,c->axisY.z);
        printf("c->axisZ: (%f,%f,%f)\n", c->axisZ.x,c->axisZ.y,c->axisZ.z);
        printf(
        "The film plane is embedded in the plane defined by c->axisX and c->axisY.\n"
        "Rays are shot originating at the lens located at c->pos and \"strike a sensor on the film to develop the image\".\n"
        "The camera has a local coordinate system which is different from the world coordinate system.\n"
        "The camera is looking down the negative c->axisZ direction.\n\n");
    }
}

// https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
float BurleyParameterization(float roughness){
    return roughness*roughness*roughness*roughness;
}

// from https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html
// p(dir)=cos(theta)/PI.
v3 RandomCosineDirectionHemisphere(){
    float r1 = RandomUnilateral();
    float r2 = RandomUnilateral();

    float phi = 2.f*PI*r1;
    float x = cos(phi)*sqrt(r2);
    float y = sin(phi)*sqrt(r2);
    float z = sqrt(1.f-r2);

    return V3(x, y, z);
}

// https://schuttejoe.github.io/post/ggximportancesamplingpart1/
v3 RandomHalfVectorGGX(float roughness){
    float a2 = BurleyParameterization(roughness);
    float z1 = RandomUnilateral();
    float z2 = RandomUnilateral();

    float theta,phi;
    phi=2.f*PI*z1;
    theta=acos( sqrt( (1.f-z2)/(1.f+z2*(a2-1.f)) ) );

    float x = cos(phi)*sin(theta);
    float y = sin(phi)*sin(theta);
    float z = cos(theta);

    return V3(x, y, z);
}


void BuildOrthonormalBasisFromW(v3 w, v3 *a, v3 *b, v3 *c){
    // NOTE that: X x Y  = Z;
    //            Y x Z  = X;
    //            Z x X  = Y;
    //            Z x Y  = -X;
    //            Z x -X = -Y;
 
    // original code from: https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#orthonormalbases.
    // I haven't analyzed this code precisely, but from a cursory read it feels like it won't work for anistropic pdf's.
    v3 unit_w = Normalize(w);
    bool bWIsGlobalX=fabs(unit_w.x) > 0.9;
    v3 A = (bWIsGlobalX) ? V3(0,1,0) : V3(1,0,0);
    v3 v = Normalize(Cross(unit_w, A));
    v3 u = Cross(unit_w, v);
    *a = u;//x
    *b = v;//y
    *c = unit_w;//z
}

mipchain_t GenerateMipmapChain(texture_t tex){
    assert(tex.width == tex.height);//currently we only support square textures for this kind of thing.
    texture_t *chain=nullptr;
    int level=0;
    int size=tex.width>>1;
    nc_sbpush(chain, tex);
    while(size){
        texture_t tex,parent=chain[level];
        tex.width=tex.height=size;
        tex.data=(v3*)malloc(sizeof(v3)*size*size);
        for (int y=0;y<size;y++)
        for (int x=0;x<size;x++) {
            v2 uv={x*2.f,y*2.f};
            v3 val=SampleTexture(parent,uv);
            tex.data[y*size+x]=val;
        }
        nc_sbpush(chain, tex);
        size=size>>1,level++;
    }
    mipchain_t result={.mips=chain};
    return result;
}

// from https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html#samplinglightsdirectly
// returns V3(0,0,0) if from is inside the sphere or very close to.
v3 RandomToSphere(sphere_t sphere, v3 from) {
    float distance_squared = MagnitudeSquared(from-sphere.p);// Square(from.x-sphere.p.x)+Square(from.y-sphere.p.y)+Square(from.z-sphere.p.z);
    assert(distance_squared>0.f);//responsibility of the caller.

    float term1, term2;
    term1 = 1.f - sphere.r * sphere.r / distance_squared;
    if (term1 < 0.f) return V3(0.f,0.f,0.f); 
    assert(term1 >= 0.f);

    float r1 = RandomUnilateral();
    float r2 = RandomUnilateral();
    float z = 1.f + r2*(sqrt(term1) - 1.f);

    term2 = 1 - z * z;
    assert(term2 >= 0.f);

    float phi = 2*PI*r1;
    float x = cos(phi)*sqrt(term2);
    float y = sin(phi)*sqrt(term2);

    return V3(x, y, z);
}

float RaySphereIntersect(v3 rayOrigin, v3 rayDirection, float minHitDistance, sphere_t sphere, v3 *normal){
    // sphere intersection test.
    float tolerance = TOLERANCE;
    v3 sphereRelativeRayOrigin = rayOrigin - sphere.p;
    float a = Dot(rayDirection, rayDirection);
    float b = 2.0f * Dot(sphereRelativeRayOrigin, rayDirection);
    float c = Dot(sphereRelativeRayOrigin, sphereRelativeRayOrigin) 
        - sphere.r * sphere.r;
    float denom = 2.0f * a;
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant<0.f)return minHitDistance;
    float rootTerm = SquareRoot(discriminant);
    if (rootTerm > tolerance){
        // NOTE: The denominator can never be zero, since we always have a valid direction.
        //also note that despite two roots, we don't need to check which is closer. the minus rootTerm
        //will always be closer, since rootTerm is positive.
        float tn = (-b - rootTerm) / denom;
        float t = tn;
        if ((t > minHitDistance)  ) {
            *normal = Normalize(t*rayDirection + sphereRelativeRayOrigin);
            return t;
        }
    }
    return minHitDistance;
}