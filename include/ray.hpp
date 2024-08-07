#include "ray_math.hpp"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof(array[0]))

#pragma pack(push, 1)
typedef struct {
	unsigned short FileType;          /* File type, always 4D42h ("BM") */
	unsigned int FileSize;            /* Size of the file in bytes */
	unsigned short Reserved1;         /* Always 0 */
	unsigned short Reserved2;         /* Always 0 */
	unsigned int BitmapOffset;        /* Starting position of image data in bytes */
	unsigned int size;                /* Size of this header in bytes */
	int Width;                        /* Image width in pixels */
	int Height;                       /* Image height in pixels */
	unsigned short Planes;            /* Number of color planes */
	unsigned short BitsPerPixel;      /* Number of bits per pixel */
    unsigned int biCompression;       /* specfies type of compression to be used */
    unsigned int biSizeImage;         /* can be set to 0 for uncompressed RGB bitmaps */
    int biXPelsPerMeter;              /* horizontal resolution in pixels per meter */
    int biYPelsPerMeter;              /* vertical resoltion in pixels per meter */
    unsigned int biClrUsed;           /* specifies color indices used in color table */
    unsigned int biClrImportant;      /* specifies color indices that are important */
    char rgbBlue;
    char rgbGreen;
    char rgbRed;
    char rgbReserved;
} bitmap_header_t;
#pragma pack(pop)

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int *pixelPointer;
} image_32_t;

typedef struct material {
    
    // Depending on the interface, we can determine how much was refracted and how much was reflected.
    // i.e. 
    /*
    float kS = calculateSpecularComponent(...); // reflection/specular fraction
    float kD = 1.0 - kS;                        // refraction/diffuse  fraction
    */
    //
    // it's also true that depending on the incoming angle, the amount reflect/refract will change. 
    // this arises from the Fresnel equation.

    // depending on the "microfacets" (the geometry at small physical scale), there might be multiple specular bounces
    // of the light before it "exists" the surface. this leads to attenuation and therefore an affect called
    // "self-shadowing".

    // spectral rendering is where we consider more than just three wavelengths of light.
    // there are physical effects that can vary quite intensely for different wavelengths.

    // 0 is pure diffuse, 1 is pure specular.
    // "diffuse" is referring to the physical phenomenon where the light refracts into the surface briefly before
    // emerging out of the surface in the reflection direction. it's an approximation to ignore the emergent offset
    // between the enter and exit locations. Subsurface scattering is a technique that takes this offset into account.
    //
    // "specular" is therefore referring to the light that hit the surface and was reflected, never refracting into it.
    //
    // we use the "roughness" term to model how rough the microfacet surface is.
    float alpha=1.f;
    float ior=1.f;

    int albedoIdx=0;
    v3 albedo; // if there is no albedo texture.

    int metalnessIdx=0;
    float metalness=0.f;
    v3 metalColor;

    int roughnessIdx=0;
    float roughness=1.f;

    int normalIdx=0;
    
    v3 emitColor;
} material_t;

// plane equation
// n dot any point on the plane plus d = 0
typedef struct {
    v3 n;
    float d; // coeff in plane eq
    unsigned int matIndex;
} plane_t;

typedef struct {
    v3 point;
    v3 u;
    v3 v;
    unsigned int matIndex;
} quad_t;

typedef struct {
    v3 p;
    float r;
    unsigned int matIndex;
} sphere_t;

typedef struct {
    v3 *points;
    int *matIndices;
    unsigned int pointCount;
} mesh_t;

typedef struct rtas_node {
    int *triangles;
    int triangleCount;
    rtas_node *children;
    aabb_t bounds;
} rtas_node_t;

typedef struct {
    int width;
    int height;
    unsigned int xPos;
    unsigned int yPos;
} texel_t;

typedef enum {
    LIGHT_KIND_DIRECTIONAL,
    LIGHT_KIND_POINT,
    LIGHT_KIND_TRIANGLE
} light_kind_t;

typedef struct {
    light_kind_t kind;
    union {
        v3 direction;//directional light.
        v3 position;//point light.
    };
    v3 radiance;
} light_t;

typedef struct {
    float hitDistance;
    unsigned int hitMatIndex;
    v3 normal;
} ray_payload_t;

typedef struct {
    int width, height;
    v3 *data;
} texture_t;

typedef struct {
    texture_t *mips;//levels go 0,1,...
} mipchain_t;

typedef struct {
    // all are stretchy buffers.
    light_t *lights;
    material_t *materials;
    plane_t *planes;
    quad_t *quads;
    sphere_t *spheres;
    aabb_t *aabbs;
    mesh_t *meshes;
    rtas_node_t rtas;
} world_t;

typedef enum world_kind {
    WORLD_DEFAULT,
    WORLD_BRDF_TEST,//metal-roughness test.
    WORLD_CORNELL_BOX,
    // WORLD_GLTF, /* we cannot support just yet due to required advancements in acceleration structure */
    WORLD_RAYTRACING_ONE_WEEKEND,
    WORLD_MARIO,
    // WORLD_SUN_TEMPLE, /* we cannot support just yet due to required advancements in acceleration structure */
    WORLD_KIND_COUNT
} world_kind_t;

// Vulkan-style API :)
typedef struct camera {
    float fov;
    float focalLength;//physical camera property.
    float focalDistance;//virtual image plane where the image is in focus.
    float aperatureRadius;
    bool use_pinhole;
    // film plane properties.
    float filmWidth, filmHeight, halfFilmWidth, halfFilmHeight, 
        halfFilmPixelW, halfFilmPixelH;
    v3 pos,target,frustrumCenter,axisZ,axisX,axisY;
} camera_t;