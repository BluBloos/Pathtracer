#include "ray_math.h"

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

    // metals (conductors) have special properties w.r.t. light.
    // when the EM wave arrives at a conductor interface, the wave goes to zero quite quickly at a characteristic
    // length-scale called the skin-depth.
    // (effectively, is completely absorbed and there is no scattering).
    
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
    float roughness=1.f;
    float alpha=1.f;
    float refractionIndex=1.f;

    int albedoIdx=0;
    v3 albedo; // if there is no albedo texture.
    
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
        v3 location;//point light.
    };
    v3 radiance;
} light_t;

typedef struct {
    float hitDistance;
    unsigned int hitMatIndex;
    v3 normal;
} ray_payload_t;

typedef struct {
    unsigned int lightCount;
    light_t *lights;
    unsigned int materialCount;
    material_t *materials;
    unsigned int planeCount;
    plane_t *planes;
    unsigned int sphereCount;
    sphere_t *spheres;
    unsigned int aabbCount;
    aabb_t *aabbs;
    unsigned int meshCount;
    mesh_t *meshes;
    rtas_node_t rtas;
} world_t;