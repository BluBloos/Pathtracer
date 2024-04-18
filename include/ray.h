#include "ray_math.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof(array[0]))

#pragma pack(push, 1)
typedef struct bitmap_header {
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

typedef struct image_32 {
    unsigned int width;
    unsigned int height;
    unsigned int *pixelPointer;
} image_32_t;

typedef struct material {
    float scatter; // 0 is pure diffuse, 1 is pure spec
    v3 refColor; /* reflection color */
    v3 emitColor;
} material_t;

// plane equation
// n dot any point on the plane plus d = 0
typedef struct plane {
    v3 n;
    float d; // coeff in plane eq
    unsigned int matIndex;
} plane_t;

typedef struct sphere {
    v3 p;
    float r;
    unsigned int matIndex;
} sphere_t;

typedef struct mesh {
    v3 *points;
    unsigned int pointCount;
    unsigned int matIndex;
} mesh_t;

typedef struct world {
    unsigned int materialCount;
    material *materials;
    unsigned int planeCount;
    plane *planes;
    unsigned int sphereCount;
    sphere *spheres;
    unsigned int aabbCount;
    aabb *aabbs;
    unsigned int meshCount;
    mesh *meshes;
} world_t;