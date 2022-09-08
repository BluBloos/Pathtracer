#define INTERNAL static

#include "ray_math.h"

#define ArrayCount(array) (sizeof(array) / sizeof(array[0]))

#pragma pack(push, 1)
struct bitmap_header
{
	unsigned short FileType;     /* File type, always 4D42h ("BM") */
	unsigned int FileSize;     /* Size of the file in bytes */
	unsigned short Reserved1;    /* Always 0 */
	unsigned short Reserved2;    /* Always 0 */
	unsigned int BitmapOffset; /* Starting position of image data in bytes */
	unsigned int size;            /* Size of this header in bytes */
	int Width;           /* Image width in pixels */
	int Height;          /* Image height in pixels */
	unsigned short Planes;          /* Number of color planes */
	unsigned short BitsPerPixel;    /* Number of bits per pixel */
    unsigned int biCompression; //specfies type of compression to be used
    unsigned int biSizeImage; //can be set to 0 for uncompressed RGB bitmaps
    int biXPelsPerMeter; //horizontal resolution in pixels per meter
    int biYPelsPerMeter; //vertical resoltion in pixels per meter
    unsigned int biClrUsed; //specifies color indices used in color table
    unsigned int biClrImportant; //specifies color indices that are important
    char rgbBlue;
    char rgbGreen;
    char rgbRed;
    char rgbReserved;
};
#pragma pack(pop)

struct image_32
{
    unsigned int width;
    unsigned int height;
    unsigned int *pixelPointer;
};


//TODO(Noah): Change these! Maybe.
struct material
{
    float scatter; //0 is pure diffuse, 1 is pure spec
    v3 refColor;
    v3 emitColor;
};

struct plane
{
    v3 n;
    float d;
    unsigned int matIndex;
};

struct sphere
{
    v3 p;
    float r;
    unsigned int matIndex;
};

struct world
{
    unsigned int materialCount;
    material *materials;
    
    unsigned int planeCount;
    plane *planes;
    
    unsigned int sphereCount;
    sphere *spheres;
};