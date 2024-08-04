int ParseArgs();
void PrintHelp();
void PrintUsage();

typedef unsigned int color_t;

typedef struct {
    double r;
    double g;
    double b;
    double a;
} v4_t;

double CompareImages(
    color_t *data, int width, int height,
    color_t *data2, int width2, int height2);

const char *g_filepath1;
const char *g_filepath2;

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define FALSE 0

#define THRESHOLD 1e6

int main(int argc, char **argv)
{
    int result = 0;

    int got_positionals = ParseArgs(argc, argv);
    if (!got_positionals) PrintUsage(), exit(1);

    do {
        int x1,y1,n1;
        color_t *data1 = (color_t *)stbi_load(
            g_filepath1, &x1, &y1, &n1, 4);

        if ( data1 == NULL) {
            printf("Error: '%s' is an invalid file path\n", g_filepath1);
            result = -1;
            break;
        }

        int x2,y2,n2;
        color_t *data2 = (color_t *)stbi_load(
            g_filepath2, &x2, &y2, &n2, 4);

        if ( data2 == NULL ) {
            printf("Error: '%s' is an invalid file path\n", g_filepath2);
            result = -1;
            break;
        }

        if ( x1 != x2 ) {
            printf("Error: Images must have same width but they are %d and %d\n", 
                x1, x2);
            result = -1;
            break;
        }

        if ( y1 != y2 ) {
            printf("Error: Images must have same height but they are %d and %d\n", 
                y1, y2);
            result = -1;
            break;
        }

        double s = CompareImages( data1, x1, y1, data2, x2, y2 );

        printf("Percentage Similarity: %f %%\n", s);
        
    } while(FALSE);

    return result;
}

double CompareImages(
    color_t *data, int width, int height,
    color_t *data2, int width2, int height2)
{
    double ColorDistance(color_t, color_t);

    double something, N;

    something = 0;
    N = width * height;

    for ( int y = 0; y < height; y++)
    for ( int x = 0; x < width;  x++)
    {
        color_t a,b;
        a = data[y*width+x];
        b = data2[y*width+x];
        double distance = ColorDistance(a,b); 
        something += 1.0 - distance;
    }

    double percentsimilarity = something / N * 100.0;

    return percentsimilarity;
}

double ColorDistance(color_t a, color_t b)
{
    // for now pretend that all colors are the same.

    v4_t VectorSub(v4_t, v4_t);
    double Magnitude(v4_t);
    v4_t ExtractColorVector (color_t);

    v4_t v1 = ExtractColorVector( a );
    v4_t v2 = ExtractColorVector( b );

    v4_t vdiff = VectorSub( v1, v2 );

    return Magnitude( vdiff );
}

double Magnitude(v4_t v)
{
    return sqrt(v.a * v.a + v.b * v.b + v.g * v.g + v.a * v.a);
}

v4_t ExtractColorVector (color_t c)
{
    v4_t v;
    v.r = (c & 0xFF) / 255.0;
    v.g = ((c >> 8) & 0xFF) / 255.0;
    v.b = ((c >> 16) & 0xFF) / 255.0;
    v.a = ((c >> 24) & 0xFF) / 255.0;
    return v;
}

v4_t VectorSub(v4_t a, v4_t b)
{
    v4_t v;
    v.r = a.r - b.r;
    v.g = a.g - b.g;
    v.b = a.b - b.b;
    v.a = a.a - b.a;
    return v;
}

void PrintUsage()
{
    printf ( "usage: ImageCompare.exe image_file1 image_file2 [options]\n" );
}


void PrintHelp() {

    PrintUsage();

    // print description.
    printf ( "\nCompare two images and print Perctange Similarity.\n" );

    printf ( "Written by Noah J. Cabral.\n\n" );

    printf ( "optional arguments:\n" );

    printf("\th                             - Print this help menu.\n");
    
}

int ParseArgs(int argc, char **argv)
{
    int required_positionals = 3;
    int n = 0;

    for( ; *argv; argv++ )
        if ( *argv[0]=='-' )
            for( char c; c=*((argv[0])++); ) {
                switch( c ) {
                    case 'h':
                     PrintHelp(); exit(0);
                        break;
                    case '-':
                        break; // skip but do not give warning.
                    default:
                        printf("Warning: invalid program arugment -%c\n", c);
                        break;
                }
            }
        else {
            switch(n) {
                case 0: /* Do nothing as this is the .exe path */ 
                    break;
                case 1: g_filepath1 = argv[0]; break;
                case 2: g_filepath2 = argv[0]; break; 
            }
            n++;
        }

    return required_positionals <= n;
}