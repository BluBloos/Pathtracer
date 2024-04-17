#include <math.h>
#include <float.h>
#define PI 3.14159265358979323846264338327f
#define TOLERANCE 0.0001f

struct v2 {
    union {
        struct {
            float x,y;
        };
        float E[2];
    };
};

// NOTE(Noah): One of the best c++ features!!!
// Operator overloading!!!
inline v2 operator*(float a, v2 b) {
    v2 result;
    result.x = a * b.x;
    result.y = a * b.y;
    return result;
}

inline v2 operator-(v2 a) {
    v2 result;
    result.x = -a.x;
    result.y = -a.y;
    return result;
}

inline v2 operator+(v2 a, v2 b) {
    v2 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}

inline v2 operator-(v2 a, v2 b) {
    v2 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

struct v3 {
    union {
        struct {
            float x, y, z;
        };
        struct {
            float r, g, b;
        };
    };
};

inline v3 operator+=(v3 a, v3 b) {
    v3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline v3 operator*(float a, v3 b) {
    v3 result;
    result.x = a * b.x;
    result.y = a * b.y;
    result.z = a * b.z;
    return result;
}

inline v3 operator-(v3 a) {
    v3 result;
    result.x = -a.x;
    result.y = -a.y;
    result.z = -a.z;
    return result;
}

inline v3 operator+(v3 a, v3 b) {
    v3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline v3 operator-(v3 a, v3 b) {
    v3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

struct v4 {
    union {
        struct {
            float x,y,z,w;
        };
        struct {
            float r,g,b,a;
        };
    };
};

struct m2 {
    // the column vectors.
    v2 a,b;
};

v2 operator*(m2 m, v2 v)
{
    v2 result;
    result.x = m.a.x * v.x + m.b.x * v.y;
    result.y = m.a.y * v.x + m.b.y * v.y;  
    return result;
}

int Inverse(m2 m, m2 *mOut)
{
    int result=0;

    float det = m.a.x * m.b.y - m.b.x * m.a.y;
    if (det < TOLERANCE && det > -TOLERANCE)
        return -1;

    mOut->a.x = m.b.y / det;
    mOut->b.y = m.a.x / det;
    mOut->b.x = -m.b.x / det;
    mOut->a.y = -m.a.y / det;
    
    return result;
}

// TODO(Noah): replace with intrinsic, probably among other changes in this file!
inline float SquareRoot(float a) {
    float result = (float)sqrt(a);
    return result;
}

inline v3 V3(float x, float y, float z) {
    v3 newv3 = {x, y, z};
    return newv3;
}

inline v4 V4(float x, float y, float z, float w) {
    v4 newv4 = {x,y,z,w};
    return newv4;
}

inline v4 V4(v3 a, float w) {
    v4 newv4 = {a.x, a.y, a.z, w};
    return newv4;
}

inline unsigned int BGRAPack4x8(v4 unpacked) {
    unsigned int result = ((((unsigned int)unpacked.a) << 24) |
                           (((unsigned int)unpacked.r) << 16) | 
                           (((unsigned int)unpacked.g) << 8)  |
                           (((unsigned int)unpacked.b) << 0));
    return result;
}

inline float Magnitude(v3 a) {
    float result;
    result = SquareRoot(a.x * a.x + a.y * a.y + a.z * a.z);
    return result;
}

// TODO(Noah): What if the vector I am trying to normalize
// is (0, 0, 0)? Will get divide error here! this is very very bad!
inline v3 Normalize(v3 a) {
    float magnitude = Magnitude(a);
    a.x /= magnitude;
    a.y /= magnitude;
    a.z /= magnitude;
    return a;
}

inline v3 Cross(v3 a, v3 b) {
    v3 newvec3 = {};
    newvec3.x = a.y * b.z - b.y * a.z;
    newvec3.y = a.z * b.x - b.z * a.x;
    newvec3.z = a.x * b.y - b.x * a.y;
    return newvec3;                                                                                                                                                                                                                                            }

inline float Dot(v3 a, v3 b) {
    float result = a.x * b.x + a.y * b.y + a.z* b.z; 
    return result;
}

inline v3 Hadamard(v3 a, v3 b) {
    v3 newvec3 = {a.x * b.x, a.y * b.y, a.z * b.z};
    return newvec3;
}

// TODO(Noah): replcae this with better entropy later
// return a random value from [0.0, 1.0]
inline float RandomUnilateral() {
    float result = (float)rand() / (float)RAND_MAX;
    return result;
}

// return a random value from [-1.0, 1.0]
inline float RandomBilateral() {
    float result = 2.0f * RandomUnilateral() - 1.0f;
    return result;
}

inline v3 Lerp(v3 a, v3 b, float p) {
    return (1.0f - p) * a + p * b;
}

// the version of linear to sRGB that is presumably correct?
// as opposed to the linear approx version ...
static float LinearToSRGB(float L) {
    if (L < 0.0f) {
        L = 0.0f;
    } 
    if (L > 1.0f) {
        L = 1.0f;
    }
    float S = L * 12.92f;
    if (L > 0.0031308f) {
        S = 1.055f * powf(L, 1.0f/2.4f) - 0.055f;
    }
    return S;
}

static float RayIntersectTri(v3 rayOrigin, v3 rayDirection, float minHit, v3 &A, v3 &B, v3 &C, v3 &N) {
    // Locate the plane that the points are on.
    // Find the point of the ray on the plane.
    // Compute the barycentric coordinates of the plane point.
    // check that they all add to 1 and are all positive, if so, we're in the triangle.

    float denom = Dot(N, rayDirection);
    if ((denom < -TOLERANCE) || (denom > TOLERANCE)) {
        float d = Dot(A, N);
        float t = (d - Dot(N, rayOrigin)) / denom;

        v3 x = rayOrigin + t*rayDirection;
        v3 a = B-A;
        v3 b = C-A;

        m2 mat;
        {
        float e,f,g,h;
        e = Dot(a,a);
        f = g = Dot(b,a);
        h = Dot(b,b);
        mat.a = {e,f};
        mat.b = {g,h};
        }

        m2 matInv;
        if (0!=Inverse(mat,&matInv)) return minHit;

        v2 alphaBetaVector, xHat;
        xHat = { Dot(a,x-A), Dot(b,x-A) };
        alphaBetaVector = matInv * xHat;

        bool bInTri=alphaBetaVector.x>=0.f&&alphaBetaVector.y>=0.f&&(1.f-alphaBetaVector.x-alphaBetaVector.y)>=0.f;
        if (bInTri) {
            return t;
        }       
    }

    return minHit;
}