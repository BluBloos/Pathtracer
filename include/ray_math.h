#include <math.h>
#include <float.h>
#include <assert.h>
#define PI 3.14159265358979323846264338327f
#define TOLERANCE float(1e-9)

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

inline v3 &operator+=(v3 &a, v3 b) {
    a.x = a.x + b.x;
    a.y = a.y + b.y;
    a.z = a.z + b.z;
    return a;
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

/// @brief A struct to define an AABB (axis aligned bounding box).
/// The extent of the box is defined by the origin and halfDim, i.e.
/// xMin=origin.x-halfDim.x, xMax=origin.x+halfDim.x, etc.
///
/// @param origin  is the center of the box.
/// @param halfDim is the half of the dimensions of the box.
/// @param min     is the bottom left corner of the box.
/// @param max     is the top right corner of the box.
typedef struct aabb {
    v3 origin;
    v3 halfDim;
    v3 min;
    v3 max;
    unsigned int matIndex;
    //static aabb_t fromCube(v3 bottomLeft, float width);
    //static aabb_t fromLine(v3 p0, v3 p1);
} aabb_t;

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

inline v2 V2(float x, float y) {
    v2 newv2 = {x, y};
    return newv2;
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

inline float Lerp1f(float a, float b, float t){
    return (1.0f - t) * a + t * b;
}

inline float Smoothstep(float a) {
    return 3.f*a*a-2.f*a*a*a;
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

static float RayIntersectPlane(v3 rayOrigin, v3 rayDirection, v3 N, float d, float minHit) {
    float denom = Dot(N, rayDirection);
    if ((denom < -TOLERANCE) || (denom > TOLERANCE)) {
        float t = (d - Dot(N, rayOrigin)) / denom;
        return t;
    }
    return minHit;
}

static float Square(float a){
    return a*a;
}

static float RayIntersectTri(v3 rayOrigin, v3 rayDirection, float minHit, v3 &A, v3 &B, v3 &C, v3 &N) {
    // Locate the plane that the points are on.
    // Find the point of the ray on the plane.
    // Compute the barycentric coordinates of the plane point.
    // check that they all add to 1 and are all positive, if so, we're in the triangle.

        float d = Dot(A, N);

    float t=RayIntersectPlane(rayOrigin,rayDirection,N,d, minHit);
    if(t!=minHit) {

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


// NOTE: This was stolen from the Atomation source code.
//
/* How it works:

 find all planes that are "facing us". should at most be three.
 intersect ray with these.
 check if point is in bounds.

 if the ray somehow intersects two planes (hits precisely vertex of AABB),
 then deterministically give back some Idx to ensure no visual jitter frame-frame
 if a consumer were to use faceHitIdx to render something. this is to say given a constant
 ray, the result of this function should be temporally stable.

 return minHitDistance if there was no hit, otherwise it returns the hit distance.
*/
float doesRayIntersectWithAABB2(
    const v3 &rayOrigin, const v3 &rayDir, float minHitDistance,
    const aabb_t &candidateBox, bool *exitedEarly, int*faceHitIdx
) {
    assert(rayDir.x != 0.f || rayDir.y != 0.f || rayDir.z != 0.f);

    int potentialFaces[6]={};  //stores face Idx's.
    int faceCount=0;

    // find faces.
    constexpr int aabbFaceCount=6;
    constexpr v3 faceNormals[] = {
        // front, back, left, right, top, bottom.
        {0.f,0.f,-1.f}, {0.f,0.f,1.f}, {-1.f,0.f,0.f}, {1.f,0.f,0.f}, {0.f,1.f,0.f}, {0.f,-1.f,0.f}
    };
    assert(sizeof(faceNormals)/sizeof(v3)==aabbFaceCount);
    for (int i=0;i<aabbFaceCount;i++){
        float d = Dot(rayDir, faceNormals[i]);
       // if (d<0) 
        {
            potentialFaces[faceCount++]=i;
        }
    }
    //assert(faceCount<=3);
    assert(aabbFaceCount==6);

    for (int i=0;i<faceCount;i++) {
        float planeCord;
        auto checkPointInBounds = [&](const v3 &p) {
            return p.x >= candidateBox.min.x && p.x <= candidateBox.max.x &&
                    p.y >= candidateBox.min.y && p.y <= candidateBox.max.y &&
                    p.z >= candidateBox.min.z && p.z <= candidateBox.max.z;
        };
        const int j=potentialFaces[i];
        switch(j) {
            case 0: // front, back: (which Z).
            case 1:
            {
                planeCord = (j==0)?candidateBox.min.z:candidateBox.max.z;
                if (rayDir.z==0.f) continue;
                float tHit = (planeCord - rayOrigin.z) / rayDir.z;
                if (tHit<0.f) continue;
                float y=rayOrigin.y + rayDir.y * tHit;
                float x=rayOrigin.x + rayDir.x * tHit;
                if (checkPointInBounds(v3{x,y,planeCord})) {
                    if (faceHitIdx) *faceHitIdx=potentialFaces[i];
                    return tHit;
                }
            }
            break;
            case 2: // left, right: (which X).
            case 3:
            {
                planeCord = (j==2)?candidateBox.min.x:candidateBox.max.x;
                if (rayDir.x==0.f) continue;
                float tHit = (planeCord - rayOrigin.x) / rayDir.x;
                if (tHit<0.f) continue;
                float y=rayOrigin.y + rayDir.y * tHit;
                float z=rayOrigin.z + rayDir.z * tHit;
                if (checkPointInBounds(v3{planeCord,y,z})) {
                    if (faceHitIdx) *faceHitIdx=potentialFaces[i];
                    return tHit;
                }
            }
            break;
            case 4: // top, bottom: (which Y).
            case 5:
            {
                planeCord = (j==4)?candidateBox.max.y:candidateBox.min.y;
                if (rayDir.y==0.f) continue;
                float tHit = (planeCord - rayOrigin.y) / rayDir.y;
                if (tHit<0.f) continue;
                float x=rayOrigin.x + rayDir.x * tHit;
                float z=rayOrigin.z + rayDir.z * tHit;
                if (checkPointInBounds(v3{x,planeCord,z})) {
                    if (faceHitIdx) *faceHitIdx=potentialFaces[i];
                    return tHit;
                }
            }
            break;
        }
    }

    return minHitDistance;
}

static aabb_t MakeAABB(v3 origin, v3 halfDim)
{
    aabb_t r = {};
    r.origin = origin;
    r.halfDim = halfDim;
    r.min = origin - halfDim;
    r.max = origin + halfDim;
    return r;
}

static aabb_t AABBFromCube(v3 bottomLeft, float width)
{
    const v3 halfDim = { width / 2.0f, width / 2.0f, width / 2.0f };
    const v3 origin = bottomLeft + halfDim;
    return MakeAABB(origin, halfDim);
}