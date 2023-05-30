
RWTexture2D<float4> gpuTex : register(u0); // maps to 0th UAV register.
RWTexture2D<float4> cpuTex : register(u1); // maps to 1st UAV register.



// so, when we div_ceil on the numthreads number, does that produce
// a "leftover" threadgroup where the occupancy is like, not full?
//
[numthreads(16, 16, 1)]
void copy_shader(uint3 DTid : SV_DispatchThreadID)
{	
    cpuTex[DTid.xy] = gpuTex.Load(DTid.xy);
    // swizzle the colors because our output surface expects so.
    cpuTex[DTid.xy].rgba = cpuTex[DTid.xy].bgra;
}


struct MyPayload
{
  float3 color;
  float3 attenuation;
  uint currBounce;
};

struct MyAttributes
{
  float3 nextNormal;
  uint hitMatIndex;
};

struct material_t
{
  float3 emitColor;
  float scatter;
  float3 refColor;
  int padding;
};

struct plane_t
{
  float d;
  uint matIndex;
  float2 padding;
  float3 n;
  float padding2;
};

struct sphere_t
{
  float r;
  unsigned int matIndex;
  float2 padding;
};

struct random_pair_t
{
  float2 rand;
  float2 padding;
};


// this is bound via a constant buffer view.
cbuffer WorldConstantBuffer : register(b0)
{
  // TODO: we want to pull this constant 10 from somewhere common.
  material_t world_materials[10];
  plane_t    world_planes[10];
  sphere_t   world_spheres[10];
  random_pair_t     rands[256]; 
};

// these are bound via constant root params.
cbuffer TexelConstantBuffer : register(b0, space1)
{
    int texelX;
    int texelY;
    int randSeed;
    uint image_width;
    uint image_height;
};

// raw buffer SRV.
RaytracingAccelerationStructure MyScene : register(t0);


float RandomUnilateral(uint2 n)
{
  return frac(sin(dot(randSeed*n, float2(12.9898, 4.1414))) * 43758.5453);
}

float RandomBilateral(uint2 n)//-1 -> 1
{
  return 2.f*(RandomUnilateral(n)-0.5f);
}


// some magic sauce. TODO: this can be much better anyways.
void LinearToSRGB(inout float L)
{
  if (L < 0.0f) {
        L = 0.0f;
    } 
  // TODO: shouldn't we just let things accumulate in HDR space??
  // it looks like currently our accumulation can go over 1,
  // then we just clamp that .... losing all information!?
  // I think maybe we need to do some sort of tonemapping from HDR to linear.
  if (L > 1.0f) {
        L = 1.0f;
    }
  float S = L * 12.92f;
  if (L > 0.0031308f) {
    S = 1.055f * pow(L, 1.0f/2.4f) - 0.055f;
  }
  L=S;
}
void LinearToSRGB(inout float3 L)
{
  LinearToSRGB(L.x);
  LinearToSRGB(L.y);
  LinearToSRGB(L.z);
}


[shader("raygeneration")]
void ray_gen_shader()
{
  const uint constRayFlags =         RAY_FLAG_FORCE_OPAQUE;

  uint3 rayIndex=DispatchRaysIndex();

  uint2 xy = rayIndex.xy + uint2(texelX,texelY);
  float x = xy.x;
  float y = xy.y;
  
  // 256 rays per pixel.
  const  int raysPerPixel=256;


  // TODO: there is a bunch of crap here that can move to CPU side
  // compute, or that are constants and only need to be compute once.
  

  // NOTE: the rays shoot at the film, from cameraP. the film is away from cameraP in -cameraZ dir.
  // therefore, rays are shooting towards (0,0,0).
  const float3 cameraP = float3(0, 1, -10); // put the camera in a place.
  const float3 cameraZ = normalize(cameraP); // away from (0,0,0).
  const float3 cameraX = normalize(cross(float3(0,1,0), cameraZ));
  const float3 cameraY = normalize(cross(cameraZ, cameraX));



  float halfPixW = 1.0f / image_width;
  float halfPixH = 1.0f / image_height;

  // compute the physical film dimensions.
  float filmH = 1;
  float filmW = 1;
  if (image_width > image_height) {
    filmH = filmW * (float)image_height / (float)image_width;
  } else if (image_height > image_width) {
    filmW = filmH * (float)image_width / (float)image_height;
  }
  float halfFilmW = filmW / 2.0f;
  float halfFilmH = filmH / 2.0f;

  // compute physical location of pixel on the film, normalized to -1 -> 1.
  float filmY = -1.0f + 2.0f * y / (float)image_height;
  float filmX = -1.0f + 2.0f * x / (float)image_width;


  const float filmDist = 1;
  float3 filmCenter = cameraP - filmDist * cameraZ;


  float3 color = float3(0,0,0);

  float contrib = 1.0f / (float)raysPerPixel;
  
  for (int i = 0; i < raysPerPixel; i++) {

    float offX = filmX + (rands[i].rand.x * halfPixW);
    float offY = filmY + (rands[i].rand.y * halfPixH);
    float3 filmP = filmCenter + (offX * halfFilmW * cameraX) + (offY * halfFilmH * cameraY);
    float3 rayOrigin = cameraP;
    float3 rayDirection = normalize(filmP - cameraP);

    RayDesc r;
    r.Origin=rayOrigin;
    r.Direction=rayDirection;
    r.TMax=100;//TODO:
    r.TMin=0;
    MyPayload p;
    p.color = float3(0,0,0);
    p.attenuation = float3(1,1,1);
    p.currBounce=0;
    TraceRay(
             MyScene,
             constRayFlags,

             0xFF, //8 bit InstanceMask.
             // NOTE: this is just where this guy & the mask on the actual tlas instance must be nonzero,
             // then we don't ignore.
             //
             //NOTE: the only thing that contributes to the hit group should be what instance it is.
             0,//RayContributionToHitGroupIndex
             0,//MultiplierForGeometryContributionToShaderIndex
             0, // MissShaderIndex, NOTE: here we use index for the single miss shader that we have.
             r, p
             );
    color = color + contrib * p.color;
  }

  LinearToSRGB(color);

//NOTE: we won't be accumulating in the alpha part of the color,
// so default it to 0xFF
  gpuTex[ xy ] = float4(color,1);
}


[shader("miss")]
void miss_main(inout MyPayload payload)
{
  payload.color += float3(
                          0,
                          float(0x82)/256.f,
                          float(0xF0)/256.f
                          ) * payload.attenuation;
}

[shader("intersection")]
void intersection_sphere()
{

  //TODO: seems we might want to use Tmin or something here.
  const   float minHitDistance = 0.001f;
  const float tolerance = 0.0001f;

  uint id = InstanceID(); //user provided value for instance id of blas in tlas.

  MyAttributes attr;//user defined attribute for the hit.
  
  sphere_t sphere = world_spheres[id];

  //  float3 sphereRelativeRayOrigin = rayOrigin - sphere.p;
  float3 sphereRelativeRayOrigin = ObjectRayOrigin();
  float3 rayDirection = WorldRayDirection();
 
  float a = dot(rayDirection, rayDirection);
  float b = 2.0f * dot(sphereRelativeRayOrigin, rayDirection);
  float c = dot(sphereRelativeRayOrigin, sphereRelativeRayOrigin) 
    - sphere.r * sphere.r;
  float denom = 2.0f * a;
  float rootTerm = sqrt(b * b - 4.0f * a * c);
  
  if (rootTerm > tolerance) {
    // NOTE(Noah): The denominator can never be zero.
    // that is, so long as rayDir is non-zero, but how ridiculous would it be if it was!??
    float tp = (-b + rootTerm) / denom;
    float tn = (-b - rootTerm) / denom;   
    float t = tp;
    if ((tn > minHitDistance) && (tn < tp)){
      t = tn;
    }
    if ((t > minHitDistance)
        //NOTE: this condition is copy pasta from the CPU side app.
          // it is there because we need to compare the curr object to the current Tmax,
          // to ensure that we are looking at the closest hit.
          // however, dxr has us covered in this case.
          //&& (t < hitDistance)
        ) {
      attr.hitMatIndex = sphere.matIndex;
      attr.nextNormal = normalize(t*rayDirection + sphereRelativeRayOrigin);

  ReportHit(
             t, // new THit
              0, //hitkind
              attr);
    }
  }

}



[shader("intersection")]
void intersection_plane()
{
  const float tolerance = 0.0001f;

  //TODO: seems we might want to use Tmin or something here.
  const   float minHitDistance = 0.001f;

  uint id = InstanceID(); //user provided value for instance id of blas in tlas.

  float THit = RayTCurrent();

  float3 rayOrigin= WorldRayOrigin();//world space origin for the current ray;
  float3 rayDirection=WorldRayDirection();
  
  MyAttributes attr;//user defined attribute for the hit.

  
  // check hit.
  plane_t plane = world_planes[id];
  float denom = dot(plane.n, rayDirection);
  if ((denom < -tolerance) || (denom > tolerance))
    {
      float t = (-plane.d - dot(plane.n, rayOrigin)) / denom;
      if ((t > minHitDistance)
          //NOTE: this condition is copy pasta from the CPU side app.
          // it is there because we need to compare the curr object to the current Tmax,
          // to ensure that we are looking at the closest hit.
          // however, dxr has us covered in this case.
          //&& (t < hitDistance)
          )
        {
          attr.hitMatIndex = plane.matIndex;
          attr.nextNormal = plane.n;
           ReportHit(
              t, // new THit
              0, //hitkind
              attr);
        }
    }
  
  
}


[shader("closesthit")]
void closesthit_main(inout MyPayload payload, in MyAttributes attr)
{

  const uint constRayFlags =         RAY_FLAG_FORCE_OPAQUE;
  uint3 rayIndex=DispatchRaysIndex();

  uint2 xy = rayIndex.xy + uint2(texelX,texelY);
  
  material_t mat = world_materials[attr.hitMatIndex];

  // make the contribution!!!!!!!!!!!!
  payload.color += payload.attenuation* mat.emitColor;
  payload.attenuation = payload.attenuation* mat.refColor;            
  payload.currBounce+=1;
  if (payload.currBounce > 8) return;

  // compute the bounce, new origin and direction.
  float3 rayOrigin = WorldRayOrigin();
  float3 rayDirection=WorldRayDirection();
  rayOrigin = rayOrigin + RayTCurrent() * rayDirection;
  float3 pureBounce = 
    rayDirection - 2.0f * dot(attr.nextNormal, rayDirection) * attr.nextNormal;
  float3 randomBounce = normalize(
                              attr.nextNormal + float3(
                                              RandomBilateral(xy),
                                              RandomBilateral(xy),
                                              RandomBilateral(xy)
                                                       ));
  // NOTE: scatter also needs to be a vector3.
  rayDirection = normalize(lerp(randomBounce, pureBounce, mat.scatter));

  RayDesc reflectedRay;
  reflectedRay.Origin=rayOrigin;
  reflectedRay.Direction=rayDirection;
  reflectedRay.TMax=100;//TODO:
  reflectedRay.TMin=0;//TODO.

  
  TraceRay(MyScene,
           constRayFlags,
           0xFF,
           //contrib idx's
           0,
           0,
           0,//miss shader idx.
           reflectedRay,
           payload);
  
}
