
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
  float4 color;
  float4 attenuation;
};

struct MyAttributes
{
  float hitDistance;
  float3 nextNormal;
  uint hitMatIndex;
};

struct material_t
{
  float4 emitColor;
  float4 scatter;
  float4 refColor;
};

struct plane_t
{
  float d;
  uint matIndex;
  float3 n; // starts on next float4 boundary.
};


// this is bound via a constant buffer view.
cbuffer WorldConstantBuffer : register(b0)
{
  // NOTE: for now, these are fixed size arrays for simplicity.
  material_t world_materials[1];
  plane_t    world_planes[1];
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
    if (L > 1.0f) {
        L = 1.0f;
    }
    float S = L * 12.92f;
    if (L > 0.0031308f) {
        S = 1.055f * pow(L, 1.0f/2.4f) - 0.055f;
    }
    L=S;
}
void LinearToSRGB(inout float4 L)
{
  LinearToSRGB(L.x);
  LinearToSRGB(L.y);
  LinearToSRGB(L.z);
  LinearToSRGB(L.w);
}


[shader("raygeneration")]
void ray_gen_shader()
{
  // TODO: for now we are using these flags to "cull the miss shader" as the DXR docs put it.
  // this should visually result in the silhouttes of objects.
  //
  //  const uint constRayFlags =         RAY_FLAG_FORCE_OPAQUE|RAY_FLAG_SKIP_TRIANGLES;
  const uint constRayFlags =         RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER;


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
  const float3 cameraP = float3(10, 10, 10); // put the camera in a place.
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


  float4 color = float4(0,0,0,
                        //NOTE: we won't be accumulating in the alpha part of the color,
                        // so default it to 0xFF
                        1);
  float contrib = 1.0f / (float)raysPerPixel;
  
  for (int i = 0; i < raysPerPixel; i++) {

    float offX = filmX + (RandomBilateral(xy) * halfPixW);
    float offY = filmY + (RandomBilateral(xy) * halfPixH);
    float3 filmP = filmCenter + (offX * halfFilmW * cameraX) + (offY * halfFilmH * cameraY);
    float3 rayOrigin = cameraP;
    float3 rayDirection = normalize(filmP - cameraP);

    RayDesc r;
    r.Origin=rayOrigin;
    r.Direction=rayDirection;
    r.TMax=100;//TODO:
    r.TMin=0;
    MyPayload p;
    p.color = float4(0,0,0,0);
    p.attenuation = float4(1,1,1,1);
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
    // TODO: shouldn't we just let things accumulate in HDR space??
    color = color + contrib * p.color;
  }

  LinearToSRGB(color);
  
  gpuTex[ xy ] = color;
}


[shader("miss")]
void miss_main(inout MyPayload payload)
{
  //TODO: need to factor in the attenuation.
  payload.color += float4(
                          0,
                          float(0x82)/256.f,
                          float(0xF0)/256.f,
                          0
                          );
}


//TODO: we'll also want the sphere intersection idea.
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

  bool hit =false;

  
  // check hit.
  plane_t plane = world_planes[id];
  float denom = dot(plane.n, rayDirection);
  if ((denom < -tolerance) || (denom > tolerance))
    {
      float t = (-plane.d - dot(plane.n, rayOrigin)) / denom;
      if ((t > minHitDistance)
          //NOTE: so I'm getting rid of this condition because it seems super
          // odd for us to not mark the hit if this ray was larger than the last
          // (where this logic was copy pasta from CPU side app).
          //&& (t < hitDistance)
          )
        {
          attr.hitDistance = t;
          attr.hitMatIndex = plane.matIndex;
          attr.nextNormal = plane.n;
          hit=true;
        }
    }
  
  if(hit)
    ReportHit(
              attr.hitDistance, // new THit
              0, //hitkind
              attr);
  
}

 /*
[shader("closesthit")]
void closesthit_main(inout MyPayload payload, in MyAttributes attr)
{

  material_t mat = world_materials[attr.hitMatIndex];

  // make the contribution!!!!!!!!!!!!
  payload.color = payload.result + Hadamard(payload.attenuation, mat.emitColor);
  payload.attenuation = Hadamard(payload.attenuation, mat.refColor);            

  // compute the bounce, new origin and direction.
  float3 rayOrigin=WorldRayOrigin();
  float3 rayDirection=WorldRayDirection();
  rayOrigin = rayOrigin + attr.hitDistance * rayDirection;
  float3 pureBounce = 
    rayDirection - 2.0f * dot(attr.nextNormal, rayDirection) * attr.nextNormal;
  float3 randomBounce = normalize(
                              attr.nextNormal + float3(
                                              RandomBilateral(),
                                              RandomBilateral(),
                                              RandomBilateral()
                                                       ));
  // NOTE: scatter also needs to be a vector3.
  rayDirection = normalize(lerp(randomBounce, pureBounce, mat.scatter));

  RayDesc r={};
  r.Origin=rayOrigin;
  r.Direction=rayDirection;
  r.TMax=100;//TODO:
                              
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
*/
