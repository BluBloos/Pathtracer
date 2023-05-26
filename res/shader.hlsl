
RWTexture2D<float4> gpuTex : register(u0); // maps to 0th UAV register.
RWTexture2D<float4> cpuTex : register(u1); // maps to 1st UAV register.



// so, when we div_ceil on the numthreads number, does that produce
// a "leftover" threadgroup where the occupancy is like, not full?
//
[numthreads(16, 16, 1)]
void copy_shader(uint3 DTid : SV_DispatchThreadID)
{	
    //cpuTex[DTid.xy] = gpuTex.Load(DTid.xy);
    // for now, we just output a pure red.	
    cpuTex[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0);

    // swizzle the colors because our output surface expects so.
    cpuTex[DTid.xy].rgba = cpuTex[DTid.xy].bgra;
}


struct MyPayload
{
  float3 color;
  float3 attenuation;
};

struct MyAttributes
{
  float hitDistance;
  float3 nextNormal;
  uint hitMatIndex;
};

struct material_t
{
  float3 emitColor;
  float3 scatter;
  float3 refColor;
};

struct plane_t
{
  float3 n;
  float3 d;
  uint matIndex;
};

// NOTE: for now these are fixed size arrays for simplicity.
struct world
{
  material_t materials[1];
  plane_t    planes[1];
};
  


// raw buffer SRV.
RaytracingAccelerationStructure MyScene[] : register(t0);

// TODO: 
HitGroup my_group_name = 
{ 
  "intersection_main", 
  "", 
  "closesthit_main"
};

// NOTE: the payload and intersection shader attr are user defined structures.
// TODO: 
RaytracingShaderConfig shader_config_name = 
{
    maxPayloadSizeInBytes,
    maxAttributeSizeInBytes
};

// TODO: 
RaytracingPipelineConfig config_name = 
{
    8//maxTraceRecursionDepth
};


float RandomBilateral()
{
  return noise();
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
void LinearToSRGB(inout float3 L)
{
  LinearToSRGB(L.x);
  LinearToSRGB(L.y);
  LinearToSRGB(L.z);
}

const uint constRayFlags=         RAY_FLAG_FORCE_OPAQUE|RAY_FLAG_SKIP_TRIANGLES;

[shader("raygeneration")]
void ray_gen_shader()
{
  auto rayIndex=DispatchRaysIndex();
  float x=rayIndex.x;
  float y=rayIndex.y;
  
  // 256 rays per pixel.
  const  int raysPerPixel=256;

  const float3  cameraP = float3(0, -10, 1); // go back 10 and up 1

  const float3 cameraZ = normalize(cameraP);
  const float3  cameraX = normalize(cross(float3(0,0,1), cameraZ));
  const float3 cameraY = normalize(cross(cameraZ, cameraX));
   
  float halfPixW = 1.0f / world.image.width;
  float halfPixH = 1.0f / world.image.height; 
  float filmY = -1.0f + 2.0f * (float)y / (float)world.image.height;
  float filmX = -1.0f + 2.0f * (float)x / (float)world.image.width;

  float3 color = {};
  float contrib = 1.0f / (float)raysPerPixel;

  for (int rayIndex = 0; rayIndex < raysPerPixel; rayIndex++) {

    float offX = filmX + (RandomBilateral() * halfPixW);
    float offY = filmY + (RandomBilateral() * halfPixH);
    float3 filmP = filmCenter + (offX * halfFilmW * cameraX) + (offY * halfFilmH * cameraY);
    float3 rayOrigin = cameraP;
    float3 rayDirection = normalize(filmP - cameraP);

    //    RayCast(&world, rayOrigin, rayDirection);
    RayDesc r={};
    r.Origin=rayOrigin;
    r.Direction=rayDirection;
    r.TMax=100;//TODO:
    MyPayload p;
    TraceRay(
             MyScene,
             constRayFlags,
             0xFF, //8 bit InstanceMask.
             //NOTE: the only thing that contributes to the hit group should be what instance it is.
             0,//RayContributionToHitGroupIndex
             0,//MultiplierForGeometryContributionToShaderIndex
             0, // MissShaderIndex, NOTE: here we use index for the single miss shader that we have.
             r, p
             );

    color = color + contrib * p.color;
  }

  LinearToSRGB(color);
  
  gpuTex[rayIndex.xy] = color;
}


[shader("miss")]
void miss_main(inout MyPayload payload)
{
  // TODO:  compute contributions of sky.
}

//TODO: we'll also want the sphere intersection idea.
[shader("intersection")]
void intersection_plane()
{
  const float tolerance = 0.0001f;
  const   float minHitDistance = 0.001f;

  uint id = InstanceID(); //user provided value for instance id of blas in tlas.

  float THit = RayTCurrent();

  float3 rayOrigin= WorldRayOrigin();//world space origin for the current ray;
  float3 rayDirection=WorldRayDirection();
  
  MyAttributes attr;//user defined attribute for the hit.

  bool hit =false;

  //TODO: need to get a const buffer bound that is "world" below.
  //TODO: also need to define plane_t.
  
  // check hit.
  plane_t plane = world.planes[id];
  float denom = dot(plane.n, rayDirection);
  if ((denom < -tolerance) || (denom > tolerance))
    {
      float t = (-plane.d - dot(plane.n, rayOrigin)) / denom;
      if ((t > minHitDistance) && (t < hitDistance))
        {
          attr.hitDistance = t;
          attr.hitMatIndex = plane.matIndex;
          attr.nextNormal = plane.n;
          hit=true;
        }
    }
  
  if(hit)
    ReportHit(
              attr.hitDistance//THit
              /*hitKind*/ 0,
              attr);
  
}

[shader("closesthit")]
void closesthit_main(inout MyPayload payload, in MyAttributes attr)
{

  material_t mat = world.materials[attr.hitMatIndex];

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
