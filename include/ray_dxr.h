// TODO: is there a way to de-duplicate this information??
//
// like, include this .h directly into the hlsl shader code??
//
// like, yes. this can be done. it's a fair idea. it's called a "compat" header.


// NOTE: how does HLSL pack data??
// - 4 byte packing.
// - if a var straddles 16 byte boundary (float4), it starts at the next float4.
// - arrays start on a float4 boundary.

#pragma pack(push, 4)
struct dxr_material_t {
  float emitColor[3];
  float scatter;
  float refColor[3];
  int padding;
};

struct dxr_plane_t {
  float d;
  unsigned int matIndex;
  unsigned int padding[2];
  float n[3];
  int padding2;
};

struct dxr_sphere_t {
  float r;
  unsigned int matIndex;
  unsigned int padding[2];
};

struct dxr_random_pair_t
{
  float xy[2];
  int padding[2];
};

#define DXR_WORLD_LIMIT 10

// NOTE: for now, these are fixed size arrays for simplicity.
struct dxr_world {
  dxr_material_t materials[DXR_WORLD_LIMIT];
  dxr_plane_t planes[DXR_WORLD_LIMIT];
  dxr_sphere_t spheres[DXR_WORLD_LIMIT];
  dxr_random_pair_t rands[256];//TODO: having the 256 here is hacky.
};
#pragma pack(pop)
