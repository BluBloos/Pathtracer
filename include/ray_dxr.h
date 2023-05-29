// TODO: is there a way to de-duplicate this information??

// NOTE: how does HLSL pack data??
// - 4 byte boundaries.
// - if a var straddles 16 byte boundary (float4), it starts at the next float4.
// - first elem of any struct is on the float4 boundary (afaik).

#pragma pack(push, 4)
struct dxr_material_t {
  float emitColor[4];
  float scatter[4];
  float refColor[4];
};

struct dxr_plane_t {
  float d;
  unsigned int matIndex;
  unsigned int padding[2];
  float n[3];
};

struct dxr_image_t {
  unsigned int width;
  unsigned int height;
};

// NOTE: for now, these are fixed size arrays for simplicity.
struct dxr_world {
  dxr_material_t materials[1];
  dxr_plane_t planes[1];
  dxr_image_t image; // TODO: this is now unused.
};
#pragma pack(pop)
