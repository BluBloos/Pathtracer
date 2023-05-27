// TODO: is there a way to de-duplicate this information??
struct dxr_material_t {
  float emitColor[4];
  float scatter[4];
  float refColor[4];
};

struct dxr_plane_t {
  float n[3];
  float d[3];
  unsigned int matIndex;
};

struct dxr_image_t {
  unsigned int width;
  unsigned int height;
};

// NOTE: for now, these are fixed size arrays for simplicity.
struct dxr_world {
  material_t materials[1];
  plane_t planes[1];
  dxr_image_t image;
};
