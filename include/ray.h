#include "ray_math.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof(array[0]))

#pragma pack(push, 1)
typedef struct bitmap_header {
    unsigned short FileType;        /* File type, always 4D42h ("BM") */
    unsigned int   FileSize;        /* Size of the file in bytes */
    unsigned short Reserved1;       /* Always 0 */
    unsigned short Reserved2;       /* Always 0 */
    unsigned int   BitmapOffset;    /* Starting position of image data in bytes */
    unsigned int   size;            /* Size of this header in bytes */
    int            Width;           /* Image width in pixels */
    int            Height;          /* Image height in pixels */
    unsigned short Planes;          /* Number of color planes */
    unsigned short BitsPerPixel;    /* Number of bits per pixel */
    unsigned int   biCompression;   /* specfies type of compression to be used */
    unsigned int   biSizeImage;     /* can be set to 0 for uncompressed RGB bitmaps */
    int            biXPelsPerMeter; /* horizontal resolution in pixels per meter */
    int            biYPelsPerMeter; /* vertical resoltion in pixels per meter */
    unsigned int   biClrUsed;       /* specifies color indices used in color table */
    unsigned int   biClrImportant;  /* specifies color indices that are important */
    char           rgbBlue;
    char           rgbGreen;
    char           rgbRed;
    char           rgbReserved;
} bitmap_header_t;
#pragma pack(pop)

typedef struct image_32 {
    unsigned int  width;
    unsigned int  height;
    unsigned int *pixelPointer;
} image_32_t;

typedef struct material {
    float scatter;  // 0 is pure diffuse, 1 is pure spec
    v3    refColor; /* reflection color */
    v3    emitColor;
} material_t;

// plane equation
// n dot any point on the plane plus d = 0
typedef struct plane {
    v3           n;
    float        d;  // coeff in plane eq
    unsigned int matIndex;
} plane_t;

typedef struct sphere {
    v3           p;
    float        r;
    unsigned int matIndex;
} sphere_t;

typedef struct world {
    unsigned int materialCount;
    material    *materials;
    unsigned int planeCount;
    plane       *planes;
    unsigned int sphereCount;
    sphere      *spheres;
} world_t;

#if AUTOMATA_ENGINE_DX12_BACKEND
#include <D3d12.h>
#include <dxgi1_6.h>
#include <D3dx12.h>
#endif

// TODO: we need to free many of the resources in this structure.
struct game_data {

#if AUTOMATA_ENGINE_DX12_BACKEND
    // some stupid shit.
    ID3D12Device *d3dDevice       = NULL;
    ID3D12Debug  *debugController = NULL;

    // pipeline stuff.
    ID3D12PipelineState *computePipelineState = NULL;
    ID3D12RootSignature *rootSig              = NULL;
    ID3D12StateObject   *rayStateObject       = NULL;

    // resources.
    ID3D12Resource *gpuTex = NULL;
    ID3D12Resource *cpuTex = NULL;

    // stuff for raytracing.
    ID3D12Resource *sceneBuffer    = NULL;
    ID3D12Resource *rayShaderTable = NULL;
    ID3D12Resource *tlas           = NULL;
    ID3D12Resource *BLASes[2]      = {};  // TODO: 2 constant is hacky.

    ID3D12Resource *tlasScratch      = NULL;
    ID3D12Resource *blasScratches[2] = {};  // TODO: 2 constant is hacky.

    ID3D12Resource *AABBs         = NULL;
    ID3D12Resource *tlasInstances = NULL;

    ID3D12DescriptorHeap *descHeap = NULL;

    // for submitting shit.
    ID3D12CommandQueue *commandQueue = NULL;

    // main command list shit for doing some of the setup work.
    // after the setup work we also use this for the UAV copy to game backbuffer.
    ID3D12CommandAllocator    *commandAllocator = NULL;
    ID3D12GraphicsCommandList *commandList      = NULL;
    ID3D12Fence               *fence            = NULL;
    int                        fenceValue;
    HANDLE                     fenceEvent;

    // we keep around cmd lists and fences for each ray render thread
    // so that we can do the dispatching.
    ID3D12CommandAllocator    *rayCmdAllocators[THREAD_COUNT] = {};
    ID3D12GraphicsCommandList *rayCmdLists[THREAD_COUNT]      = {};
    ID3D12Fence               *rayFences[THREAD_COUNT]        = {};
    int                        rayFenceValues[THREAD_COUNT]   = {};
    HANDLE                     rayFenceEvents[THREAD_COUNT]   = {};
#endif

#if AUTOMATA_ENGINE_VK_BACKEND
    VkInstance vkInstance;// = NULL;
    VkDebugUtilsMessengerEXT vkDebugMsg;// = nullptr;
    VkPhysicalDevice vkGpu; //= gpus[0];
    VkDevice vkDevice;

    // upload heaps.
    uint32_t vkUploadHeapIdx;
    uint32_t vkVramHeapIdx;

    // resources.
    VkImage vkCpuTex;
    VkImageView vkCpuTexView;
    VkBuffer vkCpuTexBuffer;

    // memory backing the resources.
    VkDeviceMemory vkCpuTexBacking;
    VkDeviceMemory vkCpuTexBufferBacking;
    size_t vkCpuTexSize;
    size_t vkCpuTexBufferSize;

    // layout.
    VkDescriptorSetLayout descSet; // TODO: maybe rename this one.
      // the layout is used as part of the pipeline sig + a way to allocated sets from the pool.
    VkPipelineLayout pipelineLayout;
    VkDescriptorSet theDescSet;    // allocated from the pool.
    VkDescriptorPool descPool;

    // for the compute copy work.
    VkCommandBuffer cmdBuf;
    VkCommandPool commandPool;
    VkPipeline vkComputePipeline;

    // for the queue.
    int vkQueueIndex = -1;
    VkQueue vkQueue;
    VkFence vkFence;
#endif
};
