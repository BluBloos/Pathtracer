
RWTexture2D<float4> gpuTex : register(u0); // maps to 0th UAV register.
RWTexture2D<float4> cpuTex : register(u1); // maps to 1st UAV register.


// TODO: this is going to be replaced with the raytracing work.
[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{	
    // for now, we just output a pure red.
    gpuTex[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0);
}


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

