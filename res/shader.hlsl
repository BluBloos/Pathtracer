
RWTexture2D<float4> backbuffer : register(u0);

[numthreads(16, 16, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{	
    float4 textureData = backbuffer.Load(DTid.xy);
    // for now, we just output a pure red.
    backbuffer[DTid.xy] = float4(1.0, 0.0, 0.0, 1.0); // textureData; //* sin(DTid.x * DTid.y);
}