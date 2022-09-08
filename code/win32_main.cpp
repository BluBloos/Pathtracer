#include <stdio.h>
#include <stdlib.h>
#include "ray.h"

INTERNAL unsigned int GetTotalPixelSize(image_32 image)
{
    return image.height * image.width * sizeof(unsigned int);
}

INTERNAL image_32 AllocateImage(unsigned int width, unsigned int height)
{
    image_32 newImage = {};
    newImage.width = width;
    newImage.height = height;
    unsigned int totalPixelSize = GetTotalPixelSize(newImage);
    newImage.pixelPointer = (unsigned int *)malloc(totalPixelSize);
    return newImage;
}

INTERNAL void WriteImage(image_32 image, char *fileName)
{
    unsigned int outputPixelSize = GetTotalPixelSize(image);
    bitmap_header header = {};
    header.FileType = 0x4D42;   
    header.FileSize = sizeof(bitmap_header) + outputPixelSize;
    header.BitmapOffset = sizeof(header); 
    header.size = 40;    
    header.Width = image.width;  
    header.Height = image.height;
    header.Planes = 1;          
    header.BitsPerPixel = 32;    
    FILE *fileHandle = fopen("test.bmp", "wb");
    
    if(fileHandle)
    {
        //"The C Runtime library introduces another layer of cruft inbetween us and the operating system" - Casey Muratori
        fwrite(&header, sizeof(header), 1, fileHandle);
        fwrite(image.pixelPointer, outputPixelSize, 1, fileHandle);
        fclose(fileHandle);
    }
    else
    {
        fprintf(stderr, "[ERROR] Unable to write output file\n");
    }
}

INTERNAL v3 RayCast(struct world *world, v3 rayOrigin, v3 rayDirection)
{
    v3 result = {};
    
    float tolerance = 0.0001f;
    float minHitDistance = 0.001f;
    
    v3 attenuation = V3(1,1,1);
    for (unsigned int rayCount = 0;
         rayCount < 8;
         ++rayCount)
    {
        float hitDistance = FLT_MAX;
        unsigned int hitMatIndex = 0;
        v3 nextNormal = {};
        
        //NOTE(Noah): The tolerance below is ad-hoc
        for (unsigned int sphereIndex= 0;
             sphereIndex < world->sphereCount; 
             sphereIndex++)
        {
            struct sphere sphere = world->spheres[sphereIndex];
            
            v3 sphereRelativeRayOrigin = rayOrigin - sphere.p;
            float a = Dot(rayDirection, rayDirection);
            float b = 2.0f * Dot(sphereRelativeRayOrigin, rayDirection);
            float c = Dot(sphereRelativeRayOrigin, sphereRelativeRayOrigin) - sphere.r * sphere.r;
            
            float denom = 2.0f * a;
            
            float rootTerm = SquareRoot(b*b - 4.0f*a*c);
            if ( rootTerm > tolerance)
            {
                //NOTE(Noah): The denominator can never be zero
                float tp = (-b + rootTerm) / denom;
                float tn = (-b - rootTerm) / denom;
                
                float t = tp;
                if ( (tn > minHitDistance) && (tn < tp) )
                {
                    t = tn;
                }
                
                if ( (t > minHitDistance) && (t < hitDistance) )
                {
                    hitDistance = t;
                    //result = world->materials[sphere.matIndex].color;
                    hitMatIndex = sphere.matIndex;
                    nextNormal = Normalize(t*rayDirection + sphereRelativeRayOrigin);
                }
            }
        }
        
        for (unsigned int planeIndex = 0; 
             planeIndex < world->planeCount;
             planeIndex++)
        {
            struct plane plane = world->planes[planeIndex];
            
            float denom = Dot(plane.n, rayDirection);
            if ( (denom < -tolerance) || (denom > tolerance) )
            {
                float t = (-plane.d - Dot(plane.n, rayOrigin)) / denom;
                if ( (t > minHitDistance) && (t < hitDistance) )
                {
                    hitDistance = t;
                    //result = world->materials[plane.matIndex].color;
                    hitMatIndex = plane.matIndex;
                    nextNormal = plane.n;
                }
            } 
        }
        
        if (hitMatIndex)
        {
            material mat = world->materials[hitMatIndex];
            
            //TODO(Noah): Do real reflectance stuff
            result = result + Hadamard(attenuation, mat.emitColor);
            attenuation = Hadamard(attenuation, mat.refColor);
            
            rayOrigin = rayOrigin + hitDistance*rayDirection;
            
            //NOTE(Noah): this does a reflection thing
            //TODO(Noah): these are not accurate permutations
            v3 pureBounce = rayDirection - 2.0f*Dot(nextNormal, rayDirection)*nextNormal;
            v3 randomBounce = Normalize(nextNormal + V3(RandomBilateral(), RandomBilateral(), RandomBilateral()));
            rayDirection = Normalize(Lerp(randomBounce, pureBounce, mat.scatter));
            //rayDirection = pureBounce;
        }
        else
        {
            material mat = world->materials[hitMatIndex];
            result = result + Hadamard(attenuation, mat.emitColor);
            break;
        }
    }
    return result;
    
}

int main(int argc, char **argv)
{
    printf("Doing stuff...\n");
    
    image_32 image = AllocateImage(1280, 720);
    
    struct world world = {};
    
    material materials[4] = {};
    materials[0].emitColor = V3(0.3f, 0.4f, 0.5f);
    materials[1].refColor = V3(0.5f, 0.5f, 0.5f);
    materials[2].refColor = V3(0.7f, 0.25f, 0.3f);
    materials[3].refColor = V3(0.0f, 0.8f, 0.0f);
    materials[3].scatter = 1.0f;
    
    plane planes[1] = {};
    //plane equation
    //n dot any point on the plane plus d = 0
    //so, we've defined a plane on the origin
    planes[0].n = V3(0,0,1);
    planes[0].d = 0;
    planes[0].matIndex = 1;
    
    sphere spheres[3] = {};
    spheres[0].p = V3(0,0,0);
    spheres[0].r = 1.0f;
    spheres[0].matIndex = 2;
    spheres[1].p = V3(-3,-2,0);
    spheres[1].r = 1.0f;
    spheres[1].matIndex = 2;
    spheres[2].p = V3(-2,0,2);
    spheres[2].r = 1.0f;
    spheres[2].matIndex = 3;
    
    world.materialCount = ArrayCount(materials);
    world.materials = materials;
    world.planeCount = ArrayCount(planes);
    world.planes = planes;
    world.sphereCount = ArrayCount(spheres);
    world.spheres = spheres;
    
    v3 cameraP = V3(0, -10, 1); //go back 10 and up 1  
    v3 cameraZ = Normalize(cameraP);
    v3 cameraX = Normalize(Cross(V3(0,0,1), cameraZ));
    v3 cameraY = Normalize(Cross(cameraZ, cameraX));
    
    float filmDist = 1.0f;
    float filmW = 1.0f;
    float filmH = 1.0f;
    
    if (image.width > image.height)
    {
        filmH = filmW * (float)image.height / (float)image.width;
    } else if (image.height > image.width)
    {
        filmW = filmH * (float)image.width / (float)image.height;
    }
    
    float halfFilmW = filmW / 2.0f;
    float halfFilmH = filmH / 2.0f;
    v3 filmCenter = cameraP - filmDist*cameraZ;
    
    float halfPixW = 1.0f / image.width;
    float halfPixH = 1.0f / image.height;
    
    unsigned int *out = image.pixelPointer;
    unsigned int raysPerPixel = 256;
    for (unsigned int y = 0; 
         y < image.height; 
         y++)
    {
        float filmY = -1.0f + 2.0f * (float)y / (float)image.height;
        for (unsigned int x = 0; 
             x < image.width; 
             x++)
        {
            float filmX = -1.0f + 2.0f * (float)x / (float)image.width;
            
            v3 color = {};
            float contrib = 1.0f / (float)raysPerPixel;
            for (unsigned int rayIndex = 0; 
                 rayIndex < raysPerPixel;
                 rayIndex++)
            
            {
                float offX = filmX + RandomBilateral() * halfPixW;
                float offY = filmY + RandomBilateral() * halfPixH;
                v3 filmP = filmCenter + offX*halfFilmW*cameraX + offY*halfFilmH*cameraY;
                v3 rayOrigin = cameraP;
                v3 rayDirection = Normalize(filmP - cameraP);
                
                color = color + contrib*RayCast(&world, rayOrigin, rayDirection);
            }
            
            
            //printf("color: %d, %d, %d", r, g, b);
            //v4 BMPColor = V4(255.0f*color, 255.0f);
            v4 BMPColor = {
                255.0f*LinearToSRGB(color.r),
                255.0f*LinearToSRGB(color.g),
                255.0f*LinearToSRGB(color.b), 
                255.0f
            }; 
            
            unsigned int BMPValue = BGRAPack4x8(BMPColor);
            *out++ = BMPValue; //ARGB
        }
        if (y % 64)
        {
            printf("\rRaycasting row %d...", y);
            fflush(stdout);
        }
    }
    
    WriteImage(image, "test.bmp");
    
    printf("Done.\n");
    return(0);
}