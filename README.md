# Path Tracer

My personal implementation of a roughly "feature complete" Path Tracer. The
beginnings of the codebase were roughly identical to the Path Tracer constructed
during the Handmade Ray Day 1 livestream, archived at
https://guide.handmadehero.org/ray/ray00/. From then, I programmed the rest "in
the dark" (for the most part); i.e., I didn't consult any open source Path
Tracer implementations. To guide development, I sourced content from books,
which I reference in the References section below. I'm aware that this isn't the
ideal approach for writing a Path Tracer as fast as possible; I chose this
approach because I was looking for a coding adventure :)

![test](https://github.com/BluBloos/Pathtracer/assets/38915815/5c0ad3e0-27ed-46ff-a8ab-6bab018551e5)
![test_balls](https://github.com/BluBloos/Pathtracer/assets/38915815/dffa43b0-96ec-4a4e-8481-90217ea9e0c1)
![test_onewknd](https://github.com/BluBloos/Pathtracer/assets/38915815/9cbaa92f-d464-4c7a-80ab-7ea359e16c3c)
![testmario](https://github.com/BluBloos/Pathtracer/assets/38915815/5016aece-4b04-4a5f-acae-41a97004e1c0)
![cornell](https://github.com/BluBloos/Pathtracer/assets/38915815/32e7af83-8507-4166-ba88-3ac51b019861)

# Features

- Importance sampling Monte Carlo Integration.
  - GGX importance sampling.
  - Emissive sphere importance sampling.
  - Cosine sampling.
- Materials:
  - Specular BSDF from microfacet theory: GGX distribution function, Smith joint
  shadow-masking courtesy of Hammon et al., and Slick approximation of Fresnel equations.
  - Uniform diffuse BSDF.
  - Albedo, roughness, metalness, and normal textures.
  - Disney "principled" roughness parameterization.
- Physical lens model.
- GLTF import.
- Supported Geometry: Triangles, Spheres, Planes, AABBs, Quads.
- Multithreaded architecture.
- Live viewer.
- Output .BMP image format.

# References

- I read https://www.pbr-book.org/4ed/Monte_Carlo_Integration and maybe other chapters.
- I own a physical copy of https://www.realtimerendering.com/; I sampled various
  chapters, especially chapter 9.
- I read the entire series to ensure completeness of my implementation:
  - [_Ray Tracing in One Weekend_](https://raytracing.github.io/books/RayTracingInOneWeekend.html)
  - [_Ray Tracing: The Next Week_](https://raytracing.github.io/books/RayTracingTheNextWeek.html)
  - [_Ray Tracing: The Rest of Your Life_](https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html)
- Importance sampling GGX: https://schuttejoe.github.io/post/ggximportancesamplingpart1/

# Usage

e.g.: "AutomataApp.exe -t16 -p16 -nmr"
```
t<int>                        - Set the number of threads to use.
p<int>                        - Set the rays to shoot per pixel.
w<int>                        - Set the world number to load. Possible options:
  1:  Default scene.
  2:  Metal-roughness test.
  3:  Cornell box.
  4:  Ray Tracing in One Weekend book cover.
  5:  Mario N64 model.
    
d                             - Enable depth of field via thin-lens approximation.
n                             - Disable loading normal map textures.
m                             - Disable loading metalness material textures.
r                             - Disable loading roughness material textures.
h                             - Print this help menu.
```
# Steps for Building

This project depends on my game engine, Automata-Engine. You can find that here: https://github.com/BluBloos/Automata-Engine

The engine has its own dependencies, but the TLDR is:
- this project only works on Windows
- install Python and make sure its on your PATH
- install MS Visual Studio 2022, community ed. and ensure it is located at "C:\Program Files"

To build and run the raytracer on the scene, please run the following.

```bash
build.bat
bin\AutomataApp.exe
```

Happy path tracing!
