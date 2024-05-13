# Path Tracer

My personal implementation of a roughly "feature complete" Path Tracer. The
beginnings of the codebase were roughly identical to the Path Tracer constructed
during the Handmade Ray Day 1 livestream, archived at
https://guide.handmadehero.org/ray/ray00/. From then, I programmed the rest "in
the dark"; i.e., I didn't consult any open source Path Tracer implementations.
To guide development, I sourced content from textbooks which I reference in the
References section below. I'm aware that this isn't the ideal approach for
writing a Path Tracer as fast as possible; I chose this approach because I was
looking for a coding adventure :)

![test](https://user-images.githubusercontent.com/38915815/190832651-a5e9dd42-5df7-4130-9d25-d87d7a039c7e.jpeg)

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
- Surface (area) and Directional lights.
- Supported Geometry: Triangles, Spheres, Planes, AABBs, Quads.
- Multithreaded architecture.
- Live viewer.
- Output .BMP image format.

# References

- https://www.pbr-book.org/4ed/Monte_Carlo_Integration, among other chapters maybe.
- I own a physical copy of https://www.realtimerendering.com/; I sampled various
  chapters, primarly chapter 9.
- I read the entire series to ensure completeness of my implementation:
  - [_Ray Tracing in One Weekend_](https://raytracing.github.io/books/RayTracingInOneWeekend.html)
  - [_Ray Tracing: The Next Week_](https://raytracing.github.io/books/RayTracingTheNextWeek.html)
  - [_Ray Tracing: The Rest of Your Life_](https://raytracing.github.io/books/RayTracingTheRestOfYourLife.html)

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