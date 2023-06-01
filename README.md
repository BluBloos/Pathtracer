# Pathtracer

a simple multi-threaded pathtracer.

CPU VERSION:
![test](https://user-images.githubusercontent.com/38915815/190832651-a5e9dd42-5df7-4130-9d25-d87d7a039c7e.jpeg)

DXR 1.0 VERSION:
![dxr](dxr_expected.bmp)

# Steps for Building

This project depends on my WIP game engine, Automata-Engine. You can find that here: https://github.com/BluBloos/Automata-Engine

To build and run the raytracer on the scene, simply:

```bash
cmake -B build
```

When testing, I used the Visual Studio generator. I don't know if things work otherwise (say, if not using MSVC compiler).

Things don't actually work out of the box. dxil.dll needs to be copied manually to the same directory as Raytracer.exe

# Notes

These notes are for future me.

The code is very cobbled together. When porting to DXR, I justed wanted to get it working.
I "broke" the original version (the scene is no longer the same), but it otherwise still runs.
Just toggle DXR_MODE macro from 1 to 0 to get back the original version.

