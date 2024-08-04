set COMPILE_FLAGS=/Z7 /O2 /I include /I include/external /std:c++20

cl %COMPILE_FLAGS% /FePathtracer src/win32_main.cpp src/inf_forge_win.c /link User32.lib DXGI.lib Gdi32.lib

cl %COMPILE_FLAGS% /FeImageCompare src/image_compare.c