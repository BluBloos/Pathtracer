cl /Z7 /I include /I include/external /std:c++20 /FePathtracer src/win32_main.cpp src/inf_forge_win.c /link User32.lib DXGI.lib Gdi32.lib