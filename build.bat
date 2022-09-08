set CompilerFlags=-O2 -MT -nologo -Oi -WX -W4 -wd4189 -wd4100 -wd4201 -wd4244 -Zi  

mkdir bin
pushd bin
cl %CompilerFlags% -D _CRT_SECURE_NO_WARNINGS -D DEBUG ../code/win32_main.cpp
popd

mkdir data
pushd data
..\bin\win32_main.exe
start test.bmp
popd
