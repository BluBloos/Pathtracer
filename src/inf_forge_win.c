/*
File: inf_forge_win.c
Author: Noah J. Cabral
Description: Inf-Forge Windows support
*/

#include <Windows.h>
#include <dxgi1_4.h>
#include <combaseapi.h>
#include <processthreadsapi.h>
#include <wingdi.h>

#include "inf_forge.h"

#define UINT32_MAX 4294967295U

typedef struct {
    HMONITOR hmon;
    RECT rect;
} win32_monitor_data;

#define TYPE win32_monitor_data
#define MAX_ELEMENTS 10 
#include "if_dyn_array.inc"
static win32_monitor_data_array_t g_monitor_data;
/*C8
typedef struct {
    cconst int max_elements = me;
    int count;
    T elements[me];
} fixed_array_t<typename T, int me>;

opdef a.elements[i] (fixed_array_t<typename T, int me> a, int i) a[i];
*/

// the registered class is automatically unregister when the application terminates.
static ATOM g_classatom = 0;

// in C8 this could have been a local function within IF_launch.
DWORD thread_launcher(void *lp_parameter)
{
    IF_func func = (IF_func )lp_parameter;
    func();
    ExitThread(0);
}

IF_V1 IF_thread_t IF_launch(IF_func func)
{
    // do nothing for now.
    IF_thread_t t;
    IF_zero(t);

    LPSECURITY_ATTRIBUTES   lpThreadAttributes = NULL;
    SIZE_T                  dwStackSize = 0; // default stack size.
    LPTHREAD_START_ROUTINE  lpStartAddress = thread_launcher;
    LPVOID                  lpParameter = (void *)func;
    DWORD                   dwCreationFlags = 0; // thread runs immediately     
                                                 // after creation.
    LPDWORD                 lpThreadId = NULL;

    HANDLE hdl = CreateThread(
        lpThreadAttributes,
        dwStackSize,
        lpStartAddress,
        lpParameter,
        dwCreationFlags,
        lpThreadId
    );

    t.opaque_handle = hdl;

    return t;
}

// --- INPUT FUNCTIONS ---
// returns false if the message has code == IF_WIN_MSG_QUIT.
IF_BOOL IF_win_get_message(IF_window_handle_t hdl, IF_winmsg_t *if_msg)
{
    MSG msg;
    HWND hwnd = NULL;
    UINT msgfiltermin = 0;
    UINT msgfiltermax = 0;
    BOOL ret = GetMessageA(&msg, hwnd, msgfiltermin, msgfiltermax);

    if (ret > 0) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }

    return ret != 0;
}

// this function cannot be called from a DLL because then the classatom
// will _not_ be automatically unregistered when the application terminates.
IF_V1 IF_window_handle_t IF_create_window(int width, int height, char *title)
{
    IF_window_handle_t hdl;
    IF_zero(hdl);

    BOOL CALLBACK win32_monitor_enum_proc(HMONITOR, HDC, LPRECT, LPARAM);
    LRESULT CALLBACK win32_window_proc(HWND, UINT, WPARAM, LPARAM);

    HINSTANCE instance = GetModuleHandleA(NULL);

    // window create params.
    int x      = CW_USEDEFAULT;
    int y      = CW_USEDEFAULT;
    int dwidth  = CW_USEDEFAULT;
    int dheight = CW_USEDEFAULT;

    HDC             hdc = NULL; // region of interest for enumeration is the virtual region that encompass all desktop displays.
    LPCRECT         lprcClip = NULL;
    LPARAM          dwData = 0;

    g_monitor_data = create_win32_monitor_data_array();
    
    EnumDisplayMonitors(
        hdc,
        lprcClip,
        win32_monitor_enum_proc, // writes to g_monitor_data.
        dwData
    );

    RECT r = g_monitor_data.elements[0].rect;

    int fwidth      = r.right - r.left;
    int fheight     = r.bottom - r.top;

    dwidth  = ((float)fwidth * 0.8F);
    dheight = (float)fheight * 0.8f;

    x      = r.left + (fwidth - dwidth) >> 1;
    y      = r.top + (fheight - dheight) >> 1;


    DWORD       dwExStyle   = 0;
    DWORD       dwStyle = (WS_OVERLAPPEDWINDOW | WS_VISIBLE);

    HWND      hWndParent = NULL;
    HMENU     hMenu = NULL;
    LPVOID    lpParam = NULL; // structure to be passed to WM_CREATE message.

    WNDCLASSA windowClass;
    IF_zero(windowClass);

    windowClass.style         = CS_VREDRAW | CS_HREDRAW;    // Set window to redraw after being resized
    windowClass.lpfnWndProc   = win32_window_proc;          // Set callback
    windowClass.hInstance     = instance;
    windowClass.lpszClassName = INF_FORGE_NAME_STRING;

    if (g_classatom == 0) g_classatom = RegisterClassA(&windowClass);
    if (g_classatom == 0) {
        return hdl;
    }

    HWND nativehandle = CreateWindowExA(
        dwExStyle,
        windowClass.lpszClassName,
        title,
        dwStyle,
        x,
        y,
        (width == UINT32_MAX) ? dwidth : width,
        (height == UINT32_MAX) ? dheight : height,
        hWndParent,
        hMenu,
        instance,
        lpParam
    );

    // Show the window and paint its contents.
    if (nativehandle) {
        ShowWindow(nativehandle, SW_SHOWNORMAL); 
        UpdateWindow(nativehandle);
    }

    hdl.opaque_handle = (void*)nativehandle;

    return hdl;
}

IF_V1 static BOOL CALLBACK win32_monitor_enum_proc(HMONITOR hmon, HDC hdc, LPRECT lprc_monitor, LPARAM pData)
{
    win32_monitor_data data;
    data.hmon = hmon;
    data.rect = *lprc_monitor;

    if (g_monitor_data.count == g_monitor_data.max_elements) return FALSE;
    g_monitor_data.elements[g_monitor_data.count++] = data;

    return TRUE;// continue enumeration.
}

static LRESULT CALLBACK win32_window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
    switch (message)
    {
    case WM_CLOSE:
        DestroyWindow(window);
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProcA(window, message, wparam, lparam);
    }

    return 0;
}

IF_V1 void IF_close_window(IF_window_handle_t hdl)
{
    DestroyWindow( (HWND)hdl.opaque_handle );
}

IF_V1 IF_rect_t IF_get_window_clientarea(IF_window_handle_t hdl)
{
    RECT rect;
    GetClientRect((HWND)hdl.opaque_handle, &rect);

    IF_rect_t result;
    result.width = rect.right - rect.left;
    result.height = rect.bottom - rect.top;
    result.x = rect.left;
    result.y = rect.bottom;

    return result;
}

IF_V1 void IF_blit_to_window_surface(IF_window_handle_t hdl, void *pixels, 
    IF_texture_info_t *pixels_info)
{
    HWND hwnd = (HWND)hdl.opaque_handle;

    RECT rect;
    GetClientRect(hwnd, &rect);

    BITMAPINFO info;
    info.bmiHeader.biSize = sizeof(info.bmiHeader);
	info.bmiHeader.biWidth = pixels_info->width;
	info.bmiHeader.biHeight = pixels_info->height;
	info.bmiHeader.biPlanes = 1;
	info.bmiHeader.biBitCount = 32;
	info.bmiHeader.biCompression = BI_RGB;

    HDC              hdc = GetWindowDC(hwnd); // TODO: does this need to be     
                                              // more robust in some way?
    int              xDest = 0;
    int              yDest = 0;
    int              DestWidth = rect.right - rect.left;
    int              DestHeight = rect.bottom - rect.top;
    int              xSrc = 0;
    int              ySrc = 0;
    int              SrcWidth = pixels_info->width;
    int              SrcHeight = pixels_info->height;
    const VOID       *lpBits = pixels;
    const BITMAPINFO *lpbmi = &info;
    UINT             iUsage = DIB_RGB_COLORS; // TODO: need to support more     
                                              // source formats.
    DWORD            rop = SRCCOPY;

    StretchDIBits(
        hdc,
        xDest,
        yDest,
        DestWidth,
        DestHeight,
        xSrc,
        ySrc,
        SrcWidth,
        SrcHeight,
        lpBits,
        lpbmi,
        iUsage,
        rop
    );

    ReleaseDC(hwnd, hdc);
}

void IF_swap_window_buffers_after_vsync(IF_window_handle_t hdl)
{
    // TODO: implement this.
}

IF_V1 void *IF_malloc(int bytes)
{
    return VirtualAlloc(0, bytes, MEM_COMMIT, PAGE_READWRITE);
}