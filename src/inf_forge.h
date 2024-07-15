/*
File: inf_forge.h
Author: Noah J. Cabral
Description: single-header library for writing portable game engines and graphical applications.
Legal: Copyright 2024, Noah J. Cabral. All rights reserved.
*/

#ifndef INF_FORGE
#define INF_FORGE

#ifdef __cplusplus
extern "C" {
#endif

#define IF_BOOL char
#define U8 unsigned char
#define INF_FORGE_NAME_STRING "Inf Forge"

// this macro will appear in the definition of functions that are "good to go"
// for the initial release of the library.
#define IF_V1

#define IF_API


typedef struct {
    void *opaque_handle;
} IF_window_handle_t;

typedef struct {
    void *opaque_handle;
} IF_thread_t;

typedef struct {
    void *data;
    int size;
} IF_rawpacket_t;

typedef enum {
    IF_ERROR_UNKNOWN,
    IF_ERROR_NO_MONITOR,
} IF_error_e;

typedef struct {
    IF_BOOL running;

    U8 *backbuffer;
    int bbwidth;
    int bbheight;

    // sb = stretchy buffer.
    // the strechy buffer is really the only clean way that I know
    // in C to support a generic list type. it's the best because
    // the addressing syntax works, which is very important.
    IF_rawpacket_t *mouse_inputbuffer_sb;

    IF_error_e lasterror;
} IF_glob_t;

typedef enum {
    IF_WIN_MSG_QUIT
} IF_winmsg_kind_e;

typedef struct {
    IF_winmsg_kind_e code;
    IF_BOOL isvalid;
} IF_winmsg_t;

typedef struct {
    int width;
    int height;
    int x,y; // bottom left.
} IF_rect_t;

typedef enum {
    IF_TEXTURE_FORMAT_RGBA8,
} IF_texture_format_t;

typedef struct {
    int width;
    int height;
    IF_texture_format_t format;
} IF_texture_info_t;

typedef struct {
    IF_BOOL mouse;
    IF_BOOL keyboard;
    IF_BOOL microphone;
} IF_hdw_request_info_t;

typedef void (*IF_func)(void);


// --- LIBRARY STATE FUNCTIONS ---
// get global object.
IF_API IF_glob_t *IF_glob();
IF_API IF_error_e IF_get_lasterror();


// --- TIMING FUNCTIONS ---
// returns rdtsc clocks for one second period.
double IF_get_hdwclock_frequency();


// --- MEMORY BLOCK FUNCTIONS ---
#define IF_zero(lval) memset(&lval, 0, sizeof(lval))
/*C8
void IF_zero<typename T>( T *lval )
{
    memset(lval, 0, sizeof(*lval));
}
*/
IF_API void *IF_malloc(int);


// --- THREADING FUNCTIONS ---
IF_API IF_thread_t IF_launch(IF_func);


// --- INPUT FUNCTIONS ---
// returns false if the message has code == IF_WIN_MSG_QUIT.
IF_API IF_BOOL IF_win_get_message(IF_window_handle_t, IF_winmsg_t *);
IF_BOOL IF_win_poll_message(IF_window_handle_t, IF_winmsg_t *);

// --- WINDOW FUNCTIONS ---
// IF_create_window is not thread safe (hopefully you weren't thinking of doing
// something psychopathic like creating tons of windows concurrently from different
// threads).
// Do not call IF_create_window from a DLL.
IF_API IF_window_handle_t IF_create_window(int width, int height, const char 
    *title);
IF_API void IF_close_window(IF_window_handle_t);
IF_API IF_rect_t IF_get_window_clientarea(IF_window_handle_t); 
                                    // not always equal to requested window 
                                    // size due to borders.
IF_API void IF_blit_to_window_surface(IF_window_handle_t, void *pixels, 
    IF_texture_info_t *pixels_info);
void IF_swap_window_buffers_after_vsync(IF_window_handle_t);


#ifdef INF_FORGE_IMPLEMENTATION


static IF_glob_t g_object;

// get global object.
IF_V1 IF_glob_t *IF_glob()
{
    return &g_object;
}

IF_V1 IF_error_e IF_get_lasterror()
{
    return IF_glob()->lasterror;
}


#endif // #ifdef INF_FORGE_IMPLEMENTATION


#ifdef __cplusplus
}
#endif


#undef U8


#endif // #ifndef INF_FORGE