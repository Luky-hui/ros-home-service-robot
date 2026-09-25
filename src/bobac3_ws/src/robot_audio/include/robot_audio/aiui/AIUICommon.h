#ifndef AIUI_COMMON_HDR_X2342Y3
#define AIUI_COMMON_HDR_X2342Y3

#if defined(WIN32)
#    ifndef AIUI_WINDOWS
#        define AIUI_WINDOWS
#    endif
#endif

#if defined(AIUI_WINDOWS)
#    if !defined(__MINGW32__)
typedef long ssize_t;
typedef unsigned long pid_t;
#    endif

#    undef AIUIEXPORT
#    if defined(AIUI_LIB_COMPILING)
#        define AIUIEXPORT __declspec(dllexport)
#        define AIUIHIDDEN
#        define AIUIAPI __stdcall
#    else
#        define AIUIEXPORT
#        define AIUIHIDDEN
#        define AIUIAPI __stdcall
#    endif
#else
#    undef AIUIEXPORT
#    define AIUIEXPORT __attribute__((visibility("default")))
#    define AIUIHIDDEN __attribute__((visibility("hidden")))
#    undef AIUIAPI
#    define AIUIAPI
#endif

#ifndef __cplusplus
#define AIUI_PRFIX(x) AIUI_ ## x
#else
#define AIUI_PRFIX(x) x
#endif
#endif

