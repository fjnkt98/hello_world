#ifndef HELLO_WORLD_CPP__VISIBILITY_CONTROL_H_
#define HELLO_WORLD_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HELLO_WORLD_CPP_EXPORT __attribute__ ((dllexport))
    #define HELLO_WORLD_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define HELLO_WORLD_CPP_EXPORT __declspec(dllexport)
    #define HELLO_WORLD_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef HELLO_WORLD_CPP_BUILDING_DLL
    #define HELLO_WORLD_CPP_PUBLIC HELLO_WORLD_CPP_EXPORT
  #else
    #define HELLO_WORLD_CPP_PUBLIC HELLO_WORLD_CPP_IMPORT
  #endif
  #define HELLO_WORLD_CPP_PUBLIC_TYPE HELLO_WORLD_CPP_PUBLIC
  #define HELLO_WORLD_CPP_LOCAL
#else
  #define HELLO_WORLD_CPP_EXPORT __attribute__ ((visibility("default")))
  #define HELLO_WORLD_CPP_IMPORT
  #if __GNUC__ >= 4
    #define HELLO_WORLD_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define HELLO_WORLD_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HELLO_WORLD_CPP_PUBLIC
    #define HELLO_WORLD_CPP_LOCAL
  #endif
  #define HELLO_WORLD_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // HELLO_WORLD_CPP__VISIBILITY_CONTROL_H_
