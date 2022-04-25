#ifndef BaseSdkDefines_h__
#define BaseSdkDefines_h__

////////////////////////
// ABASESDK_LINK_SPEC
//
#if defined(_WIN32) && defined(BUILD_ABASESDK_SHARED_LIB)
	#define ABASESDK_LINK_SPEC __declspec (dllexport)
#else
	#define ABASESDK_LINK_SPEC
#endif // _WIN32 DLL

////////////////////////
// ASDK_DEPRECATED
//
#ifdef __has_cpp_attribute
#   if __has_cpp_attribute(deprecated)
#       define HAS_STD_DEPRECATION
#   endif
#endif // __has_cpp_attribute

#ifdef HAS_STD_DEPRECATION
#   define ASDK_DEPRECATED(func)[[deprecated]] func
#   undef HAS_STD_DEPRECATION
#else // HAS_STD_DEPRECATION
#   if defined(_MSC_VER)
#       define ASDK_DEPRECATED(func) __declspec(deprecated) func
#   elif defined(__GNUC__) || defined(__clang__)
#       define ASDK_DEPRECATED(func) func __attribute__((deprecated))
#       define ASDK_DEPRECATED(func) func __attribute__((deprecated))
#   else
#       pragma message("WARNING: ASDK_DEPRECATED macro is not implemented for this compiler")
#       define ASDK_DEPRECATED(func) func
#   endif
#endif // HAS_STD_DEPRECATION

////////////////////////
// ASDK_UNUSED
//
#define ASDK_UNUSED(x) x

#endif // BaseSdkDefines_h__