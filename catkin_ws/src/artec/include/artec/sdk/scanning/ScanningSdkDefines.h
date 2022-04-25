#ifndef ScanningSdkDefines_h__
#define ScanningSdkDefines_h__

#if defined(_WIN32) && defined(BUILD_ASCANNINGSDK_SHARED_LIB)
	#define ASCANNINGSDK_LINK_SPEC __declspec (dllexport)
#else
	#define ASCANNINGSDK_LINK_SPEC
#endif // _WIN32 DLL

#endif // ScanningSdkDefines_h__