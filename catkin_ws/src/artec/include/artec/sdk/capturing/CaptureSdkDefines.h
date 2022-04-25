#ifndef CaptureSdkDefines_h__
#define CaptureSdkDefines_h__

#if defined(_WIN32) && defined(BUILD_ACAPTURESDK_SHARED_LIB)
	#define ACAPTURESDK_LINK_SPEC __declspec (dllexport)
#else
	#define ACAPTURESDK_LINK_SPEC
#endif // _WIN32 DLL

#endif // CaptureSdkDefines_h__