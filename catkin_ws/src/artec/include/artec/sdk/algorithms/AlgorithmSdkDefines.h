#ifndef AlgorithmSdkDefines_h__
#define AlgorithmSdkDefines_h__

#if defined(_WIN32) && defined(BUILD_AALGORITHMSDK_SHARED_LIB)
	#define AALGORITHMSDK_LINK_SPEC __declspec (dllexport)
#else
	#define AALGORITHMSDK_LINK_SPEC
#endif // _WIN32 DLL

#endif // AlgorithmSdkDefines_h__