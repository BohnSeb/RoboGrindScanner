#pragma once

#if defined(_WIN32) && defined(BUILD_ADATASDK_SHARED_LIB)
#define APROJECT_LINK_SPEC __declspec (dllexport)
#else
#define APROJECT_LINK_SPEC
#endif // defined(_WIN32) && defined(BUILD_ADATASDK_SHARED_LIB)
