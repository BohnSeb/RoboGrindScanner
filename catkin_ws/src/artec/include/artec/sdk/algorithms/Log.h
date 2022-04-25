/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	log code
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _AALGORITHMSDK_LOG_H_
#define _AALGORITHMSDK_LOG_H_

#ifndef AALGORITHMSDK_DISABLE_TRACING
#include <artec/sdk/base/Log.h>

namespace artec { namespace sdk { namespace algorithms
{

#define AALGORITHMSDK_MESSAGE(severity, ...) { if (artec::sdk::base::getOutputLevel() >= severity) artec::sdk::base::report(severity, L"ALGO", __VA_ARGS__); }
#define AALGORITHMSDK_INFO(...)           AALGORITHMSDK_MESSAGE(artec::sdk::base::VerboseLevel_Info, __VA_ARGS__)
#define AALGORITHMSDK_ERROR(...)             AALGORITHMSDK_MESSAGE(artec::sdk::base::VerboseLevel_Error, __VA_ARGS__)
#define AALGORITHMSDK_WARNING(...)           AALGORITHMSDK_MESSAGE(artec::sdk::base::VerboseLevel_Warning, __VA_ARGS__)
#define AALGORITHMSDK_TRACE(...)             AALGORITHMSDK_MESSAGE(artec::sdk::base::VerboseLevel_Trace, __VA_ARGS__)

} } } // namespace artec::sdk::algorithms

#else

#define AALGORITHMSDK_MESSAGE(severity, ...) {}
#define AALGORITHMSDK_INFO(...)           {}
#define AALGORITHMSDK_ERROR(...)             {}
#define AALGORITHMSDK_WARNING(...)           {}
#define AALGORITHMSDK_TRACE(...)             {}

#endif

#endif // _AALGORITHMSDK_LOG_H_
