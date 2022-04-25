/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	log code
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ASCANNINGSDK_LOG_H_
#define _ASCANNINGSDK_LOG_H_

#ifndef ASCANNINGSDK_DISABLE_TRACING
#include <artec/sdk/base/Log.h>

namespace artec { namespace sdk { namespace scanning
{

#define ASCANNINGSDK_MESSAGE(severity, ...) { if (artec::sdk::base::getOutputLevel() >= severity) artec::sdk::base::report(severity, L"SCAN", __VA_ARGS__); }
#define ASCANNINGSDK_INFO(...)           ASCANNINGSDK_MESSAGE(artec::sdk::base::VerboseLevel_Info, __VA_ARGS__)
#define ASCANNINGSDK_ERROR(...)             ASCANNINGSDK_MESSAGE(artec::sdk::base::VerboseLevel_Error, __VA_ARGS__)
#define ASCANNINGSDK_WARNING(...)           ASCANNINGSDK_MESSAGE(artec::sdk::base::VerboseLevel_Warning, __VA_ARGS__)
#define ASCANNINGSDK_TRACE(...)             ASCANNINGSDK_MESSAGE(artec::sdk::base::VerboseLevel_Trace, __VA_ARGS__)

} } } // namespace artec::sdk::algorithms

#else

#define ASCANNINGSDK_MESSAGE(severity, ...) {}
#define ASCANNINGSDK_INFO(...)           {}
#define ASCANNINGSDK_ERROR(...)             {}
#define ASCANNINGSDK_WARNING(...)           {}
#define ASCANNINGSDK_TRACE(...)             {}

#endif

#endif // _ASCANNINGSDK_LOG_H_
