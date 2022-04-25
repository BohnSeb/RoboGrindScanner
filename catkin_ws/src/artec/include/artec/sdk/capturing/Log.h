/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	log code
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ACAPTURESDK_LOG_H_
#define _ACAPTURESDK_LOG_H_

#ifndef ACAPTURESDK_DISABLE_TRACING
#include <artec/sdk/base/Log.h>

namespace artec { namespace sdk { namespace capturing
{

#define ACAPTURESDK_MESSAGE(severity, ...) { if (artec::sdk::base::getOutputLevel() >= severity) artec::sdk::base::report(severity, L"CAPT", __VA_ARGS__); }
#define ACAPTURESDK_INFO(...)           ACAPTURESDK_MESSAGE(artec::sdk::base::VerboseLevel_Info, __VA_ARGS__)
#define ACAPTURESDK_ERROR(...)             ACAPTURESDK_MESSAGE(artec::sdk::base::VerboseLevel_Error, __VA_ARGS__)
#define ACAPTURESDK_WARNING(...)           ACAPTURESDK_MESSAGE(artec::sdk::base::VerboseLevel_Warning, __VA_ARGS__)
#define ACAPTURESDK_TRACE(...)             ACAPTURESDK_MESSAGE(artec::sdk::base::VerboseLevel_Trace, __VA_ARGS__)

} } } // namespace artec::sdk::capturing

#else

#define ACAPTURESDK_MESSAGE(severity, ...) {}
#define ACAPTURESDK_INFO(...)           {}
#define ACAPTURESDK_ERROR(...)             {}
#define ACAPTURESDK_WARNING(...)           {}
#define ACAPTURESDK_TRACE(...)             {}

#endif

#endif // _ACAPTURESDK_LOG_H_
