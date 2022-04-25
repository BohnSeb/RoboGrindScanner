/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	log code
*
*	Copyright:	Artec Group
*
********************************************************************/
#pragma once

#ifndef ADATASDK_DISABLE_TRACING
#include <artec/sdk/base/Log.h>

#define ADATASDK_MESSAGE(severity, ...) { if (artec::sdk::base::getOutputLevel() >= severity) artec::sdk::base::report(severity, L"DATA", __VA_ARGS__); }
#define ADATASDK_INFO(...)           ADATASDK_MESSAGE(artec::sdk::base::VerboseLevel_Info, __VA_ARGS__)
#define ADATASDK_ERROR(...)             ADATASDK_MESSAGE(artec::sdk::base::VerboseLevel_Error, __VA_ARGS__)
#define ADATASDK_WARNING(...)           ADATASDK_MESSAGE(artec::sdk::base::VerboseLevel_Warning, __VA_ARGS__)
#define ADATASDK_TRACE(...)             ADATASDK_MESSAGE(artec::sdk::base::VerboseLevel_Trace, __VA_ARGS__)

#else

#define ADATASDK_MESSAGE(severity, ...) {}
#define ADATASDK_INFO(...)           {}
#define ADATASDK_ERROR(...)             {}
#define ADATASDK_WARNING(...)           {}
#define ADATASDK_TRACE(...)             {}

#endif
