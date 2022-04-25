/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	log code
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ABASESDK_LOG_H_
#define _ABASESDK_LOG_H_

#ifndef ABASESDK_DISABLE_TRACING
#include <artec/sdk/base/BaseSdkDefines.h>
#include <wctype.h>
#include <cstdarg>

namespace artec { namespace sdk { namespace base
{

typedef int (* REPORT_HOOK)(int, const wchar_t *);

const int RPTHOOK_INSTALL = 0;
const int RPTHOOK_REMOVE = 1;

/// The level of detail of verbose output
enum VerboseLevel
{
	VerboseLevel_Disable = -1,	///< Noting to output
	VerboseLevel_Error,			///< Output errors
	VerboseLevel_Warning,		///< Output errors and warnings
	VerboseLevel_Info,			///< Output errors, warnings and information
	VerboseLevel_Trace,			///< Comprehensive output
	VerboseLevel_Number
};

extern "C"
{

void ABASESDK_LINK_SPEC setOutputLevel( int mode );

int ABASESDK_LINK_SPEC getOutputLevel( );

bool ABASESDK_LINK_SPEC setReportHook( 
		int mode, 
		REPORT_HOOK newHook 
		);

void ABASESDK_LINK_SPEC report(
	int verboseLevel,
	const wchar_t * module,
	const wchar_t * format,
	...);

void ABASESDK_LINK_SPEC	vReport(
	int verboseLevel,
	const wchar_t * module,
	const wchar_t * format,
	va_list args);

}

#define ABASESDK_MESSAGE(severity, ...) { if (getOutputLevel() >= severity) report(severity, L"BASE", __VA_ARGS__); }
#define ABASESDK_INFO(...)           ABASESDK_MESSAGE(VerboseLevel_Info, __VA_ARGS__)
#define ABASESDK_ERROR(...)             ABASESDK_MESSAGE(VerboseLevel_Error, __VA_ARGS__)
#define ABASESDK_WARNING(...)           ABASESDK_MESSAGE(VerboseLevel_Warning, __VA_ARGS__)
#define ABASESDK_TRACE(...)             ABASESDK_MESSAGE(VerboseLevel_Trace, __VA_ARGS__)

} } } // namespace artec::sdk::base

#else

#define ABASESDK_MESSAGE(severity, ...) {}
#define ABASESDK_INFO(...)           {}
#define ABASESDK_ERROR(...)             {}
#define ABASESDK_WARNING(...)           {}
#define ABASESDK_TRACE(...)             {}

#endif

#endif // _ABASESDK_LOG_H_
