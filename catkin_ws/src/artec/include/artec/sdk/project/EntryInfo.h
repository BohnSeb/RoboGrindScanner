#pragma once

#include "EntryType.h"

#include <artec/sdk/base/Uuid.h>
#include <artec/sdk/base/IString.h>

namespace artec { namespace sdk { namespace project {

/// Information on project entry (scan or model), e.g. its type and identifier
struct EntryInfo
{
    base::Uuid uuid; ///< Unique identifier
    EntryType type; ///< Entry type 
    unsigned long long size; ///< Entry size in bytes
}; // struct EntryInfo

} } }