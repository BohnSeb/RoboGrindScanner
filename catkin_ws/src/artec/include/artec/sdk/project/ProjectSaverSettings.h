#pragma once

#include <artec/sdk/base/Uuid.h>

namespace artec { namespace sdk { namespace project {


/// Settings for a save operation
struct ProjectSaverSettings
{
    const wchar_t* path; ///< Path to save to
    base::Uuid projectId; ///< Project unique identifier
    int compressionLevel; ///< Project compression level
};

} } }
