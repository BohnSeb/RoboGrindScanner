#pragma once

namespace artec { namespace sdk { namespace base {

class IArrayUuid;

} } }

namespace artec { namespace sdk { namespace project {

/// Settings for a load operation
struct ProjectLoaderSettings
{
    base::IArrayUuid* entryList; ///< List of entries constituting a project
};

} } }
