#pragma once

namespace artec { namespace sdk { namespace base {

class IArrayUuid;

} } }

namespace artec { namespace sdk { namespace project {

/// Settings for a delete operation
struct ProjectDeleterSettings
{
    base::IArrayUuid* entryList; ///< List of entries constituting a project
};

} } }