#pragma once

namespace artec { namespace sdk { namespace project {

/// Type of the project entry (scan or model)
enum EntryType
{
    EntryType_Scan = 0x0, ///< Entry is a scan
    EntryType_CompositeMesh = 0x1, ///< Entry is a 3D mesh model

    EntryType_Unknown = 0xFFFFFFFF ///< Entry type is unknown
}; // enum EntryType

} } }
