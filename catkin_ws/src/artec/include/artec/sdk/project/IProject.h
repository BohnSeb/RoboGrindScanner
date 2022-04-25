/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Interface to load/save Arteс Studio projects
*
*	Copyright:	Artec Group
*
********************************************************************/
#pragma once

#include <stddef.h>
#include <artec/sdk/project/EntryInfo.h>
#include <artec/sdk/project/ProjectSdkDefines.h>

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base {

class IJob;
class IString;

} } }

namespace artec { namespace sdk { namespace project {

using namespace artec::sdk::base::errors;

using artec::sdk::base::IJob;
using artec::sdk::base::IString;

class IProject;

struct ProjectSettings;
struct ProjectLoaderSettings;
struct ProjectSaverSettings;
struct ProjectDeleterSettings;
struct ProjectCopierSettings;

extern "C"
{

/**
*   @brief    Create a new project
*   @param    pProject Destination project
*   @param    settings Project creation parameters, e.g. path
*   @return   Error code
*/
ErrorCode APROJECT_LINK_SPEC createNewProject(IProject** pProject, ProjectSettings* settings);

/**
*   @brief    Open an existing project
*   @param    pProject Destination project
*   @param    path Project path
*   @return   Error code
*/
ErrorCode APROJECT_LINK_SPEC openProject(IProject** pProject, const wchar_t* path);

/// @brief  Get maximum supported project version
/// @return All projects wits smaller or equal version will be read correctly
/// @note If the project version exceeds this value, the project can still be read. Some data, however, might be missing.  
int APROJECT_LINK_SPEC getMaximumSupportedProjectVersion();

}


/**
*   @brief    Interface for loading/saving Artec Studio projects
*   @details  Use it to fetch scans and ICompositeMesh's objects from Arteс Studio projects, assemble projects from scans and
*   models (composite meshes) and save them in Arteс Studio format.
*/
class IProject: public artec::sdk::base::IRef
{
public:
    /// @brief  Get current project version
    /// @return Current project version
    virtual int getProjectVersion() = 0;

    /// @{ @name Functions to create asynchronous job

    /// @brief  Create a job that loads project entries
    /// @param  pLoader A load job to be created
    /// @param  settings Settings for a load operation
    /// @return Error code
    virtual ErrorCode createLoader(IJob** pLoader, ProjectLoaderSettings* settings) = 0;
    /// @brief  Create a job that saves the project
    /// @param  pSaver A save job to be created
    /// @param  settings Settings for a save operation
    /// @return Error code
    virtual ErrorCode createSaver(IJob** pSaver, ProjectSaverSettings* settings) = 0;
    /// @brief  Create a job that deletes the project
    /// @note Not implemented yet
    /// @param  pDeleter A delete job to be created
    /// @param  settings Settings for a delete operation
    /// @return Error code
    virtual ErrorCode createDeleter(IJob** pDeleter, ProjectDeleterSettings* settings) = 0;
    /// @brief  Create a job that copies the project
    /// @note Not implemented yet
    /// @param  pCopier a copy job to be created
    /// @param  settings settings for a copy operation
    /// @return Error code
    virtual ErrorCode createCopier(IJob** pCopier, ProjectCopierSettings* settings) = 0;

    /// @}


    /// @{ @name Functions to manipulate project entry

    /// @brief  Get the number of entries (scans/models) available in the project
    /// @return A total number of entries (as stated in metadata) available in the project
    virtual int getEntryCount() const = 0;
    /// @brief  Get an information structure for an entry specified by an index
    /// @param  entryIndex A zero-based entry index [0,..,N-1]
    /// @param  entry An entry structure to be filled in
    /// @param  entryName An entry name to be filled in. Can be NULL if the name is not necessary
    /// @return Error code
    virtual ErrorCode getEntry(int entryIndex, EntryInfo* entry, IString** entryName = NULL) const = 0;

    /// @}
}; // class IProject

} } }