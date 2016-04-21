/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PROJECT_MANAGER_H_INCLUDED
#define CNOID_BASE_PROJECT_MANAGER_H_INCLUDED

#include "Archive.h"
#include <string>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class ProjectManagerImpl;

class CNOID_EXPORT ProjectManager
{
public:
    static ProjectManager* instance();
        
    void loadProject(const std::string& filename);
    void saveProject(const std::string& filename);
    void overwriteCurrentProject();
    const std::string& getProjectFileName();

    static void initialize(ExtensionManager* em);

private:
    ProjectManager(ExtensionManager* em);
    ~ProjectManager();

    ProjectManagerImpl* impl;

    friend class ExtensionManager;
    friend class ExtensionManagerImpl;

    void setArchiver(
        const std::string& moduleName,
        const std::string& objectName,
        boost::function<bool(Archive&)> storeFunction,
        boost::function<void(const Archive&)> restoreFunction);

    void resetArchivers(const std::string& moduleName);
};

}

#endif
