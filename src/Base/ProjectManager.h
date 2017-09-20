/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PROJECT_MANAGER_H
#define CNOID_BASE_PROJECT_MANAGER_H

#include "Archive.h"
#include <string>
#include <functional>
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
    std::string currentProjectFile() const;
    std::string currentProjectDirectory() const;

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
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);

    void resetArchivers(const std::string& moduleName);
};

}

#endif
