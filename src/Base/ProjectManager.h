/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PROJECT_MANAGER_H
#define CNOID_BASE_PROJECT_MANAGER_H

#include "ItemList.h"
#include <string>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Archive;

class CNOID_EXPORT ProjectManager
{
public:
     // This function is called before initializaing the App class
    static void setDefaultOptionToStoreLayoutInProjectFile(bool on);
    
    static void initializeClass(ExtensionManager* ext);
    static ProjectManager* instance();

    //The constructor used to create a sub instance for recursive loading / saving
    ProjectManager();
    
    ~ProjectManager();

    void clearProject();

    /*
      \param parentItem The item to add the root item of the project to load.
      If this argument is nullptr, the items of the existing project are cleared
      and the project is newly loaded.
    */
    void loadProject(const std::string& filename, Item* parentItem = nullptr);
    
    void loadBuiltinProject(const std::string& resourceFile, Item* parentItem = nullptr);
    bool isLoadingProject() const;
    void saveProject(const std::string& filename, Item* item = nullptr);
    void overwriteCurrentProject();
    std::string currentProjectFile() const;
    std::string currentProjectDirectory() const;
    void setCurrentProjectName(const std::string& filename);

    SignalProxy<void(int recursiveLevel)> sigProjectAboutToBeLoaded();
    SignalProxy<void(int recursiveLevel)> sigProjectLoaded();

private:
    ProjectManager(ExtensionManager* ext);

    class Impl;
    Impl* impl;

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
