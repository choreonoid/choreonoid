/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PLUGIN_MANAGER_H
#define CNOID_BASE_PLUGIN_MANAGER_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class Plugin;
class ExtensionManager;

class CNOID_EXPORT PluginManager
{
public:

    static void initialize(ExtensionManager* ext);
    static PluginManager* instance();
    static void finalize();
	
    ~PluginManager();

    void doStartupLoading(const char* pluginPathList);
    void scanPluginFilesInPathList(const std::string& pathList);
    void scanPluginFilesInDirectoyOfExecFile();
    void scanPluginFiles(const std::string& pathString);
    void clearUnusedPlugins();
    void loadPlugins();
    bool finalizePlugins();

    int numPlugins() const;

    const std::string& pluginPath(int index) const;
    const std::string& pluginName(int index) const;

    enum PluginStatus { NOT_LOADED, LOADED, ACTIVE, FINALIZED, UNLOADED, INVALID, CONFLICT };
    int pluginStatus(int index) const;
	
    Plugin* findPlugin(const std::string& name);
    const std::string& getErrorMessage(const std::string& name);

    bool loadPlugin(int index);
    bool unloadPlugin(int index);
    bool unloadPlugin(const std::string& name);
    bool reloadPlugin(const std::string& name);

    const char* guessActualPluginName(const std::string& name);
	
private:
    PluginManager(ExtensionManager* ext);

    class Impl;
    Impl* impl;
};

}

#endif
