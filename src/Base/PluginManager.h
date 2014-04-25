/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PLUGIN_MANAGER_H_INCLUDED
#define CNOID_BASE_PLUGIN_MANAGER_H_INCLUDED

#include <string>

namespace cnoid {

class ExtensionManager;
class PluginManagerImpl;

class PluginManager
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

    enum PluginStatus { NOT_LOADED, LOADED, ACTIVE, FINALIZED, INVALID, CONFLICT };
    int pluginStatus(int index) const;
	
    bool loadPlugin(int index);
    bool unloadPlugin(int index);

    const char* guessActualPluginName(const std::string& name);
	
private:
    PluginManager(ExtensionManager* ext);

    PluginManagerImpl* impl;
};
};


#endif
