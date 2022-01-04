/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PLUGIN_MANAGER_H
#define CNOID_BASE_PLUGIN_MANAGER_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class Plugin;

class CNOID_EXPORT PluginManager
{
public:
    static PluginManager* instance();
	
    ~PluginManager();

    void addPluginPath(const std::string& path);
    bool isStartupLoadingDisabled() const;
    void doStartupLoading();
    void loadPlugins(bool doActivation);
    bool loadPlugin(int index);
    bool reloadPlugin(const std::string& name);
    bool unloadPlugin(int index);
    bool unloadPlugin(const std::string& name);
    bool finalizePlugins();
    void clearUnusedPlugins();
    void flushMessagesTo(std::ostream& os);

    int numPlugins() const;
    const std::string& pluginPath(int index) const;
    const std::string& pluginName(int index) const;

    enum PluginStatus { NOT_LOADED, LOADED, ACTIVE, FINALIZED, UNLOADED, INVALID, CONFLICT };
    int pluginStatus(int index) const;
	
    Plugin* findPlugin(const std::string& name);
    const std::string& getErrorMessage(const std::string& name);
    const char* guessActualPluginName(const std::string& name);

    void showDialogToLoadPlugin();
	
private:
    PluginManager();

    class Impl;
    Impl* impl;
};

}

#endif
