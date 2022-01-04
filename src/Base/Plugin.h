/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PLUGIN_H
#define CNOID_BASE_PLUGIN_H

#include "ExtensionManager.h"
#include <cnoid/Config>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;
class View;
class ToolBar;
class PluginManager;
class AppCustomizationUtil;

class CNOID_EXPORT Plugin : public ExtensionManager
{
public:
    typedef Plugin* (*PluginEntry)();

    Plugin(const std::string& name);
    virtual ~Plugin();

    const std::string& name() const;
    const std::string& filePath() const;

    virtual bool customizeApplication(AppCustomizationUtil& app);
    virtual bool initialize();
    virtual bool finalize();

    bool isActive() const { return isActive_; }
    bool isUnloadable() const;

    const std::string& requisite(int index) const;
    int numRequisites() const;

    const std::string& subsequence(int index) const;
    int numSubsequences() const;

    const std::string& oldName(int index) const;
    int numOldNames() const;

    virtual const char* description() const;

    int activationPriority() const;

    unsigned int internalVersion() const;

    // Only called from the getChoreonoidPlugin() function
    void setInternalVersion(unsigned int version);

protected:
    void setPluginScope(Item* item);
    void setPluginScope(View* view);
    void setPluginScope(ToolBar* toolBar);

    void setUnloadable(bool on);

    void require(const std::string& pluginName);
    void precede(const std::string& pluginName);

    /**
       Call this function in the constructor if necessary.
       @param priority
       A smaller value means a higher priority.
       The default value is the maximum integer value.
       The value 0 is set for fundamental plugins which should be initialized before extra plugins.
    */
    void setActivationPriority(int priority);

    /**
       When the plugin name is changed but the old project files should be loadable,
       specify old names of the plugin with this function in the constructor.
    */
    void addOldName(const std::string& name);

#ifdef CNOID_BACKWARD_COMPATIBILITY
    void depend(const std::string& pluginName);
#endif

    static const char* MITLicenseText();
    static const char* LGPLtext();

private:
    Plugin(const Plugin& org); // disable the copy constructor
    void setFilePath(const std::string& filePath);

    class Impl;
    Impl* impl;
    bool isActive_;

    friend class PluginManager;
};

}


#define CNOID_IMPLEMENT_PLUGIN_ENTRY(PluginTypeName)                    \
    extern "C" CNOID_BASE_DLLEXPORT cnoid::Plugin* getChoreonoidPlugin() \
    {                                                                   \
        cnoid::Plugin* plugin = new PluginTypeName();                   \
        plugin->setInternalVersion(CNOID_INTERNAL_VERSION); \
        return plugin;            \
    }

#endif
