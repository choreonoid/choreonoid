/**
   @author Shin'ichiro Nakaoka
*/

#include "Plugin.h"
#include "Item.h"
#include "ToolBar.h"
#include "View.h"
#include "Licenses.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class PluginImpl
{
public:
    PluginImpl(const char* name);

    string name;
    vector<string> requisites;
    vector<string> subsequences;
    vector<string> oldNames;
    int activationPriority;
    unsigned int internalVersion;
    bool isUnloadable;
};

}


Plugin::Plugin(const char* name)
    : ExtensionManager(name, true)
{
    impl = new PluginImpl(name);
}


PluginImpl::PluginImpl(const char* name)
    : name(name)
{
    activationPriority = std::numeric_limits<int>::max();
    internalVersion = 0;
    isUnloadable = false;
}


Plugin::~Plugin() 
{
    delete impl;
}


const char* Plugin::name()
{
    return impl->name.c_str();
}


void Plugin::setInternalVersion(unsigned int version)
{
    impl->internalVersion = version;
}


unsigned int Plugin::internalVersion() const
{
    return impl->internalVersion;
}


bool Plugin::initialize()
{
    return true;
}


bool Plugin::finalize()
{
    return true;
}


bool Plugin::isUnloadable() const
{
    return impl->isUnloadable;
}


void Plugin::setUnloadable(bool on)
{
    impl->isUnloadable = on;
}


/**
   When the plugin depends on some other plugins,
   please specify the plugins to depend with this function in the constructor.
*/
void Plugin::require(const char* pluginName)
{
    impl->requisites.push_back(pluginName);
}


#ifdef CNOID_BACKWARD_COMPATIBILITY
/**
   Deprecated. Please use require() instead of this function.
*/
void Plugin::depend(const char* pluginName)
{
    require(pluginName);
}
#endif


void Plugin::precede(const char* pluginName)
{
    impl->subsequences.push_back(pluginName);
}


const char* Plugin::requisite(int index) const
{
    return impl->requisites[index].c_str();
}


int Plugin::numRequisites() const
{
    return impl->requisites.size();
}


const char* Plugin::subsequence(int index) const
{
    return impl->subsequences[index].c_str();
}


int Plugin::numSubsequences() const
{
    return impl->subsequences.size();
}


void Plugin::addOldName(const char* name)
{
    impl->oldNames.push_back(name);
}


const char* Plugin::oldName(int index) const
{
    return impl->oldNames[index].c_str();
}


int Plugin::numOldNames() const
{
    return impl->oldNames.size();
}


const char* Plugin::description() const
{
    return "";
}


const char* Plugin::MITLicenseText()
{
    return cnoid::MITLicenseText();
}


const char* Plugin::LGPLtext()
{
    return cnoid::LGPLtext();
}


int Plugin::activationPriority() const
{
    return impl->activationPriority;
}


void Plugin::setActivationPriority(int priority)
{
    impl->activationPriority = priority;
}
