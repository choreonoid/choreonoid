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

class Plugin::Impl
{
public:
    Impl(const std::string& name);

    string name;
    string filePath;
    vector<string> requisites;
    vector<string> subsequences;
    vector<string> oldNames;
    int activationPriority;
    unsigned int internalVersion;
    bool isUnloadable;
};

}


Plugin::Plugin(const std::string& name)
    : ExtensionManager(name, true)
{
    impl = new Impl(name);
    isActive_ = false;
}


Plugin::Impl::Impl(const std::string& name)
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


const std::string& Plugin::name() const
{
    return impl->name;
}


const std::string& Plugin::filePath() const
{
    return impl->filePath;
}


void Plugin::setFilePath(const std::string& filePath)
{
    impl->filePath = filePath;
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
void Plugin::require(const std::string& pluginName)
{
    impl->requisites.push_back(pluginName);
}


#ifdef CNOID_BACKWARD_COMPATIBILITY
/**
   Deprecated. Please use require() instead of this function.
*/
void Plugin::depend(const std::string& pluginName)
{
    require(pluginName);
}
#endif


void Plugin::precede(const std::string& pluginName)
{
    impl->subsequences.push_back(pluginName);
}


const string& Plugin::requisite(int index) const
{
    return impl->requisites[index];
}


int Plugin::numRequisites() const
{
    return impl->requisites.size();
}


const std::string& Plugin::subsequence(int index) const
{
    return impl->subsequences[index];
}


int Plugin::numSubsequences() const
{
    return impl->subsequences.size();
}


void Plugin::addOldName(const std::string& name)
{
    impl->oldNames.push_back(name);
}


const std::string& Plugin::oldName(int index) const
{
    return impl->oldNames[index];
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
