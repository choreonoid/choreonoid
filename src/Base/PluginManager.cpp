#include "PluginManager.h"
#include "Plugin.h"
#include "MainMenu.h"
#include "DescriptionDialog.h"
#include "LazyCaller.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "Action.h"
#include <cnoid/MessageOut>
#include <cnoid/ValueTree>
#include <cnoid/ExecutablePath>
#include <cnoid/Tokenizer>
#include <cnoid/Config>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QLibrary>
#include <QRegExp>
#include <QFileDialog>
#include <vector>
#include <map>
#include <set>
#include <list>
#include <fmt/format.h>

#ifdef Q_OS_WIN32
#include <QtGlobal>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;


#ifdef Q_OS_WIN32
# ifdef CNOID_DEBUG
static const char* DEBUG_SUFFIX = "d";
# else
static const char* DEBUG_SUFFIX = "";
# endif
#else
# ifdef Q_OS_MAC
static const char* DEBUG_SUFFIX = "";
# else
static const char* DEBUG_SUFFIX = "";
# endif
#endif

namespace {

PluginManager* instance_ = nullptr;

struct PluginInfo;
typedef std::shared_ptr<PluginInfo> PluginInfoPtr;
typedef map<std::string, PluginInfoPtr> PluginMap;

struct PluginInfo
{
    QLibrary dll;
    std::string pathString;
    Plugin* plugin;
    string name;
    vector<string> requisites;
    vector<string> dependents;
    set<string> subsequences;
    int status;
    bool areAllRequisitiesResolved;
    bool doReloading;
    Action* aboutMenuItem;
    DescriptionDialog* aboutDialog;
    string lastErrorMessage;

    PluginInfo(){
        plugin = nullptr;
        status = PluginManager::NOT_LOADED;
        areAllRequisitiesResolved = false;
        doReloading = false;
        aboutMenuItem = nullptr;
        aboutDialog = nullptr;
    }
};

}

namespace cnoid {

class PluginManager::Impl
{
public:
    Impl();
    ~Impl();

    bool isStartupLoadingDisabled;
    bool isNamingConventionCheckDisabled;

    MessageOut* mout;
    MainMenu* mainMenu;

    string pluginDirectory;
    vector<string> pluginDirectories;
    QRegExp pluginNamePattern;

    vector<PluginInfoPtr> allPluginInfos;
    PluginMap nameToPluginInfoMap;
    PluginMap pathToPluginInfoMap;

    typedef multimap<std::string, std::string> MultiNameMap;
    MultiNameMap oldNameToCurrentPluginNameMap;

    typedef map<std::string, int> CountMap;
    CountMap precedentCountMap;

    // Finalization should be done in the inverse order
    typedef list<PluginInfoPtr> PluginInfoList;
    PluginInfoList pluginsInDeactivationOrder;

    vector<PluginInfoPtr> pluginsToUnload;
    LazyCaller unloadPluginsLater;
    LazyCaller reloadPluginsLater;

    void addPluginDirectory(const std::string& directory, bool doMakeAbsolute);
    void loadPlugins(bool doActivation);
    void scanPluginFiles(const std::string& pathString, bool isUTF8, bool isRecursive);
    void loadScannedPluginFiles(bool doActivation);
    bool loadPlugin(int index);
    bool activatePlugin(int index);
    bool unloadPlugin(int index);
    bool unloadPlugin(const std::string& name, bool doReloading);
    bool finalizePlugin(PluginInfoPtr info);
    void unloadPluginsActually();
    bool finalizePlugins();
    void clearUnusedPlugins();
    const char* guessActualPluginName(const std::string& name);
    void onAboutDialogTriggered(PluginInfoPtr info);
    void showDialogToLoadPlugin();
};

}


static bool comparePluginInfo(const PluginInfoPtr& p, const PluginInfoPtr& q)
{
    int priority1 = p->plugin ? p->plugin->activationPriority() : std::numeric_limits<int>::min();
    int priority2 = q->plugin ? q->plugin->activationPriority() : std::numeric_limits<int>::min();

    if(priority1 == priority2){
        return (p->name < q->name);
    } else {
        return (priority1 < priority2);
    }
}


PluginManager* PluginManager::instance()
{
    if(!instance_){
        instance_ = new PluginManager;
    }
    return instance_;
}


PluginManager::PluginManager()
{
    impl = new Impl;
}


PluginManager::Impl::Impl()
    : unloadPluginsLater([&](){ unloadPluginsActually(); }, LazyCaller::LowPriority),
      reloadPluginsLater([&](){ loadScannedPluginFiles(true); }, LazyCaller::LowPriority)
{
    mout = MessageOut::master();
    mainMenu = nullptr;

    addPluginDirectory(cnoid::pluginDir(), false);

    pluginNamePattern.setPattern(
        QString(DLL_PREFIX) + "Cnoid(.+)Plugin" + DEBUG_SUFFIX + "\\." + DLL_EXTENSION);

    // for the base module
    PluginInfoPtr info = std::make_shared<PluginInfo>();
    info->name = "Base";
    nameToPluginInfoMap.insert(make_pair(string("Base"), info));

    auto config = AppConfig::archive()->openMapping("PluginManager");
    
    // The following options are used for debug and can only be specified in the config file.
    isStartupLoadingDisabled = config->get("disable_startup_loading", false);
    isNamingConventionCheckDisabled = config->get("disable_naming_convention_check", false);
}


PluginManager::~PluginManager()
{
    delete impl;
    instance_ = nullptr;
}


PluginManager::Impl::~Impl()
{
    finalizePlugins();
}


/**
   @param pathList semicolon or colon separeted absolute path list.
*/
void PluginManager::addPluginPathList(const std::string& pathList)
{
    for(auto& dir : Tokenizer<CharSeparator<char>>(pathList, CharSeparator<char>(PATH_DELIMITER))){
        impl->addPluginDirectory(dir, false);
    }
}


void PluginManager::addPluginPath(const std::string& pathList)
{
    addPluginPathList(pathList);
}


void PluginManager::addPluginDirectory(const std::string& directory)
{
    impl->addPluginDirectory(directory, true);
}


void PluginManager::Impl::addPluginDirectory(const std::string& directory, bool doMakeAbsolute)
{
    string directoryFromUTF8;
    if(doMakeAbsolute){
        directoryFromUTF8 = fromUTF8(directory);
        filesystem::path path(directoryFromUTF8);
        if(!path.is_absolute()){
            string absDirectory = filesystem::absolute(path).string();
            pluginDirectories.insert(pluginDirectories.begin(), toUTF8(absDirectory));
#ifdef Q_OS_WIN32
            // Add the plugin directory to PATH
            qputenv("PATH", format("{0};{1}", absDirectory, qgetenv("PATH")).c_str());
#endif
            return;
        }
    }
    
    pluginDirectories.insert(pluginDirectories.begin(), directory);
    
#ifdef Q_OS_WIN32
    // Add the plugin directory to PATH
    if(directoryFromUTF8.empty()){
        directoryFromUTF8 = fromUTF8(directory);
    }
    qputenv("PATH", format("{0};{1}", directoryFromUTF8, qgetenv("PATH")).c_str());
#endif
}


/**
   @param directory the install prefix of the corresponding plugin module.
*/
void PluginManager::addPluginDirectoryAsPrefix(const std::string& prefix)
{
    addPluginDirectory(
        toUTF8((filesystem::path(fromUTF8(prefix)) / CNOID_PLUGIN_SUBDIR).string()));
}


const std::vector<std::string> PluginManager::pluginDirectories() const
{
    return impl->pluginDirectories;
}


bool PluginManager::isStartupLoadingDisabled() const
{
    return impl->isStartupLoadingDisabled;
}


void PluginManager::doStartupLoading()
{
    if(!impl->isStartupLoadingDisabled){
        impl->loadPlugins(true);
    }
}


void PluginManager::loadPlugins(bool doActivation)
{
    impl->loadPlugins(doActivation);
}


void PluginManager::Impl::loadPlugins(bool doActivation)
{
    if(allPluginInfos.empty()){ // Not scanned yet
        for(auto& dir : pluginDirectories){
            scanPluginFiles(dir, true, false);
        }
    }
    loadScannedPluginFiles(doActivation);
}


void PluginManager::Impl::scanPluginFiles(const std::string& pathString, bool isUTF8, bool isRecursive)
{
    filesystem::path pluginPath;
    if(isUTF8){
        pluginPath = fromUTF8(pathString);
    } else {
        pluginPath = pathString;
    }

    if(filesystem::exists(pluginPath)){
        if(filesystem::is_directory(pluginPath)){
            if(!isRecursive){
                static const bool doSorting = false;
                filesystem::directory_iterator end;
                if(doSorting){
                    list<string> paths;
                    for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                        auto path(it->path());
                        paths.push_back(path.make_preferred().string());
                    }
                    paths.sort();
                    for(list<string>::iterator p = paths.begin(); p != paths.end(); ++p){
                        scanPluginFiles(*p, false, true);
                    }
                } else {
                    for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                        auto path(it->path());
                        scanPluginFiles(path.make_preferred().string(), false, true);
                    }
                }
            }
        } else {
            const string* pPathStringUtf8; //pluginPath;
            string tmpPathStringUtf8;
            if(isUTF8){
                pPathStringUtf8 = &pathString;
            } else {
                tmpPathStringUtf8 = toUTF8(pathString);
                pPathStringUtf8 = &tmpPathStringUtf8;
            }
            const string& pathStringUtf8 = *pPathStringUtf8;
            QString filename(toUTF8(pluginPath.filename().string()).c_str());
            if(isNamingConventionCheckDisabled || pluginNamePattern.exactMatch(filename)){
                PluginMap::iterator p = pathToPluginInfoMap.find(pathStringUtf8);
                if(p == pathToPluginInfoMap.end()){
                    PluginInfoPtr info = std::make_shared<PluginInfo>();
                    // Set a tentative name extracted from the plugin file name
                    info->name = pluginNamePattern.cap(1).toStdString();
                    info->pathString = pathStringUtf8;
                    allPluginInfos.push_back(info);
                    pathToPluginInfoMap[info->pathString] = info;
                }
            }
        }
    }
}


void PluginManager::Impl::loadScannedPluginFiles(bool doActivation)
{
    while(true){
        int numLoaded = 0;
        int numNotLoaded = 0;
        for(size_t i=0; i < allPluginInfos.size(); ++i){
            if(allPluginInfos[i]->status == PluginManager::NOT_LOADED){
                if(loadPlugin(i)){
                    ++numLoaded;
                } else {
                    ++numNotLoaded;
                }
            }
        }
        if(numLoaded == 0 || numNotLoaded == 0){
            break;
        }
    }

    if(!doActivation){
        return;
    }

    std::sort(allPluginInfos.begin(), allPluginInfos.end(), comparePluginInfo);
    
    size_t totalNumActivated = 0;
    while(true){
        size_t numActivated = 0;
        for(size_t i=0; i < allPluginInfos.size(); ++i){
            int status = allPluginInfos[i]->status;
            if(status == PluginManager::LOADED){
                if(activatePlugin(i)){
                    numActivated++;
                    totalNumActivated++;
                }
            }
        }
        if(numActivated == 0 || totalNumActivated == allPluginInfos.size()){
            if(allPluginInfos.size() - totalNumActivated > 0){
                // Put information about plugins which cannot find required plugins
                for(size_t i=0; i < allPluginInfos.size(); ++i){
                    PluginInfoPtr& info = allPluginInfos[i];
                    if(info->status == PluginManager::LOADED && !info->areAllRequisitiesResolved){
                        string lacks;
                        int n = 0;
                        for(size_t j=0; j < info->requisites.size(); ++j){
                            PluginMap::iterator q = nameToPluginInfoMap.find(info->requisites[j]);
                            if(q == nameToPluginInfoMap.end()){
                                if(n++ > 0){
                                    lacks += ", ";
                                }
                                lacks += info->requisites[j];
                            }
                        }
                        mout->putError(
                            format(_("{0}-plugin cannot be initialized because required plugin(s) {1} are not found.\n"),
                                   info->name, lacks));
                    }
                }
            }
            break;
        }
    }
}


bool PluginManager::loadPlugin(int index)
{
    if(impl->loadPlugin(index)){
        return impl->activatePlugin(index);
    }
    return false;
}


bool PluginManager::Impl::loadPlugin(int index)
{
    PluginInfoPtr& info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mout->put(fmt::format(_("Plugin file \"{}\" has already been activated.\n"), info->pathString));

    } else if(info->status == PluginManager::NOT_LOADED){
        if(false){
            mout->put(fmt::format(_("Detecting plugin file \"{}\".\n"), info->pathString));
        }

        info->dll.setFileName(info->pathString.c_str());

        /*
          If some plugins do not work correctly due to a dynamic link problem, the problem might be solved
          by enabling the following option. However, this option might cause the symbol conflicts amaong
          internally used libraries and the functions provided by plugins may not be able to work correctly.
          Note that this options is originally required to make the Python plugin able to import binary
          Python modules on Linux. However, it is now handled inside the Python plugin, and it is not
          neccessary to enable the option to use Python modules.
        */
        char* CNOID_EXPORT_PLUGIN_EXTERNAL_SYMBOLS = getenv("CNOID_EXPORT_PLUGIN_EXTERNAL_SYMBOLS");
        if(CNOID_EXPORT_PLUGIN_EXTERNAL_SYMBOLS && (strcmp(CNOID_EXPORT_PLUGIN_EXTERNAL_SYMBOLS, "1") == 0)){
            info->dll.setLoadHints(QLibrary::ExportExternalSymbolsHint);
        }

        if(!(info->dll.load())){
            info->lastErrorMessage = fmt::format(_("System error: {0}"), info->dll.errorString().toStdString());
            mout->putErrorln(info->lastErrorMessage);
            info->status = PluginManager::INVALID;

        } else {
            QFunctionPointer symbol = info->dll.resolve("getChoreonoidPlugin");
            if(!symbol){
                info->status = PluginManager::INVALID;
                info->lastErrorMessage = _("The plugin entry function \"getChoreonoidPlugin\" is not found.\n");
                info->lastErrorMessage += info->dll.errorString().toStdString();
                mout->putErrorln(info->lastErrorMessage);

            } else {
                Plugin::PluginEntry getCnoidPluginFunc = (Plugin::PluginEntry)(symbol);
                Plugin*& plugin = info->plugin;
                plugin = getCnoidPluginFunc();

                if(!plugin){
                    info->status = PluginManager::INVALID;
                    mout->putError(_("The plugin object cannot be created.\n"));

                } else {
                    info->status = PluginManager::LOADED;
                    info->name = plugin->name();
                    plugin->setFilePath(info->pathString.c_str());

                    if(plugin->internalVersion() != CNOID_INTERNAL_VERSION){
                        mout->putWarning(
                            fmt::format(
                                _("The internal version of the {0} plugin is different from the system internal version.\n"
                                  "The plugin file \"{1}\" should be removed or updated to avoid a problem.\n"),
                                info->name, info->pathString));
                    }
                        
                    const int numRequisites = plugin->numRequisites();
                    for(int i=0; i < numRequisites; ++i){
                        info->requisites.push_back(plugin->requisite(i));
                    }

                    const int numSubsequences = plugin->numSubsequences();
                    for(int i=0; i < numSubsequences; ++i){
                        const string subsequence(plugin->subsequence(i));
                        CountMap::iterator p = precedentCountMap.find(subsequence);
                        if(p == precedentCountMap.end()){
                            precedentCountMap.insert(make_pair(subsequence, 1));
                        } else {
                            p->second += 1;
                        }
                        info->subsequences.insert(subsequence);
                    }
                }
            }
        }

        PluginMap::iterator p = nameToPluginInfoMap.find(info->name);
        if(p == nameToPluginInfoMap.end()){
            nameToPluginInfoMap.insert(make_pair(info->name, info));
        } else {
            info->status = PluginManager::CONFLICT;
            PluginInfoPtr& another = p->second;
            another->status = PluginManager::CONFLICT;
            info->lastErrorMessage =
                fmt::format(_("Plugin file \"{0}\" conflicts with \"{1}\"."),
                            info->pathString, another->pathString);
            mout->putErrorln(info->lastErrorMessage);
        }
    }

    if(info->status == PluginManager::LOADED){
        info->lastErrorMessage.clear();
    } else {
        mout->putError(_("Loading the plugin failed.\n"));
    }

    return (info->status == PluginManager::LOADED);
}


bool PluginManager::Impl::activatePlugin(int index)
{
    string errorMessage;

    PluginInfoPtr info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mout->putWarning(fmt::format(_("Plugin file \"{}\" has already been activated.\n"), info->pathString));

    } else if(info->status == PluginManager::LOADED){

        bool requisitesActive = true;

        CountMap::iterator p = precedentCountMap.find(info->name);
        if(p != precedentCountMap.end()){
            if(p->second > 0){
                // precedent plugins should be initialized before this
                return false;
            }
        }

        // check whether all the required plugins have already been active
        for(size_t i=0; i < info->requisites.size(); ++i){
            const string& requisiteName = info->requisites[i];
            PluginMap::iterator q = nameToPluginInfoMap.find(requisiteName);
            if(q == nameToPluginInfoMap.end()){
                requisitesActive = false;
                break;
            }
            PluginInfoPtr& requisite = q->second;
            if(info->subsequences.find(requisiteName) != info->subsequences.end()){
                if(requisite->status != PluginManager::LOADED){
                    requisitesActive = false;
                    break;
                }
            } else if(requisite->status != PluginManager::ACTIVE){
                requisitesActive = false;
                break;
            }
        }

        if(requisitesActive){

            info->areAllRequisitiesResolved = true;
                
            if(!info->plugin->initialize()){
                info->status = PluginManager::INVALID;
                errorMessage = _("The plugin object cannot be intialized.");

            } else {
                info->status = PluginManager::ACTIVE;
                info->plugin->isActive_ = true;
                
                pluginsInDeactivationOrder.push_front(info);

                // add this plugin to dependent list of the requisites
                for(size_t i=0; i < info->requisites.size(); ++i){
                    PluginMap::iterator p = nameToPluginInfoMap.find(info->requisites[i]);
                    if(p != nameToPluginInfoMap.end()){
                        p->second->dependents.push_back(info->name);
                    }
                }

                // decreate the count of subsequent pulgins
                for(set<string>::iterator p = info->subsequences.begin(); p != info->subsequences.end(); ++p){
                    const string& pluginName = *p;
                    precedentCountMap[pluginName] -= 1;
                }
                
                // set an about dialog
                if(!mainMenu){
                    mainMenu = MainMenu::instance();
                }
                mainMenu->add_Help_AboutPlugins_Item(
                    fmt::format(_("About {} Plugin"), info->name),
                    [this, info]{ onAboutDialogTriggered(info); });
                
                // register old names
                int numOldNames = info->plugin->numOldNames();
                for(int i=0; i < numOldNames; ++i){
                    oldNameToCurrentPluginNameMap.insert(
                        make_pair(info->plugin->oldName(i), info->name));
                }
                
                mout->put(fmt::format(_("{}-plugin has been activated.\n"), info->name));
            }
        }
    }

    if(!errorMessage.empty()){
        mout->putErrorln(format(_("Loading the plugin failed.\n{0}"), errorMessage));
    }

    return (info->status == PluginManager::ACTIVE);
}


bool PluginManager::reloadPlugin(const std::string& name)
{
    return impl->unloadPlugin(name, true);
}


bool PluginManager::unloadPlugin(int index)
{
    return impl->unloadPlugin(index);
}


bool PluginManager::Impl::unloadPlugin(int index)
{
    PluginInfoPtr& info = allPluginInfos[index];
    if(info->plugin && info->plugin->isUnloadable()){
        return finalizePlugin(info);
    }
    return false;
}


bool PluginManager::unloadPlugin(const std::string& name)
{
    return impl->unloadPlugin(name, false);
}



bool PluginManager::Impl::unloadPlugin(const std::string& name, bool doReloading)
{
    PluginMap::iterator p = nameToPluginInfoMap.find(name);
    if(p != nameToPluginInfoMap.end()){
        auto info = p->second;
        size_t index = 0;
        while(index < allPluginInfos.size()){
            if(allPluginInfos[index] == info){
                info->doReloading = doReloading;
                return unloadPlugin(index);
            }
            ++index;
        }
    }
    return false;
}


bool PluginManager::Impl::finalizePlugin(PluginInfoPtr info)
{
    if(info->status == PluginManager::ACTIVE){

        if(info->plugin){
            bool allDependentsFinalized = true;
            for(size_t i=0; i < info->dependents.size(); ++i){
                PluginMap::iterator p = nameToPluginInfoMap.find(info->dependents[i]);
                if(p != nameToPluginInfoMap.end()){
                    PluginInfoPtr& dependentInfo = p->second;
                    if(dependentInfo->status == PluginManager::ACTIVE){
                        allDependentsFinalized = false;
                        break;
                    }
                }
            }

            if(allDependentsFinalized){
                info->plugin->isActive_ = false;
                if(!info->plugin->finalize()){
                    MessageOut::master()->putError(
                        fmt::format(_("{0}-plugin cannot be finalized.\n"), info->name));
                } else {
                    bool isUnloadable = info->plugin->isUnloadable();
                    info->status = PluginManager::FINALIZED;
                    if(info->aboutMenuItem){
                        delete info->aboutMenuItem;
                        info->aboutMenuItem = nullptr;
                    }
                    delete info->plugin;
                    info->plugin = 0;
                    if(info->aboutDialog){
                        delete info->aboutDialog;
                        info->aboutDialog = nullptr;
                    }

                    if(isUnloadable){
                        pluginsToUnload.push_back(info);
                        unloadPluginsLater();
                    }
                }
            }
        }
    }
    
    return (info->status == PluginManager::FINALIZED);
}


void PluginManager::Impl::unloadPluginsActually()
{
    bool doReloading = false;

    for(size_t i=0; i < pluginsToUnload.size(); ++i){
        PluginInfoPtr& info = pluginsToUnload[i];
        if(info->dll.unload()){
            info->status = PluginManager::UNLOADED;
            nameToPluginInfoMap.erase(info->name);
            mout->put(fmt::format(_("Plugin DLL \"{}\" has been unloaded.\n"), info->pathString));
            if(info->doReloading){
                info->status = PluginManager::NOT_LOADED;
                info->doReloading = false;
                doReloading = true;
            }
        }
    }
    pluginsToUnload.clear();

    if(doReloading){
        reloadPluginsLater();
    }
}


bool PluginManager::finalizePlugins()
{
    return impl->finalizePlugins();
}


bool PluginManager::Impl::finalizePlugins()
{
    bool failed = false;

    bool finalized;
    do {
        finalized = false;
        PluginInfoList::iterator p = pluginsInDeactivationOrder.begin();
        while(p != pluginsInDeactivationOrder.end()){
            PluginInfoPtr& info = *p;
            if(info->status != PluginManager::ACTIVE){
                p = pluginsInDeactivationOrder.erase(p);
            } else if(finalizePlugin(info)){
                finalized = true;
                p = pluginsInDeactivationOrder.erase(p);
            } else {
                ++p;
            }
        }
    } while(finalized);

    return !failed;
}


void PluginManager::clearUnusedPlugins()
{
    impl->clearUnusedPlugins();
}


void PluginManager::Impl::clearUnusedPlugins()
{
    vector<PluginInfoPtr> oldList = allPluginInfos;
    allPluginInfos.clear();

    for(size_t i=0; i < oldList.size(); ++i){
        PluginInfoPtr& info = oldList[i];
        if(info->status == PluginManager::ACTIVE){
            allPluginInfos.push_back(info);
        } else {
            pathToPluginInfoMap.erase(info->pathString);
        }
    }
}


int PluginManager::numPlugins() const
{
    return impl->allPluginInfos.size();
}


const std::string& PluginManager::pluginFile(int index) const
{
    return impl->allPluginInfos[index]->pathString;
}


const std::string& PluginManager::pluginName(int index) const
{
    return impl->allPluginInfos[index]->name;
}


int PluginManager::pluginStatus(int index) const
{
    return impl->allPluginInfos[index]->status;
}


Plugin* PluginManager::findPlugin(const std::string& name)
{
    auto p = impl->nameToPluginInfoMap.find(name);
    if(p != impl->nameToPluginInfoMap.end()){
        return p->second->plugin;
    }
    return nullptr;
}

const std::string& PluginManager::getErrorMessage(const std::string& name)
{
    auto p = impl->nameToPluginInfoMap.find(name);
    if(p != impl->nameToPluginInfoMap.end()){
        return p->second->lastErrorMessage;
    } else {
        static const string message;
        return message;
    }
}
    

const char* PluginManager::guessActualPluginName(const std::string& name)
{
    return impl->guessActualPluginName(name);
}


const char* PluginManager::Impl::guessActualPluginName(const std::string& name)
{
    PluginMap::iterator p = nameToPluginInfoMap.find(name);
    if(p != nameToPluginInfoMap.end()){
        return p->second->name.c_str();
    }

    MultiNameMap::iterator q, upper_bound;
    std::pair<MultiNameMap::iterator, MultiNameMap::iterator> range =
        oldNameToCurrentPluginNameMap.equal_range(name);
    for(MultiNameMap::iterator q = range.first; q != range.second; ++q){
        const string& candidate = q->second;
        PluginMap::iterator r = nameToPluginInfoMap.find(candidate);
        if(r != nameToPluginInfoMap.end()){
            return r->second->name.c_str();
        }
    }

    return nullptr;
}


void PluginManager::Impl::onAboutDialogTriggered(PluginInfoPtr info)
{
    if(!info->aboutDialog){
        info->aboutDialog = new DescriptionDialog;
        info->aboutDialog->setWindowTitle(fmt::format(_("About {} Plugin"), info->name).c_str());
        info->aboutDialog->setDescription(info->plugin->description());
    }

    info->aboutDialog->show();
}


void PluginManager::showDialogToLoadPlugin()
{
    impl->showDialogToLoadPlugin();
}


void PluginManager::Impl::showDialogToLoadPlugin()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setOptions(QFileDialog::DontUseNativeDialog);
    dialog.setWindowTitle(_("Load plugins"));
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Load"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << QString(_("Plugin files (*.%1)")).arg(DLL_EXTENSION);
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    MappingPtr config = AppConfig::archive()->openMapping("PluginManager");
    dialog.setDirectory(config->get("pluginLoadingDialogDirectory", executableTopDir()).c_str());
    
    if(dialog.exec()){
        config->writePath("pluginLoadingDialogDirectory", dialog.directory().absolutePath().toStdString());
        QStringList filenames = dialog.selectedFiles();
        for(int i=0; i < filenames.size(); ++i){
            auto path = filesystem::path(fromUTF8(filenames[i].toStdString()));
            string filename = toUTF8(path.make_preferred().string());

            // This code was taken from 'scanPluginFiles'. This should be unified.
            //if(pluginNamePattern.exactMatch(QString(getFilename(pluginPath).c_str()))){
            if(true){
                PluginMap::iterator p = pathToPluginInfoMap.find(filename);
                if(p == pathToPluginInfoMap.end()){
                    PluginInfoPtr info(new PluginInfo);
                    info->pathString = filename;
                    allPluginInfos.push_back(info);
                    pathToPluginInfoMap[filename] = info;
                }
            }
        }
        loadScannedPluginFiles(true);
    }
}
