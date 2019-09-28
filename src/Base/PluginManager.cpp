/**
   @author Shin'ichiro Nakaoka
*/

#include "PluginManager.h"
#include "Plugin.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "DescriptionDialog.h"
#include "LazyCaller.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/Tokenizer>
#include <cnoid/Config>
#include <QLibrary>
#include <QRegExp>
#include <QFileDialog>
#include <vector>
#include <map>
#include <set>
#include <list>
#include <iostream>
#include <fmt/format.h>

#ifdef Q_OS_WIN32
#include <QtGlobal>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
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
PluginManager* instance_ = 0;
}

namespace cnoid {

class PluginManagerImpl
{
public:

    PluginManagerImpl(ExtensionManager* ext);
    ~PluginManagerImpl();

    Action* startupLoadingCheck;
    Action* namingConventionCheck;
        
    MessageView* mv;

    string pluginDirectory;
    QRegExp pluginNamePattern;

    struct PluginInfo;
    typedef std::shared_ptr<PluginInfo> PluginInfoPtr;
    typedef map<std::string, PluginInfoPtr> PluginMap;

    struct PluginInfo {
        PluginInfo(){
            plugin = 0;
            status = PluginManager::NOT_LOADED;
            areAllRequisitiesResolved = false;
            doReloading = false;
            aboutMenuItem = 0;
            aboutDialog = 0;
        }
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
    };

    typedef vector<PluginInfoPtr> PluginInfoArray;

    PluginInfoArray allPluginInfos;
    PluginMap nameToPluginInfoMap;
    PluginMap pathToPluginInfoMap;

    typedef multimap<std::string, std::string> MultiNameMap;
    MultiNameMap oldNameToCurrentPluginNameMap;

    typedef map<std::string, int> CountMap;
    CountMap precedentCountMap;

    // Finalization should be done in the inverse order
    typedef list<PluginInfoPtr> PluginInfoList;
    PluginInfoList pluginsInDeactivationOrder;

    PluginInfoArray pluginsToUnload;
    LazyCaller unloadPluginsLater;
    LazyCaller reloadPluginsLater;
    
    void clearUnusedPlugins();
    void scanPluginFilesInDefaultPath(const std::string& pathList);
    void scanPluginFilesInDirectoyOfExecFile();
    void scanPluginFiles(const std::string& pathString, bool isRecursive);
    void loadPlugins();
    void unloadPluginsActually();
    bool finalizePlugins();
    bool loadPlugin(int index);
    bool activatePlugin(int index);
    void onLoadPluginTriggered();
    void onAboutDialogTriggered(PluginInfo* info);
    const char* guessActualPluginName(const std::string& name);
    bool unloadPlugin(const std::string& name, bool doReloading);
    bool unloadPlugin(int index);
    bool finalizePlugin(PluginInfoPtr info);
};

}


static bool comparePluginInfo(const PluginManagerImpl::PluginInfoPtr& p, const PluginManagerImpl::PluginInfoPtr& q)
{
    int priority1 = p->plugin ? p->plugin->activationPriority() : std::numeric_limits<int>::min();
    int priority2 = q->plugin ? q->plugin->activationPriority() : std::numeric_limits<int>::min();

    if(priority1 == priority2){
        return (p->name < q->name);
    } else {
        return (priority1 < priority2);
    }
}


void PluginManager::initialize(ExtensionManager* ext)
{
    if(!instance_){
        instance_ = new PluginManager(ext);
    }
}


PluginManager* PluginManager::instance()
{
    return instance_;
}


void PluginManager::finalize()
{
    if(instance_){
        delete instance_;
        instance_ = 0;
    }
}


PluginManager::PluginManager(ExtensionManager* ext)
{
    impl = new PluginManagerImpl(ext);
}


PluginManagerImpl::PluginManagerImpl(ExtensionManager* ext)
    : mv(MessageView::mainInstance()),
      unloadPluginsLater(std::bind(&PluginManagerImpl::unloadPluginsActually, this), LazyCaller::PRIORITY_LOW),
      reloadPluginsLater(std::bind(&PluginManagerImpl::loadPlugins, this), LazyCaller::PRIORITY_LOW)
{
    pluginDirectory = getNativePathString(
        filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR);

#ifdef Q_OS_WIN32
    // Add the plugin directory to PATH
    auto newPath = QString("%1;%2").arg(pluginDirectory.c_str()).arg(qEnvironmentVariable("PATH"));
    qputenv("PATH", newPath.toLocal8Bit());
#endif

    pluginNamePattern.setPattern(
        QString(DLL_PREFIX) + "Cnoid.+Plugin" + DEBUG_SUFFIX + "\\." + DLL_EXTENSION);

    // for the base module
    PluginInfoPtr info = std::make_shared<PluginInfo>();
    info->name = "Base";
    nameToPluginInfoMap.insert(make_pair(string("Base"), info));

    auto config = AppConfig::archive()->openMapping("PluginManager");

    MenuManager& mm = ext->menuManager();
    mm.setPath("/File").setPath(N_("Plugin"));
    mm.addItem(_("Load Plugin"))
        ->sigTriggered().connect(std::bind(&PluginManagerImpl::onLoadPluginTriggered, this));

    mm.addSeparator();

    startupLoadingCheck = mm.addCheckItem(_("Startup Plugin Loading"));
    startupLoadingCheck->setChecked(config->get("startupPluginLoading", true));

    namingConventionCheck = mm.addCheckItem(_("Check the naming convention of plugin files"));
    namingConventionCheck->setChecked(config->get("checkPluginfileNamingConvention", true));
    
    mm.addSeparator();
}


PluginManager::~PluginManager()
{
    delete impl;
}


PluginManagerImpl::~PluginManagerImpl()
{
    finalizePlugins();

    auto config = AppConfig::archive()->openMapping("PluginManager");
    config->write("startupPluginLoading", startupLoadingCheck->isChecked());
    config->write("checkPluginfileNamingConvention", namingConventionCheck->isChecked());
}


int PluginManager::numPlugins() const
{
    return impl->allPluginInfos.size();
}


const std::string& PluginManager::pluginPath(int index) const
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
    PluginManagerImpl::PluginMap::iterator p = impl->nameToPluginInfoMap.find(name);
    if(p != impl->nameToPluginInfoMap.end()){
        return p->second->plugin;
    }
    return 0;
}


void PluginManager::doStartupLoading(const char* pluginPathList)
{
    if(impl->startupLoadingCheck->isChecked()){
        if(pluginPathList){
            scanPluginFilesInPathList(pluginPathList);
        }
        scanPluginFilesInDirectoyOfExecFile();
        loadPlugins();
    }
}


/**
   This function scans plugin files in the path list.
   @param pathList List of the directories to scan, which is specified with the same format
   as the PATH environment varibale.
*/
void PluginManager::scanPluginFilesInPathList(const std::string& pathList)
{
    impl->scanPluginFilesInDefaultPath(pathList);
}


void PluginManagerImpl::scanPluginFilesInDefaultPath(const std::string& pathList)
{
    for(auto& path : Tokenizer<CharSeparator<char>>(pathList, CharSeparator<char>(PATH_DELIMITER))){
        scanPluginFiles(path, false);
    }
}


void PluginManager::scanPluginFilesInDirectoyOfExecFile()
{
    impl->scanPluginFilesInDirectoyOfExecFile();
}


void PluginManagerImpl::scanPluginFilesInDirectoyOfExecFile()
{
    scanPluginFiles(pluginDirectory, false);
} 


void PluginManager::scanPluginFiles(const std::string& pathString)
{
    impl->scanPluginFiles(pathString, false);
}


void PluginManagerImpl::scanPluginFiles(const std::string& pathString, bool isRecursive)
{
    filesystem::path pluginPath(pathString);

    if(filesystem::exists(pluginPath)){
        if(filesystem::is_directory(pluginPath)){
            if(!isRecursive){

                static const bool doSorting = false;
                filesystem::directory_iterator end;
                if(doSorting){
                    list<string> paths;
                    for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                        paths.push_back(getNativePathString(*it));
                    }
                    paths.sort();
                    for(list<string>::iterator p = paths.begin(); p != paths.end(); ++p){
                        scanPluginFiles(*p, true);
                    }
                } else {
                    for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                        scanPluginFiles(getNativePathString(*it), true);
                    }
                }
            }
        } else {
            QString filename(getFilename(pluginPath).c_str());
            if(!namingConventionCheck->isChecked() || pluginNamePattern.exactMatch(filename)){
                PluginMap::iterator p = pathToPluginInfoMap.find(pathString);
                if(p == pathToPluginInfoMap.end()){
                    PluginInfoPtr info = std::make_shared<PluginInfo>();
                    info->pathString = pathString;
                    allPluginInfos.push_back(info);
                    pathToPluginInfoMap[pathString] = info;
                }
            }
        }
    }
}


void PluginManager::clearUnusedPlugins()
{
    impl->clearUnusedPlugins();
}


void PluginManagerImpl::clearUnusedPlugins()
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


void PluginManager::loadPlugins()
{
    impl->loadPlugins();
}


void PluginManagerImpl::loadPlugins()
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
                        mv->putln(fmt::format(_("{0}-plugin cannot be initialized because required plugin(s) {1} are not found."),
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


bool PluginManagerImpl::loadPlugin(int index)
{
    PluginInfoPtr& info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mv->putln(fmt::format(_("Plugin file \"{}\" has already been activated."), info->pathString));

    } else if(info->status == PluginManager::NOT_LOADED){
        mv->putln(fmt::format(_("Detecting plugin file \"{}\""), info->pathString));

        info->dll.setFileName(info->pathString.c_str());

        // Some Python modules written in C/C++ requires the following options
        // to link with the python runtime library, but this might cause the symbol conflicts
        // amaong internal use libraries such as OPCODE used in both the Body module and ODE.
        info->dll.setLoadHints(QLibrary::ExportExternalSymbolsHint);
        
        //info->dll.setLoadHints(QLibrary::ResolveAllSymbolsHint);
        //info->dll.setLoadHints(0);
        
        if(!(info->dll.load())){
            mv->putln(MessageView::ERROR, info->dll.errorString());

        } else {
            QFunctionPointer symbol = info->dll.resolve("getChoreonoidPlugin");
            if(!symbol){
                info->status = PluginManager::INVALID;
                mv->putln(MessageView::ERROR, _("The plugin entry function \"getChoreonoidPlugin\" is not found."));
                mv->putln(MessageView::ERROR, info->dll.errorString());

            } else {
                Plugin::PluginEntry getCnoidPluginFunc = (Plugin::PluginEntry)(symbol);
                Plugin*& plugin = info->plugin;
                plugin = getCnoidPluginFunc();

                if(!plugin){
                    info->status = PluginManager::INVALID;
                    mv->putln(MessageView::ERROR, _("The plugin object cannot be created."));

                } else {

                    info->status = PluginManager::LOADED;
                    info->name = plugin->name();

                    if(plugin->internalVersion() != CNOID_INTERNAL_VERSION){
                        mv->putln(
                            fmt::format(
                                _("The internal version of the {0} plugin is different from the system internal version.\n"
                                  "The plugin file \"{1}\" should be removed or updated to avoid a problem."),
                                info->name, info->pathString),
                            MessageView::WARNING);
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

                    PluginMap::iterator p = nameToPluginInfoMap.find(info->name);
                    if(p == nameToPluginInfoMap.end()){
                        nameToPluginInfoMap.insert(make_pair(info->name, info));
                    } else {
                        info->status = PluginManager::CONFLICT;
                        PluginInfoPtr& another = p->second;
                        another->status = PluginManager::CONFLICT;
                        mv->putln(MessageView::ERROR,
                                  fmt::format(_("Plugin file \"{0}\" conflicts with \"{1}\"."),
                                  info->pathString, another->pathString));
                    }
                }
            }
        }
    }

    if(info->status != PluginManager::LOADED){
        mv->putln(MessageView::ERROR, _("Loading the plugin failed."));
        mv->flush();
    }

    return (info->status == PluginManager::LOADED);
}


bool PluginManagerImpl::activatePlugin(int index)
{
    QString errorMessage;

    PluginInfoPtr& info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mv->putln(fmt::format(_("Plugin file \"{}\" has already been activated."), info->pathString));

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
                info->aboutMenuItem =
                    info->plugin->menuManager().setPath("/Help").setPath(_("About Plugins"))
                    .addItem(fmt::format(_("About {} Plugin"), info->name).c_str());
                info->aboutMenuItem->sigTriggered().connect(
                    std::bind(&PluginManagerImpl::onAboutDialogTriggered, this, info.get()));
                
                // register old names
                int numOldNames = info->plugin->numOldNames();
                for(int i=0; i < numOldNames; ++i){
                    oldNameToCurrentPluginNameMap.insert(
                        make_pair(info->plugin->oldName(i), info->name));
                }
                
                mv->putln(fmt::format(_("{}-plugin has been activated."), info->name));
                mv->flush();
                ExtensionManager::notifySystemUpdate();
            }
        }
    }

    if(!errorMessage.isEmpty()){
        mv->putln(_("Loading the plugin failed."));
        mv->putln(errorMessage);
        mv->flush();
    }

    return (info->status == PluginManager::ACTIVE);
}


void PluginManagerImpl::onLoadPluginTriggered()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Load plugins"));
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << QString(_("Plugin files (*.%1)")).arg(DLL_EXTENSION);
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    MappingPtr config = AppConfig::archive()->openMapping("PluginManager");
    dialog.setDirectory(config->get("pluginLoadingDialogDirectory", executableTopDirectory()).c_str());
    
    if(dialog.exec()){
        config->writePath("pluginLoadingDialogDirectory", dialog.directory().absolutePath().toStdString());
        QStringList filenames = dialog.selectedFiles();
        for(int i=0; i < filenames.size(); ++i){
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));

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
        loadPlugins();
    }
}


void PluginManagerImpl::onAboutDialogTriggered(PluginInfo* info)
{
    if(!info->aboutDialog){
        info->aboutDialog = new DescriptionDialog();
        info->aboutDialog->setWindowTitle(fmt::format(_("About {} Plugin"), info->name).c_str());
        info->aboutDialog->setDescription(info->plugin->description());
    }

    info->aboutDialog->show();
}


const char* PluginManager::guessActualPluginName(const std::string& name)
{
    return impl->guessActualPluginName(name);
}


const char* PluginManagerImpl::guessActualPluginName(const std::string& name)
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

    return 0;
}


bool PluginManager::unloadPlugin(const std::string& name)
{
    return impl->unloadPlugin(name, false);
}


bool PluginManager::reloadPlugin(const std::string& name)
{
    return impl->unloadPlugin(name, true);
}


bool PluginManagerImpl::unloadPlugin(const std::string& name, bool doReloading)
{
    PluginMap::iterator p = nameToPluginInfoMap.find(name);
    if(p != nameToPluginInfoMap.end()){
        PluginManagerImpl::PluginInfoPtr info = p->second;
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


bool PluginManager::unloadPlugin(int index)
{
    return impl->unloadPlugin(index);
}


bool PluginManagerImpl::unloadPlugin(int index)
{
    PluginInfoPtr& info = allPluginInfos[index];
    if(info->plugin && info->plugin->isUnloadable()){
        return finalizePlugin(info);
    }
    return false;
}


bool PluginManagerImpl::finalizePlugin(PluginInfoPtr info)
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
                if(!info->plugin->finalize()){
                    mv->putln(fmt::format(_("Plugin {} cannot be finalized."), info->name));
                    mv->flush();
                } else {
                    bool isUnloadable = info->plugin->isUnloadable();
                    info->status = PluginManager::FINALIZED;
                    if(info->aboutMenuItem){
                        delete info->aboutMenuItem;
                        info->aboutMenuItem = 0;
                    }
                    delete info->plugin;
                    info->plugin = 0;
                    if(info->aboutDialog){
                        delete info->aboutDialog;
                        info->aboutDialog = 0;
                    }
                    ExtensionManager::notifySystemUpdate();

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


void PluginManagerImpl::unloadPluginsActually()
{
    bool doReloading = false;
    
    for(size_t i=0; i < pluginsToUnload.size(); ++i){
        PluginInfoPtr& info = pluginsToUnload[i];
        if(info->dll.unload()){
            info->status = PluginManager::UNLOADED;
            nameToPluginInfoMap.erase(info->name);
            mv->putln(fmt::format(_("Plugin dll {} has been unloaded."), info->pathString));
            mv->flush();
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


bool PluginManagerImpl::finalizePlugins()
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
    
    for(size_t i=0; i < allPluginInfos.size(); ++i){
        PluginInfoPtr& info = allPluginInfos[i];
        if(info->status == PluginManager::ACTIVE){
            if(!finalizePlugin(info)){
                failed = true;
            }
        }
    }
    return !failed;
}
