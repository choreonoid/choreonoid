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
#include <cnoid/Format>
#include <filesystem>
#include <QLibrary>
#include <QFileDialog>
#include <vector>
#include <map>
#include <set>
#include <list>
#include <regex>

#ifdef Q_OS_WIN32
#include <QtGlobal>
#include <windows.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;


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

class PluginInfo;
typedef std::shared_ptr<PluginInfo> PluginInfoPtr;
typedef map<std::string, PluginInfoPtr> PluginMap;

class PluginInfo
{
public:
    QLibrary* dll;
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
        dll = nullptr;
        plugin = nullptr;
        status = PluginManager::NOT_LOADED;
        areAllRequisitiesResolved = false;
        doReloading = false;
        aboutMenuItem = nullptr;
        aboutDialog = nullptr;
    }

    ~PluginInfo(){
        if(dll){
            delete dll;
        }
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
    std::regex pluginNamePattern;

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

    bool addPluginDirectory(const std::string& directory, bool doMakeAbsolute);
    void loadPlugins(bool doActivation);
    void scanPluginFiles(const std::string& pathString, bool isUTF8, bool isRecursive);
    void loadScannedPluginFiles(bool doActivation);
    bool loadPlugin(int index, bool isLoadingMultiplePlugins);
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

    pluginNamePattern.assign(string(DLL_PREFIX) + "Cnoid(.+)Plugin" + DEBUG_SUFFIX + "\\." + DLL_EXTENSION);

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


bool PluginManager::addPluginDirectory(const std::string& directory)
{
    return impl->addPluginDirectory(fromUTF8(directory), true);
}


bool PluginManager::Impl::addPluginDirectory(const std::string& nativeDirectory, bool doMakeAbsolute)
{
    filesystem::path path(nativeDirectory);

    if(!path.is_absolute()){
        if(!doMakeAbsolute){
            return false;
        }
        path = filesystem::absolute(path);
    }

    std::error_code ec;
    if(!filesystem::is_directory(path, ec)){
        return false;
    }
    
    auto pathString = path.string();
    pluginDirectories.insert(pluginDirectories.begin(), toUTF8(pathString));
    
#ifdef Q_OS_WIN32
    // Add the plugin directory to PATH
    auto pathenv = QString::fromLocal8Bit(qgetenv("PATH"));
    qputenv("PATH", QString("%1;%2").arg(pathString.c_str()).arg(pathenv).toLocal8Bit());
#endif

    return true;
}


/**
   @param directory the install prefix of the corresponding plugin module.
*/
bool PluginManager::addPluginDirectoryAsPrefix(const std::string& prefix)
{
    return impl->addPluginDirectory(
        (filesystem::path(fromUTF8(prefix)) / CNOID_PLUGIN_SUBDIR).string(), false);
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
            string filename(toUTF8(pluginPath.filename().string()));
            std::smatch match;
            if(isNamingConventionCheckDisabled || regex_match(filename, match, pluginNamePattern)){
                auto it = pathToPluginInfoMap.find(pathStringUtf8);
                if(it == pathToPluginInfoMap.end()){
                    PluginInfoPtr info = std::make_shared<PluginInfo>();
                    // Set a tentative name extracted from the plugin file name
                    info->name = match.str(1);
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
                if(loadPlugin(i, true)){
                    ++numLoaded;
                } else {
                    ++numNotLoaded;
                }
            }
        }
        if(numLoaded == 0){
            if(numNotLoaded > 0){
                for(auto& plugin : allPluginInfos){
                    if(plugin->status == PluginManager::NOT_LOADED){
                        mout->putError(formatR(_("Loading {0} failed.\n"), plugin->name));
                        if(!plugin->lastErrorMessage.empty()){
                            mout->putErrorln(plugin->lastErrorMessage);
                        } else {
                            mout->flush();
                        }
                        plugin->status = PluginManager::INVALID;
                    }
                }
            }
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
                        mout->putErrorln(
                            formatR(_("{0}-plugin cannot be initialized because required plugin(s) {1} are not found."),
                                    info->name, lacks));
                    }
                }
            }
            break;
        }
    }
}


#ifdef Q_OS_WIN32
/**
   Finds missing dependency DLL by analyzing the PE import table.

   @param dllPath Path to the DLL file to analyze
   @return Name of the missing dependency DLL, or empty string if all dependencies are found

   This function is needed because QLibrary::errorString() only provides generic error messages
   (e.g., "Cannot load library") without specifying which dependency DLL is missing.

   Windows does not provide a standard API to retrieve the name of a missing dependency DLL
   when LoadLibrary fails. The standard approach using GetLastError() only returns a generic
   error code (e.g., ERROR_MOD_NOT_FOUND), without specifying which dependency is missing.

   While ntdll.dll provides LdrGetFailureData(), it is an undocumented internal API that
   proves unreliable in practice:
   - It does not record failures when the top-level DLL itself is not found
   - It only records failures of dependent DLLs, and even then inconsistently
   - Previous error data may persist if not manually cleared
   - It may be cleared by subsequent Windows API calls

   Therefore, we directly parse the PE (Portable Executable) file format to:
   1. Read the import directory from the PE header
   2. Enumerate all dependency DLLs listed in the import table
   3. Check each dependency using SearchPath() to find which one is missing

   This approach is more reliable as it does not depend on undocumented Windows internals.

   Note on RVA (Relative Virtual Address) conversion:
   PE files use RVAs which are memory addresses relative to the image base when loaded.
   To read the import table from the file on disk, we must convert RVAs to file offsets
   using the section headers, as the memory layout differs from the file layout.
*/
static std::string findMissingDependencyDLL(const std::string& dllPath)
{
    std::string missingDependency;

    int wideSize = MultiByteToWideChar(CP_UTF8, 0, dllPath.c_str(), -1, NULL, 0);
    if(wideSize > 0){
        std::vector<wchar_t> widePath(wideSize);
        MultiByteToWideChar(CP_UTF8, 0, dllPath.c_str(), -1, &widePath[0], wideSize);

        // Analyze the DLL file to find missing dependencies
        HANDLE hFile = CreateFileW(&widePath[0], GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
        if(hFile != INVALID_HANDLE_VALUE){
            HANDLE hMapping = CreateFileMappingW(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
            if(hMapping){
                LPVOID pFileBase = MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0);
                if(pFileBase){
                    IMAGE_DOS_HEADER* pDosHeader = (IMAGE_DOS_HEADER*)pFileBase;
                    if(pDosHeader->e_magic == IMAGE_DOS_SIGNATURE){
                        IMAGE_NT_HEADERS* pNtHeaders = (IMAGE_NT_HEADERS*)((BYTE*)pFileBase + pDosHeader->e_lfanew);
                        if(pNtHeaders->Signature == IMAGE_NT_SIGNATURE){
                            IMAGE_DATA_DIRECTORY* pImportDir = &pNtHeaders->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
                            if(pImportDir->Size > 0 && pImportDir->VirtualAddress > 0){
                                // Convert import directory RVA to file offset
                                IMAGE_SECTION_HEADER* pSectionHeader = IMAGE_FIRST_SECTION(pNtHeaders);
                                DWORD importFileOffset = 0;
                                bool found = false;

                                for(int i = 0; i < pNtHeaders->FileHeader.NumberOfSections; i++, pSectionHeader++){
                                    if(pImportDir->VirtualAddress >= pSectionHeader->VirtualAddress &&
                                       pImportDir->VirtualAddress < pSectionHeader->VirtualAddress + pSectionHeader->Misc.VirtualSize){
                                        importFileOffset = pImportDir->VirtualAddress - pSectionHeader->VirtualAddress + pSectionHeader->PointerToRawData;
                                        found = true;
                                        break;
                                    }
                                }

                                if(found && importFileOffset > 0){
                                    IMAGE_IMPORT_DESCRIPTOR* pImportDesc = (IMAGE_IMPORT_DESCRIPTOR*)((BYTE*)pFileBase + importFileOffset);

                                    // Iterate through all imported DLLs
                                    while(pImportDesc->Name != 0){
                                        // Convert DLL name RVA to file offset
                                        pSectionHeader = IMAGE_FIRST_SECTION(pNtHeaders);
                                        DWORD nameFileOffset = 0;
                                        for(int i = 0; i < pNtHeaders->FileHeader.NumberOfSections; i++, pSectionHeader++){
                                            if(pImportDesc->Name >= pSectionHeader->VirtualAddress &&
                                               pImportDesc->Name < pSectionHeader->VirtualAddress + pSectionHeader->Misc.VirtualSize){
                                                nameFileOffset = pImportDesc->Name - pSectionHeader->VirtualAddress + pSectionHeader->PointerToRawData;
                                                break;
                                            }
                                        }

                                        if(nameFileOffset > 0){
                                            const char* dllName = (const char*)((BYTE*)pFileBase + nameFileOffset);

                                            // Check if this dependency DLL can be found
                                            WCHAR searchPath[MAX_PATH];
                                            int searchWideSize = MultiByteToWideChar(CP_ACP, 0, dllName, -1, NULL, 0);
                                            if(searchWideSize > 0 && searchWideSize <= MAX_PATH){
                                                std::vector<wchar_t> dllNameWide(searchWideSize);
                                                MultiByteToWideChar(CP_ACP, 0, dllName, -1, &dllNameWide[0], searchWideSize);

                                                if(SearchPathW(NULL, &dllNameWide[0], NULL, MAX_PATH, searchPath, NULL) == 0){
                                                    // This dependency DLL is missing
                                                    missingDependency = dllName;
                                                    break;
                                                }
                                            }
                                        }
                                        pImportDesc++;
                                    }
                                }
                            }
                        }
                    }
                    UnmapViewOfFile(pFileBase);
                }
                CloseHandle(hMapping);
            }
            CloseHandle(hFile);
        }
    }

    return missingDependency;
}
#endif


bool PluginManager::loadPlugin(int index)
{
    if(impl->loadPlugin(index, false)){
        return impl->activatePlugin(index);
    }
    return false;
}


bool PluginManager::Impl::loadPlugin(int index, bool isLoadingMultiplePlugins)
{
    PluginInfoPtr& info = allPluginInfos[index];
    bool hasMessages = false;
    
    if(info->status == PluginManager::ACTIVE){
        mout->put(formatR(_("Plugin file \"{}\" has already been activated.\n"), info->pathString));
        hasMessages = true;

    } else if(info->status == PluginManager::NOT_LOADED){
        if(false){
            mout->put(formatR(_("Detecting plugin file \"{}\".\n"), info->pathString));
            hasMessages = true;
        }

        if(!info->dll){
            info->dll = new QLibrary;
            info->dll->setFileName(info->pathString.c_str());

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
                info->dll->setLoadHints(QLibrary::ExportExternalSymbolsHint);
            }
        }

        if(!(info->dll->load())){
            info->lastErrorMessage = formatR(_("System error: {0}"), info->dll->errorString().toStdString());

#ifdef Q_OS_WIN32
            // Try to identify which dependency DLL is missing
            std::string missingDependency = findMissingDependencyDLL(info->pathString);
            if(!missingDependency.empty()){
                info->lastErrorMessage += formatR(_("\nMissing dependency: {0}"), missingDependency);
            }
#endif

            if(isLoadingMultiplePlugins){
                /*
                  A plugin without the RPATH information may not be loaded if it is loaded before
                  the plugins it depends on are loaded. In such cases, loading the plugin again
                  after the other plugins have been loaded may allow it to be loaded.
                */
                delete info->dll; // QLibrary object must be deleted to retry loading
                info->dll = nullptr;
                return false;
            } else {
                mout->putErrorln(info->lastErrorMessage);
                info->status = PluginManager::INVALID;
            }
        } else {
            QFunctionPointer symbol = info->dll->resolve("getChoreonoidPlugin");
            if(!symbol){
                info->status = PluginManager::INVALID;
                info->lastErrorMessage = _("The plugin entry function \"getChoreonoidPlugin\" is not found.\n");
                info->lastErrorMessage += info->dll->errorString().toStdString() + "\n";
                mout->putError(info->lastErrorMessage);
                hasMessages = true;

            } else {
                Plugin::PluginEntry getCnoidPluginFunc = (Plugin::PluginEntry)(symbol);
                Plugin*& plugin = info->plugin;
                plugin = getCnoidPluginFunc();

                if(!plugin){
                    info->status = PluginManager::INVALID;
                    mout->putError(_("The plugin object cannot be created.\n"));
                    hasMessages = true;

                } else {
                    info->status = PluginManager::LOADED;
                    info->name = plugin->name();
                    plugin->setFilePath(info->pathString.c_str());

                    if(plugin->internalVersion() != CNOID_INTERNAL_VERSION){
                        mout->putWarning(
                            formatR(
                                _("The internal version of the {0} plugin is different from the system internal version.\n"
                                  "The plugin file \"{1}\" should be removed or updated to avoid a problem.\n"),
                                info->name, info->pathString));
                        hasMessages = true;
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
                formatR(_("Plugin file \"{0}\" conflicts with \"{1}\".\n"),
                        info->pathString, another->pathString);
            mout->putError(info->lastErrorMessage);
            hasMessages = true;
        }
    }

    if(info->status == PluginManager::LOADED){
        info->lastErrorMessage.clear();
    } else {
        mout->putError(_("Loading the plugin failed.\n"));
        hasMessages = true;
    }

    if(hasMessages){
        mout->flush();
    }

    return (info->status == PluginManager::LOADED);
}


bool PluginManager::Impl::activatePlugin(int index)
{
    string errorMessage;
    bool hasMessages = false;

    PluginInfoPtr info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mout->putWarning(formatR(_("Plugin file \"{}\" has already been activated.\n"), info->pathString));
        hasMessages = true;

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
                    formatR(_("About {} Plugin"), info->name),
                    [this, info]{ onAboutDialogTriggered(info); });
                
                // register old names
                int numOldNames = info->plugin->numOldNames();
                for(int i=0; i < numOldNames; ++i){
                    oldNameToCurrentPluginNameMap.insert(
                        make_pair(info->plugin->oldName(i), info->name));
                }
                
                mout->put(formatR(_("{}-plugin has been activated.\n"), info->name));
                hasMessages = true;
            }
        }
    }

    if(!errorMessage.empty()){
        mout->putErrorln(formatR(_("Loading the plugin failed.\n{0}"), errorMessage));
        hasMessages = true;
    }

    if(hasMessages){
        mout->flush();
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
                    mout->putErrorln(
                        formatR(_("{0}-plugin cannot be finalized."), info->name));
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
        if(info->dll->unload()){
            info->status = PluginManager::UNLOADED;
            nameToPluginInfoMap.erase(info->name);
            mout->putln(formatR(_("Plugin DLL \"{}\" has been unloaded."), info->pathString));
            if(info->doReloading){
                info->status = PluginManager::NOT_LOADED;
                info->doReloading = false;
                doReloading = true;
                delete info->dll;
                info->dll = nullptr;
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
        info->aboutDialog->setWindowTitle(formatR(_("About {} Plugin"), info->name).c_str());
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
