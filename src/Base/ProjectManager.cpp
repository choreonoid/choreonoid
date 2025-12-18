#include "ProjectManager.h"
#include "RootItem.h"
#include "SubProjectItem.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "ToolBar.h"
#include "Archive.h"
#include "ItemTreeArchiver.h"
#include "ExtensionManager.h"
#include "OptionManager.h"
#include "AppConfig.h"
#include "AppUtil.h"
#include "FileDialog.h"
#include "MainWindow.h"
#include "CheckBox.h"
#include <cnoid/MessageOut>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <filesystem>
#include <QResource>
#include <QMessageBox>
#include <string>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

bool defaultLayoutInclusionMode = true;
bool isLayoutInclusionMode = true;
bool isTemporaryItemSaveCheckAvailable = true;
int projectBeingLoadedCounter = 0;
MainWindow* mainWindow = nullptr;
vector<string> projectFilesToLoad;

class SaveDialog : public FileDialog
{
public:
    CheckBox temporaryItemSaveCheck;

    SaveDialog(ProjectManager::Impl* manager);
    std::string getSaveFilename();
};

}

namespace cnoid {

ProjectManager* ProjectManager::instance_ = nullptr;

class ProjectManager::Impl
{
public:
    Impl(ProjectManager* self);
    Impl(ProjectManager* self, ExtensionManager* ext);
    ~Impl();

    void setCurrentProjectFile(const string& filename);
    void clearCurrentProjectFile();
    void clearProject();
        
    template <class TObject>
    bool restoreObjectStates(
        Archive* projectArchive, Archive* states, const vector<TObject*>& objects, const char* nameSuffix, MessageOut* mout);

    ItemList<> loadProject(
        const std::string& filename, Item* parentItem,
        bool isInvokingApplication, bool isBuiltinProject, bool doClearExistingProject);

    void setItemsConsistentWithProjectArchive(Item* item);

    template<class TObject>
    bool storeObjects(Archive& parentArchive, const char* key, vector<TObject*> objects);
        
    bool saveProject(const string& filename, Item* item, bool doSaveTemporaryItems, bool isBackupMode);
        
    void onProjectOptionsParsed();
    void onInputFileOptionsParsed(std::vector<std::string>& inputFiles);
    bool onSaveDialogAboutToFinish(int result);
    bool confirmToCloseProject(bool isAboutToLoadNewProject);
    static bool checkValidItemExistence(Item* item);
    static bool checkIfItemsConsistentWithProjectArchive(Item* item, bool isTopItem = true);

    void connectArchiver(
        const std::string& name,
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);

    ProjectManager* self;
    ItemTreeArchiver itemTreeArchiver;
    string currentProjectName;
    string currentProjectFile;
    string currentProjectDirectory;

    struct ArchiverInfo {
        std::function<bool(Archive&)> storeFunction;
        std::function<void(const Archive&)> restoreFunction;
    };
    typedef map<string, ArchiverInfo> ArchiverMap;
    typedef map<string, ArchiverMap> ArchiverMapMap;
    ArchiverMapMap archivers;

    MappingPtr config;

    SaveDialog* saveDialog;

    Signal<void()> sigProjectCleared;
    Signal<void(int recursiveLevel)> sigProjectAboutToBeLoaded;
    Signal<void(int recursiveLevel)> sigProjectLoaded;

    bool isMainInstance;
};

}


void ProjectManager::setDefaultLayoutInclusionMode(bool on)
{
    defaultLayoutInclusionMode = on;
}


void ProjectManager::setDefaultOptionToStoreLayoutInProjectFile(bool on)
{
    defaultLayoutInclusionMode = on;
}


void ProjectManager::setTemporaryItemSaveCheckAvailable(bool on)
{
    isTemporaryItemSaveCheckAvailable = on;

    if(instance_){
        if(auto dialog = instance_->impl->saveDialog){
            dialog->temporaryItemSaveCheck.setVisible(on);
            if(!on){
                dialog->temporaryItemSaveCheck.setChecked(false);
            }
        }
    }
}


void ProjectManager::initializeClass(ExtensionManager* ext)
{
    if(!instance_){
        instance_ = ext->manage(new ProjectManager(ext));
        mainWindow = MainWindow::instance();
    }
}


ProjectManager::ProjectManager(ExtensionManager* ext)
{
    impl = new Impl(this, ext);
}


// The constructor for the main instance
ProjectManager::Impl::Impl(ProjectManager* self, ExtensionManager* ext)
    : Impl(self)
{
    config = AppConfig::archive()->openMapping("ProjectManager");
    ::isLayoutInclusionMode = config->get({ "include_layout", "store_perspective" }, defaultLayoutInclusionMode);
    saveDialog = nullptr;
    isMainInstance = true;

    auto om = OptionManager::instance();
    om->add_option("--project", projectFilesToLoad, "load a project file");
    om->sigInputFileOptionsParsed().connect(
        [this](std::vector<std::string>& inputFiles){ onInputFileOptionsParsed(inputFiles); });
    om->sigOptionsParsed().connect(
        [this](OptionManager*){ onProjectOptionsParsed(); });
}


ProjectManager::ProjectManager()
{
    impl = new Impl(this);
}


ProjectManager::Impl::Impl(ProjectManager* self)
    : self(self)
{
    saveDialog = nullptr;
    isMainInstance = false;
}


ProjectManager::~ProjectManager()
{
    delete impl;
}


ProjectManager::Impl::~Impl()
{
    if(isMainInstance){
        config->write("include_layout", ::isLayoutInclusionMode);
    }
    if(saveDialog){
        delete saveDialog;
    }
}


bool ProjectManager::isLoadingProject() const
{
    return projectBeingLoadedCounter > 0;
}


void ProjectManager::setCurrentProjectName(const std::string& name)
{
    impl->currentProjectName = name;
    mainWindow->setProjectTitle(name);

    if(!impl->currentProjectFile.empty()){
        auto path = filesystem::path(fromUTF8(impl->currentProjectFile));
        impl->currentProjectFile = toUTF8((path.parent_path() / (name + ".cnoid")).string());
    }
}
        

void ProjectManager::Impl::setCurrentProjectFile(const string& filename)
{
    if(filename.empty()){
        currentProjectName.clear();
        currentProjectFile.clear();
        currentProjectDirectory.clear();
    } else {
        filesystem::path path(fromUTF8(filename));
        auto name = toUTF8(path.stem().string());
        currentProjectName = name;

        path = filesystem::absolute(path).lexically_normal();

        currentProjectFile = toUTF8(path.string());
        currentProjectDirectory = toUTF8(path.parent_path().string());
    }

    mainWindow->setProjectTitle(currentProjectName);
}


void ProjectManager::Impl::clearCurrentProjectFile()
{
    currentProjectFile.clear();
}


const std::string& ProjectManager::currentProjectName() const
{
    return impl->currentProjectName;
}


const std::string& ProjectManager::currentProjectFile() const
{
    return impl->currentProjectFile;
}


const std::string& ProjectManager::currentProjectDirectory() const
{
    return impl->currentProjectDirectory;
}


bool ProjectManager::isLayoutInclusionMode() const
{
    return ::isLayoutInclusionMode;
}


void ProjectManager::setLayoutInclusionMode(bool on)
{
    ::isLayoutInclusionMode = on;
}


SignalProxy<void()> ProjectManager::sigProjectCleared()
{
    return impl->sigProjectCleared;
}


SignalProxy<void(int recursiveLevel)> ProjectManager::sigProjectAboutToBeLoaded()
{
    return impl->sigProjectAboutToBeLoaded;
}


SignalProxy<void(int recursiveLevel)> ProjectManager::sigProjectLoaded()
{
    return impl->sigProjectLoaded;
}


void ProjectManager::clearProject()
{
    impl->clearProject();
}


void ProjectManager::Impl::clearProject()
{
    auto rootItem = RootItem::instance();
    rootItem->clearChildren();
    rootItem->setConsistentWithProjectArchive(true);
    currentProjectName.clear();
    currentProjectFile.clear();
    mainWindow->setProjectTitle("");
}


template <class TObject>
bool ProjectManager::Impl::restoreObjectStates
(Archive* projectArchive, Archive* states, const vector<TObject*>& objects, const char* nameSuffix, MessageOut* mout)
{
    bool restored = false;
    for(size_t i=0; i < objects.size(); ++i){
        TObject* object = objects[i];
        const string& name = object->name();
        Archive* state = states->findSubArchive(name);
        if(state->isValid()){
            state->inheritSharedInfoFrom(*projectArchive);
            try {
                if(object->restoreState(*state)){
                    restored = true;
                }
            } catch(const ValueNode::Exception& ex){
                mout->putWarningln(
                    formatR(_("The state of the \"{0}\" {1} was not completely restored.\n"),
                            name, nameSuffix, ex.message()));
            }
        }
    }
    return restored;
}


ItemList<> ProjectManager::loadProject(const std::string& filename, Item* parentItem)
{
    return impl->loadProject(filename, parentItem, false, false, (parentItem == nullptr));
}


void ProjectManager::loadBuiltinProject(const std::string& resourceFile, Item* parentItem)
{
    impl->loadProject(resourceFile, parentItem, true, true, false);
}


ItemList<> ProjectManager::Impl::loadProject
(const std::string& filename, Item* parentItem,
 bool isInvokingApplication, bool isBuiltinProject, bool doClearExistingProject)
{
    auto mout = MessageOut::master();
    
    ItemList<> topLevelItems;
    
    if(doClearExistingProject){
        clearProject();
        mout->flush();
        sigProjectCleared();
    }

    bool isBackup = false;
    bool isTopProject = (projectBeingLoadedCounter == 0);
    if(isTopProject && parentItem){
        projectBeingLoadedCounter = 1;
    }
    
    sigProjectAboutToBeLoaded(projectBeingLoadedCounter);
    
    ++projectBeingLoadedCounter;

    bool loaded = false;
    YAMLReader reader;
    reader.setMappingClass<Archive>();

    try {
        if(!isBuiltinProject){
            mout->notify(formatR(_("Loading project file \"{}\" ..."), filename));
            if(!isInvokingApplication){
                mout->flush();
            }
        }

        int numArchivedItems = 0;
        int numRestoredItems = 0;

        bool parsed = false;
        if(!isBuiltinProject){
            parsed = reader.load(filename);
            if(!parsed){
                mout->putErrorln(reader.errorMessage());
            }
        } else {
            QResource resource(filename.c_str());
            if(!resource.isValid()){
                mout->putErrorln(formatR(_("Resource \"{0}\" is not found."), filename));
            } else {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
                auto data = resource.uncompressedData();
#else
                auto data = qUncompress(QByteArray((const char*)resource.data(), (int)resource.size()));
#endif
                parsed = reader.parse(data.constData(), data.size());
                if(!parsed){
                    mout->putErrorln(reader.errorMessage());
                }
            }
        }
        
        if(parsed && reader.numDocuments() == 0){
            mout->putWarningln(_("The project file is empty."));

        } else if(parsed){

            bool isSubProject = (parentItem != nullptr);
            if(!isSubProject){
                parentItem = RootItem::instance();
            }

            Archive* archive = static_cast<Archive*>(reader.document()->toMapping());
            archive->initSharedInfo(filename, isSubProject, mout, false);

            string backupSourceFile;
            isBackup = archive->read("backup_source", backupSourceFile);
            if(isBackup){
                if(backupSourceFile.empty()){
                    mout->putln(_("This is a backup of an unsaved project."));
                } else {
                    mout->putln(formatR(_("This is a backup of \"{0}\"."), backupSourceFile));
                }
            }

            std::set<string> optionalPlugins;
            Listing& optionalPluginsNode = *archive->findListing("optionalPlugins");
            if(optionalPluginsNode.isValid()){
                for(int i=0; i < optionalPluginsNode.size(); ++i){
                    optionalPlugins.insert(optionalPluginsNode[i].toString());
                }
            }

            ViewManager::ViewStateInfo viewStateInfo;
            if(ViewManager::restoreViews(archive, "views", viewStateInfo, optionalPlugins)){
                loaded = true;
            }

            MainWindow* mainWindow = MainWindow::instance();
            if(isInvokingApplication){
                if(isBuiltinProject || ::isLayoutInclusionMode){
                    mainWindow->setInitialLayout(archive);
                }
                if(!isBuiltinProject && !AppUtil::isNoWindowMode()){
                    mainWindow->show();
                }
            } else {
                if(isBuiltinProject || ::isLayoutInclusionMode){
                    mainWindow->restoreLayout(archive);
                }
            }

            for(auto& kv1 : archivers){
                const string& moduleName = kv1.first;
                const ArchiverMap& archiveMap = kv1.second;
                Archive* moduleArchive = archive->findSubArchive(moduleName);
                if(moduleArchive->isValid()){
                    for(auto& kv2 : archiveMap){
                        const ArchiverInfo& info = kv2.second;
                        if(info.restoreFunction){
                            const string& objectName = kv2.first;
                            Archive* objArchive;
                            if(objectName.empty()){
                                objArchive = moduleArchive;
                            } else {
                                objArchive = moduleArchive->findSubArchive(objectName);
                            }
                            if(objArchive->isValid()){
                                objArchive->inheritSharedInfoFrom(*archive);
                                info.restoreFunction(*objArchive);
                                loaded = true;
                            }
                        }
                    }
                }
            }

            if(ViewManager::restoreViewStates(viewStateInfo)){
                loaded = true;
            }

            Archive* barStates = archive->findSubArchive("toolbars");
            if(barStates->isValid()){
                if(restoreObjectStates(archive, barStates, mainWindow->toolBars(), "bar", mout)){
                    loaded = true;
                }
            }

            if(isInvokingApplication && !isBuiltinProject){
                mainWindow->waitForWindowSystemToActivate();
            }

            itemTreeArchiver.setMessageOut(mout);
            itemTreeArchiver.reset();
            Archive* items = archive->findSubArchive("items");
            if(items->isValid()){
                items->inheritSharedInfoFrom(*archive);

                topLevelItems = itemTreeArchiver.restore(items, parentItem, optionalPlugins);
                
                numArchivedItems = itemTreeArchiver.numArchivedItems();
                numRestoredItems = itemTreeArchiver.numRestoredItems();
                if(!isBuiltinProject){
                    mout->putln(formatR(_("{0} / {1} item(s) have been loaded."), numRestoredItems, numArchivedItems));
                }

                if(numRestoredItems < numArchivedItems){
                    int numUnloaded = numArchivedItems - numRestoredItems;
                    if(!isBuiltinProject){
                        mout->putWarningln(
                            formatR(_("{0} item(s) were not loaded."), numUnloaded));
                    } else {
                        mout->putWarningln(
                            formatR(_("{0} item(s) were not loaded in the builtin project \"{1}\"."),
                                    numUnloaded, filename));
                    }
                }
                
                if(numRestoredItems > 0){
                    loaded = true;
                }
            }

            if(loaded){
                if(!isSubProject){
                    if(archive->get("isNewProjectTemplate", false)){
                        clearCurrentProjectFile();
                    } else {
                        if(!isBackup){
                            setCurrentProjectFile(filename);
                        } else {
                            setCurrentProjectFile(backupSourceFile);
                        }
                    }
                }

                mout->flush();

                archive->callPostProcesses();

                if(!isBuiltinProject){
                    if(numRestoredItems == numArchivedItems){
                        mout->notify(formatR(_("Project \"{}\" has been completely loaded."), filename));
                    } else {
                        mout->notify(formatR(_("Project \"{}\" has been partially loaded."), filename));
                    }
                }
                if(!isSubProject && isBackup){
                    if(backupSourceFile.empty()){
                        mout->putln(_("This project backup has been loaded as an unsaved project."));
                    } else {
                        mout->putln(
                            formatR(_("This project backup has been loaded as project \"{0}\"."), backupSourceFile));
                    }
                }
            }
        }
    } catch (const ValueNode::Exception& ex){
        mout->putErrorln(ex.message());
    }

    if(!loaded){
        mout->notifyError(
            formatR(_("Loading project \"{}\" failed. Any valid objects were not loaded."), filename));
        clearCurrentProjectFile();
    }

    --projectBeingLoadedCounter;

    if(!self->isLoadingProject()){
        Archive::callFinalProcesses();
        auto vp = FilePathVariableProcessor::currentInstance();
        vp->clearBaseDirectory();
        vp->clearProjectDirectory();

        if(!isBackup){
            for(auto& item : topLevelItems){
                setItemsConsistentWithProjectArchive(item);
            }
        }
    }

    sigProjectLoaded(projectBeingLoadedCounter);

    if(isTopProject){
        projectBeingLoadedCounter = 0;
    }

    return topLevelItems;
}


void ProjectManager::Impl::setItemsConsistentWithProjectArchive(Item* item)
{
    // The following check is necessary in the backup mode.
    // In this mode, each item that is saved to a file is loaded from a backup file,
    // but the file to save is replaced with the original file. In this case, the file consistency
    // flag must be set to false when the backup file is not equivalent to the original file.
    // This check maintains the flags for those items.
    bool doUpdate = item->isConsistentWithFile();

    if(doUpdate){ 
        item->setConsistentWithProjectArchive(true);
    }
    
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        setItemsConsistentWithProjectArchive(child);
    }
}


void ProjectManager::restoreLayout(Mapping* layout)
{
    ArchivePtr archive = new Archive;
    archive->initSharedInfo("dummy", false, MessageOut::master(), false);
    archive->insert(layout, false);

    ViewManager::ViewStateInfo viewStateInfo;
    if(ViewManager::restoreViews(archive, "views", viewStateInfo, false)){
        mainWindow->restoreLayout(archive);
        if(ViewManager::restoreViewStates(viewStateInfo)){
            Archive* barStates = archive->findSubArchive("toolbars");
            if(barStates->isValid()){
                impl->restoreObjectStates(archive, barStates, mainWindow->toolBars(), "bar", MessageOut::master());
            }
        }
    }
}



template<class TObject> bool ProjectManager::Impl::storeObjects
(Archive& parentArchive, const char* key, vector<TObject*> objects)
{
    bool result = true;
    
    if(!objects.empty()){
        MappingPtr archives = new Mapping;
        archives->setKeyQuoteStyle(DOUBLE_QUOTED);
        for(size_t i=0; i < objects.size(); ++i){
            TObject* object = objects[i];
            const string& name = object->name();
            if(!name.empty()){
                ArchivePtr archive = new Archive;
                archive->inheritSharedInfoFrom(parentArchive);
                if(object->storeState(*archive) && !archive->empty()){
                    archives->insert(name, archive);
                }
            }
        }
        if(!archives->empty()){
            parentArchive.insert(key, archives);
            result = true;
        }
    }

    return result;
}


bool ProjectManager::saveProject(const string& filename, Item* item)
{
    return impl->saveProject(filename, item, false, false);
}


bool ProjectManager::saveProjectAsBackup(const string& filename, Item* item)
{
    return impl->saveProject(filename, item, false, true);
}


bool ProjectManager::Impl::saveProject
(const string& filename, Item* item, bool doSaveTemporaryItems, bool isBackupMode)
{
    MessageOutPtr mout;
    ofstream ofs;
    
    if(!isBackupMode){
        mout = MessageOut::master();
    } else {
        ofs.open(fromUTF8(filename + ".log"));
        mout = new MessageOut(ofs);
    }
            
    YAMLWriter writer(filename);
    if(!writer.isFileOpen()){
        mout->putErrorln(formatR(_("Can't open file \"{}\" for writing."), filename));
        return false;
    }

    bool isSubProject = false;
    auto rootItem = RootItem::instance();
    if(item){
        if(item != rootItem){
            isSubProject = true;
        }
    } else {
        item = rootItem;
    }
    
    if(!isSubProject){
        mout->put("\n");
        mout->notify(formatR(_("Saving the project as \"{}\" ..."), filename));
    }
    
    bool saved = false;
    
    itemTreeArchiver.setMessageOut(mout);
    itemTreeArchiver.reset();
    itemTreeArchiver.setTemporaryItemSaveEnabled(doSaveTemporaryItems);

    ArchivePtr archive = new Archive;
    archive->initSharedInfo(filename, isSubProject, mout, isBackupMode);

    if(isBackupMode){
        archive->write("backup_source", currentProjectFile, DOUBLE_QUOTED);
    }

    ArchivePtr itemArchive = itemTreeArchiver.store(archive, item);

    if(itemArchive){
        archive->insert("items", itemArchive);
        saved = true;
    }

    if(ViewManager::storeViewStates(archive, "views", false)){
        saved = true;
    }
    if(storeObjects(*archive, "toolbars", mainWindow->toolBars())){
        saved = true;
    }

    for(auto& kv1 : archivers){
        ArchivePtr moduleArchive = new Archive;
        moduleArchive->setKeyQuoteStyle(DOUBLE_QUOTED);
        ArchiverMap& archiveMap = kv1.second;
        for(auto& kv2 : archiveMap){
            ArchiverInfo& info = kv2.second;
            if(info.storeFunction){
                const string& objectName = kv2.first;
                ArchivePtr objArchive;
                if(objectName.empty()){
                    objArchive = moduleArchive;
                } else {
                    objArchive = new Archive;
                }
                objArchive->inheritSharedInfoFrom(*archive);
                if(info.storeFunction(*objArchive)){
                    if(!objectName.empty()){
                        moduleArchive->insert(objectName, objArchive);
                    }
                }
            }
        }
        if(!moduleArchive->empty()){
            const string& moduleName = kv1.first;
            archive->insert(moduleName, moduleArchive);
            saved = true;
        }
    }

    if(::isLayoutInclusionMode && !isSubProject){
        mainWindow->storeLayout(archive);
        saved = true;
    }

    if(saved){
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive);
        if(!isSubProject){
            mout->notify(_("Saving the project file has been finished."));
        } else {
            mout->notify(_("Saving the sub-project file has been finished."));
        }
        if(!isBackupMode && !isSubProject){
            setCurrentProjectFile(filename);
        }
    } else {
        if(!isSubProject){
            mout->notifyError(_("Saving the project file failed."));
        } else {
            mout->notifyError(_("Saving the sub-project file failed."));
        }            
        clearCurrentProjectFile();
    }

    return saved;
}


ref_ptr<Mapping> ProjectManager::storeCurrentLayout()
{
    ArchivePtr archive = new Archive;
    archive->initSharedInfo("dummy", false, MessageOut::master(), false);
    ViewManager::storeViewStates(archive, "views", true);
    mainWindow->storeLayout(archive);
    return archive;
}


bool ProjectManager::overwriteCurrentProject()
{
    bool saved;
    if(impl->currentProjectFile.empty()){
        saved = showDialogToSaveProject();
    } else {
        saved = impl->saveProject(impl->currentProjectFile, nullptr, false, false);
    }
    return saved;
}

    
void ProjectManager::Impl::onProjectOptionsParsed()
{
    for(auto& file : projectFilesToLoad){
        loadProject(file, nullptr, true, false, false);
    }
}


void ProjectManager::Impl::onInputFileOptionsParsed(std::vector<std::string>& inputFiles)
{
    auto it = inputFiles.begin();
    while(it != inputFiles.end()){
        if(filesystem::path(fromUTF8(*it)).extension().string() == ".cnoid"){
            loadProject(*it, nullptr, true, false, false);
            it = inputFiles.erase(it);
        } else {
            ++it;
        }
    }
}


bool ProjectManager::showDialogToLoadProject()
{
    if(!impl->confirmToCloseProject(true)){
        return false;
    }
    
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Open a project"));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.updatePresetDirectories(true);

    bool loaded = false;
    if(dialog.exec()){
        string filename = dialog.selectedFiles().front().toStdString();
        if(!impl->loadProject(filename, nullptr, false, false, true).empty()){
            loaded = true;
        }
    }

    return loaded;
}


bool ProjectManager::showDialogToSaveProject()
{
    auto& dialog = impl->saveDialog;
    
    if(!dialog){
        dialog = new SaveDialog(impl);
    }

    dialog->updatePresetDirectories(true);

    if(!dialog->selectFilePath(impl->currentProjectFile)){
        dialog->selectFile(impl->currentProjectName);
    }

    bool saved = false;
    if(dialog->exec() == QDialog::Accepted){
        saved = impl->saveProject(
            dialog->getSaveFilename(), nullptr, dialog->temporaryItemSaveCheck.isChecked(), false);
    }

    return saved;
}


bool ProjectManager::Impl::onSaveDialogAboutToFinish(int result)
{
    bool finished = true;
    if(result == QFileDialog::Accepted){
        auto filename = saveDialog->getSaveFilename();
        filesystem::path path(fromUTF8(filename));
        if(filesystem::exists(path)){
            saveDialog->fileDialog()->show();
            QString file(toUTF8(path.filename().string()).c_str());
            QString message(QString(_("%1 already exists. Do you want to replace it? ")).arg(file));
            auto button = QMessageBox::warning(
                saveDialog, saveDialog->windowTitle(), message, QMessageBox::Ok | QMessageBox::Cancel);
            if(button == QMessageBox::Cancel){
                finished = false;
            }
        }
    }
    return finished;
}


bool ProjectManager::tryToCloseProject()
{
    return impl->confirmToCloseProject(false);
}


bool ProjectManager::Impl::confirmToCloseProject(bool isAboutToLoadNewProject)
{
    auto rootItem = RootItem::instance();
    
    if(!checkValidItemExistence(rootItem)){
        // The current project is empty
        return true;
    }
    if(checkIfItemsConsistentWithProjectArchive(rootItem)){
        return true;
    }

    auto mw = MainWindow::instance();

    QString title = _("Warning");
    QString message;
    QMessageBox::StandardButton clicked;
    if(currentProjectFile.empty()){
        if(isAboutToLoadNewProject){
            message = _("The current project has not been saved yet. "
                        "Do you want to save it as a project file before loading a new project?");
        } else {
            message = _("The current project has not been saved yet. "
                        "Do you want to save it as a project file before closing the project?");
        }
        clicked = QMessageBox::warning(
            mw, title, message, QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Ignore);
    } else {
        if(isAboutToLoadNewProject){
            message = _("Project \"%1\" has been updated. "
                        "Do you want to save it before loading a new project?");
        } else {
            message = _("Project \"%1\" has been updated. "
                        "Do you want to save it before closing the project?");
        }
        clicked = QMessageBox::warning(
            mw, title, message.arg(currentProjectName.c_str()),
            QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Ignore);
    }
    bool accepted = false;
    if(clicked == QMessageBox::Ignore){
        accepted = true;
    } else if(clicked == QMessageBox::Save){
        accepted = self->overwriteCurrentProject();
    }

    return accepted;
}


bool ProjectManager::checkValidItemExistence()
{
    return Impl::checkValidItemExistence(RootItem::instance());
}


bool ProjectManager::Impl::checkValidItemExistence(Item* item)
{
    if(!item->hasAttribute(Item::Builtin) && !item->isTemporary()){
        return true;
    }
    for(auto child = item->childItem(); child; child = child->nextItem()){
        if(checkValidItemExistence(child)){
            return true;
        }
    }
    return false;
}


bool ProjectManager::checkIfItemsConsistentWithProjectArchive(Item* topItem)
{
    return Impl::checkIfItemsConsistentWithProjectArchive(topItem ? topItem : RootItem::instance(), true);
}


bool ProjectManager::Impl::checkIfItemsConsistentWithProjectArchive(Item* item, bool isTopItem)
{
    if(!isTopItem){
        if(!item->isConsistentWithProjectArchive()){
            return false;
        }
        if(auto subProjectItem = dynamic_cast<SubProjectItem*>(item)){
            if(subProjectItem->saveMode() == SubProjectItem::MANUAL_SAVE){
                return true; // Do not check sub-project items if the sub-project is the manusal save mode.
            }
        }
    }
    for(auto child = item->childItem(); child; child = child->nextItem()){
        if(!checkIfItemsConsistentWithProjectArchive(child, false)){
            return false;
        }
    }
    return true;
}


void ProjectManager::setArchiver(
    const std::string& moduleName,
    const std::string& name,
    std::function<bool(Archive&)> storeFunction,
    std::function<void(const Archive&)> restoreFunction)
{
    Impl::ArchiverInfo& info = impl->archivers[moduleName][name];
    info.storeFunction = storeFunction;
    info.restoreFunction = restoreFunction;
}


void ProjectManager::resetArchivers(const std::string& moduleName)
{
    impl->archivers.erase(moduleName);
}


SaveDialog::SaveDialog(ProjectManager::Impl* manager)
{
    setWindowTitle(_("Save a project"));
    setFileMode(QFileDialog::AnyFile);
    setAcceptMode(QFileDialog::AcceptSave);
    setViewMode(QFileDialog::List);
    setLabelText(QFileDialog::Accept, _("Save"));
    setLabelText(QFileDialog::Reject, _("Cancel"));
    setOption(QFileDialog::DontConfirmOverwrite);

    auto optionPanel = new QWidget;
    auto vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    temporaryItemSaveCheck.setText(_("Save temporary items"));
    temporaryItemSaveCheck.setVisible(isTemporaryItemSaveCheckAvailable);
    vbox->addWidget(&temporaryItemSaveCheck);
    optionPanel->setLayout(vbox);
    insertOptionPanel(optionPanel);
    
    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    setNameFilters(filters);

    sigAboutToFinish().connect(
        [this, manager](int result){
            return manager->onSaveDialogAboutToFinish(result);
        });
}


std::string SaveDialog::getSaveFilename()
{
    std::string filename;
    auto filenames = selectedFiles();
    if(!filenames.isEmpty()){
        filename = filenames.front().toStdString();
        filesystem::path path(fromUTF8(filename));
        string ext = path.extension().string();
        if(ext != ".cnoid"){
            filename += ".cnoid";
        }
    }
    return filename;
}
