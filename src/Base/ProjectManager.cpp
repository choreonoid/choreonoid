/**
   @author Shin'ichiro Nakaoka
*/

#include "ProjectManager.h"
#include "RootItem.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "MessageView.h"
#include "ToolBar.h"
#include "Archive.h"
#include "ItemTreeArchiver.h"
#include "ExtensionManager.h"
#include "OptionManager.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "AppUtil.h"
#include "LazyCaller.h"
#include "FileDialog.h"
#include <cnoid/MainWindow>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/ExecutablePath>
#include <cnoid/Sleep>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QEvent>
#include <QResource>
#include <QMessageBox>
#include <string>
#include <vector>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

ProjectManager* instance_ = nullptr;
bool defaultOptionToSotreLayoutInProjectFile = true;
int projectBeingLoadedCounter = 0;
Action* perspectiveCheck = nullptr;
MainWindow* mainWindow = nullptr;
MessageView* mv = nullptr;

Signal<void(int recursiveLevel)> sigProjectAboutToBeLoaded;
Signal<void(int recursiveLevel)> sigProjectLoaded;

#ifdef Q_OS_UNIX
class WindowActivationChecker : public QObject
{
public:
    bool isWindowActivated;

    WindowActivationChecker() : isWindowActivated(false) { }

    bool eventFilter(QObject* obj, QEvent* event) override {
        if(event->type() == QEvent::ActivationChange){
            isWindowActivated = true;
        }
        return false;
    }
};
#endif

}

namespace cnoid {

class ProjectManager::Impl
{
public:
    Impl(ProjectManager* self);
    Impl(ProjectManager* self, ExtensionManager* ext);

    void setCurrentProjectFile(const string& filename);
    void clearCurrentProjectFile();
    void clearProject();
        
    template <class TObject>
    bool restoreObjectStates(
        Archive* projectArchive, Archive* states, const vector<TObject*>& objects, const char* nameSuffix);

    void loadProject(
        const std::string& filename, Item* parentItem,
        bool isInvokingApplication, bool isBuiltinProject, bool doClearExistingProject);

    template<class TObject>
    bool storeObjects(Archive& parentArchive, const char* key, vector<TObject*> objects);
        
    void saveProject(const string& filename, Item* item = nullptr);
    void overwriteCurrentProject();
        
    void onProjectOptionsParsed(boost::program_options::variables_map& v);
    void onInputFileOptionsParsed(std::vector<std::string>& inputFiles);
    void openDialogToLoadProject();
    void openDialogToSaveProject();
    std::string getSaveFilename(FileDialog& dialog);
    bool onSaveDialogAboutToFinished(FileDialog& dialog, int result);

    void onPerspectiveCheckToggled(bool on);
        
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

#ifdef Q_OS_UNIX
    WindowActivationChecker mainWindowActivationChecker;
#endif

    MappingPtr config;
    MappingPtr managerConfig;
};

}


void ProjectManager::setDefaultOptionToStoreLayoutInProjectFile(bool on)
{
    defaultOptionToSotreLayoutInProjectFile = on;
}


ProjectManager* ProjectManager::instance()
{
    return instance_;
}


void ProjectManager::initializeClass(ExtensionManager* ext)
{
    if(!instance_){
        instance_ = ext->manage(new ProjectManager(ext));
    }
}


ProjectManager::ProjectManager()
{
    impl = new Impl(this);
}


ProjectManager::Impl::Impl(ProjectManager* self)
    : self(self)
{
    
}


ProjectManager::ProjectManager(ExtensionManager* ext)
{
    impl = new Impl(this, ext);
}


// The constructor for the main instance
ProjectManager::Impl::Impl(ProjectManager* self, ExtensionManager* ext)
    : Impl(self)
{
    config = AppConfig::archive();
    managerConfig = config->openMapping("ProjectManager");
    
    MenuManager& mm = ext->menuManager();

    mm.setPath("/File");

    mm.addItem(_("Open Project"))
        ->sigTriggered().connect([&](){ openDialogToLoadProject(); });
    mm.addItem(_("Save Project"))
        ->sigTriggered().connect([&](){ overwriteCurrentProject(); });
    mm.addItem(_("Save Project As"))
        ->sigTriggered().connect([&](){ openDialogToSaveProject(); });

    mm.setPath(N_("Project File Options"));

    perspectiveCheck = mm.addCheckItem(_("Perspective"));
    bool isPerspectiveChecked = managerConfig->get("store_perspective", defaultOptionToSotreLayoutInProjectFile);
    perspectiveCheck->setChecked(isPerspectiveChecked);
    perspectiveCheck->sigToggled().connect([&](bool on){ onPerspectiveCheckToggled(on); });

    mm.setPath("/File");
    mm.addSeparator();

    OptionManager& om = ext->optionManager();
    om.addOption("project", boost::program_options::value<vector<string>>(), "load a project file");
    om.sigInputFileOptionsParsed().connect(
        [&](std::vector<std::string>& inputFiles){ onInputFileOptionsParsed(inputFiles); });
    om.sigOptionsParsed().connect(
        [&](boost::program_options::variables_map& v){ onProjectOptionsParsed(v); });

    mainWindow = MainWindow::instance();
    mv = MessageView::instance();
}


ProjectManager::~ProjectManager()
{
    delete impl;
}


SignalProxy<void(int recursiveLevel)> ProjectManager::sigProjectAboutToBeLoaded()
{
    return ::sigProjectAboutToBeLoaded;
}


SignalProxy<void(int recursiveLevel)> ProjectManager::sigProjectLoaded()
{
    return ::sigProjectLoaded;
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
    filesystem::path path(fromUTF8(filename));
    auto name = toUTF8(path.stem().string());
    currentProjectName = name;
    mainWindow->setProjectTitle(name);

    // filesystem::canonical can only be used with C++17
    path = filesystem::lexically_normal(filesystem::absolute(path));

    currentProjectFile = toUTF8(path.string());
    currentProjectDirectory = toUTF8(path.parent_path().string());
}


void ProjectManager::Impl::clearCurrentProjectFile()
{
    currentProjectFile.clear();
}


std::string ProjectManager::currentProjectFile() const
{
    return impl->currentProjectFile;
}


std::string ProjectManager::currentProjectDirectory() const
{
    return impl->currentProjectDirectory;
}


void ProjectManager::clearProject()
{
    impl->clearProject();
}


void ProjectManager::Impl::clearProject()
{
    RootItem::instance()->clearChildren();
    currentProjectName.clear();
    currentProjectFile.clear();
    mainWindow->setProjectTitle("");
}


template <class TObject>
bool ProjectManager::Impl::restoreObjectStates
(Archive* projectArchive, Archive* states, const vector<TObject*>& objects, const char* nameSuffix)
{
    bool restored = false;
    for(size_t i=0; i < objects.size(); ++i){
        TObject* object = objects[i];
        const string name = object->objectName().toStdString();
        Archive* state = states->findSubArchive(name);
        if(state->isValid()){
            state->inheritSharedInfoFrom(*projectArchive);
            try {
                if(object->restoreState(*state)){
                    restored = true;
                }
            } catch(const ValueNode::Exception& ex){
                mv->putln(
                    format(_("The state of the \"{0}\" {1} was not completely restored.\n{2}"),
                    name, nameSuffix, ex.message()),
                    MessageView::Warning);
            }
        }
    }
    return restored;
}


void ProjectManager::loadProject(const std::string& filename, Item* parentItem)
{
    impl->loadProject(filename, parentItem, false, false, (parentItem == nullptr));
}


void ProjectManager::loadBuiltinProject(const std::string& resourceFile, Item* parentItem)
{
    impl->loadProject(resourceFile, parentItem, true, true, false);
}


void ProjectManager::Impl::loadProject
(const std::string& filename, Item* parentItem,
 bool isInvokingApplication, bool isBuiltinProject, bool doClearExistingProject)
{
    ::sigProjectAboutToBeLoaded(projectBeingLoadedCounter);
    
    ++projectBeingLoadedCounter;

    if(doClearExistingProject){
        clearProject();
        mv->flush();
    }
    
    bool loaded = false;
    YAMLReader reader;
    reader.setMappingClass<Archive>();

    try {
        if(!isBuiltinProject){
            mv->notify(format(_("Loading project file \"{}\" ..."), filename));
            if(!isInvokingApplication){
                mv->flush();
            }
        }

        int numArchivedItems = 0;
        int numRestoredItems = 0;

        bool parsed = false;
        if(!isBuiltinProject){
            parsed = reader.load(filename);
            if(!parsed){
                mv->putln(reader.errorMessage(), MessageView::Error);
            }
        } else {
            QResource resource(filename.c_str());
            if(resource.isValid()){
                auto data = qUncompress(QByteArray((const char*)resource.data(), (int)resource.size()));
                parsed = reader.parse(data.constData(), data.size());
                if(!parsed){
                    mv->putln(reader.errorMessage(), MessageView::Error);
                }
            } else {
                mv->putln(format(_("Resource \"{0}\" is not found."), filename), MessageView::Error);
            }
        }
        
        if(parsed && reader.numDocuments() == 0){
            mv->putln(_("The project file is empty."), MessageView::Warning);

        } else if(parsed){

            bool isSubProject = (parentItem != nullptr);
            if(!isSubProject){
                parentItem = RootItem::instance();
            }

            Archive* archive = static_cast<Archive*>(reader.document()->toMapping());
            archive->initSharedInfo(filename, isSubProject);

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
                if(isBuiltinProject || perspectiveCheck->isChecked()){
                    mainWindow->setInitialLayout(archive);
                }
                if(!isBuiltinProject){
#ifdef Q_OS_UNIX
                    mainWindow->installEventFilter(&mainWindowActivationChecker);
#endif
                    mainWindow->show();
                }
            } else {
                if(isBuiltinProject || perspectiveCheck->isChecked()){
                    mainWindow->restoreLayout(archive);
                }
            }

            ArchiverMapMap::iterator p;
            for(p = archivers.begin(); p != archivers.end(); ++p){
                const string& moduleName = p->first;
                Archive* moduleArchive = archive->findSubArchive(moduleName);
                if(moduleArchive->isValid()){
                    ArchiverMap::iterator q;
                    for(q = p->second.begin(); q != p->second.end(); ++q){
                        const string& objectName = q->first;
                        Archive* objArchive;
                        if(objectName.empty()){
                            objArchive = moduleArchive;
                        } else {
                            objArchive = moduleArchive->findSubArchive(objectName);
                        }
                        if(objArchive->isValid()){
                            ArchiverInfo& info = q->second;
                            objArchive->inheritSharedInfoFrom(*archive);
                            info.restoreFunction(*objArchive);
                            loaded = true;
                        }
                    }
                }
            }

            if(ViewManager::restoreViewStates(viewStateInfo)){
                loaded = true;
            } else {
                // load the old format (version 1.4 or earlier)
                Archive* viewStates = archive->findSubArchive("views");
                if(viewStates->isValid()){
                    if(restoreObjectStates(archive, viewStates, ViewManager::allViews(), "view")){
                        loaded = true;
                    }
                }
            }

            Archive* barStates = archive->findSubArchive("toolbars");
            if(barStates->isValid()){
                if(restoreObjectStates(archive, barStates, mainWindow->toolBars(), "bar")){
                    loaded = true;
                }
            }

#ifdef Q_OS_UNIX
            if(isInvokingApplication && !isBuiltinProject){
                /**
                   There is a delay between executing the show function of a window and the window
                   is actually displayed. If the event loop is blocked by an item that takes a long
                   time to load before the window is displayed, the window will remain hidden for
                   a while. This behavior gives a user the bad impression that the application is
                   slow to start. To avoid this problem, the window should be displayed before any
                   items are loaded. This can be achieved by decreasing the time difference from
                   the window display delay by the following loop to check if the window is actually shown.
                */
                int timeoutCounter = 0;
                while(true){
                    updateGui();
                    if(mainWindowActivationChecker.isWindowActivated){
                        break;
                    }
                    msleep(1);
                    ++timeoutCounter;
                    if(timeoutCounter > 100){
                        break;
                    }
                }
                mainWindow->removeEventFilter(&mainWindowActivationChecker);
            }
#endif
            
            itemTreeArchiver.reset();
            Archive* items = archive->findSubArchive("items");
            if(items->isValid()){
                items->inheritSharedInfoFrom(*archive);

                itemTreeArchiver.restore(items, parentItem, optionalPlugins);
                
                numArchivedItems = itemTreeArchiver.numArchivedItems();
                numRestoredItems = itemTreeArchiver.numRestoredItems();
                if(!isBuiltinProject){
                    mv->putln(format(_("{0} / {1} item(s) have been loaded."), numRestoredItems, numArchivedItems));
                }

                if(numRestoredItems < numArchivedItems){
                    int numUnloaded = numArchivedItems - numRestoredItems;
                    if(!isBuiltinProject){
                        mv->putln(format(_("{0} item(s) were not loaded."), numUnloaded), MessageView::Warning);
                    } else {
                        mv->putln(format(_("{0} item(s) were not loaded in the builtin project \"{1}\"."),
                                         numUnloaded, filename), MessageView::Warning);
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
                        setCurrentProjectFile(filename);
                    }
                }

                mv->flush();

                archive->callPostProcesses();

                if(!isBuiltinProject){
                    if(numRestoredItems == numArchivedItems){
                        mv->notify(format(_("Project \"{}\" has been completely loaded."), filename));
                    } else {
                        mv->notify(format(_("Project \"{}\" has been partially loaded."), filename));
                    }
                }
            }
        }
    } catch (const ValueNode::Exception& ex){
        mv->put(ex.message());
    }

    if(!loaded){
        mv->notify(
            format(_("Loading project \"{}\" failed. Any valid objects were not loaded."), filename),
            MessageView::Error);
        clearCurrentProjectFile();
    }

    --projectBeingLoadedCounter;

    if(self->isLoadingProject()){
        ::sigProjectLoaded(projectBeingLoadedCounter);
    } else {
        Archive::callFinalProcesses();
        ::sigProjectLoaded(projectBeingLoadedCounter);
        auto vp = FilePathVariableProcessor::systemInstance();
        vp->clearBaseDirectory();
        vp->clearProjectDirectory();
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
            string name = object->objectName().toStdString();
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


void ProjectManager::saveProject(const string& filename, Item* item)
{
    impl->saveProject(filename, item);
}


void ProjectManager::Impl::saveProject(const string& filename, Item* item)
{
    YAMLWriter writer(filename);
    if(!writer.isFileOpen()){
        mv->put(
            format(_("Can't open file \"{}\" for writing.\n"), filename),
            MessageView::Error);
        return;
    }

    bool isSubProject;
    if(item){
        isSubProject = true;
    } else {
        item = RootItem::instance();
        isSubProject = false;
    }
    
    mv->putln();
    if(isSubProject){
        mv->notify(format(_("Saving sub project {0} as \"{1}\" ..."), item->displayName(), filename));
    } else {
        mv->notify(format(_("Saving the project as \"{}\" ..."), filename));
    }
    mv->flush();
    
    itemTreeArchiver.reset();

    ArchivePtr archive = new Archive;
    archive->initSharedInfo(filename, isSubProject);

    ArchivePtr itemArchive = itemTreeArchiver.store(archive, item);

    if(itemArchive){
        archive->insert("items", itemArchive);
    }

    bool stored = ViewManager::storeViewStates(archive, "views");

    stored |= storeObjects(*archive, "toolbars", mainWindow->toolBars());

    ArchiverMapMap::iterator p;
    for(p = archivers.begin(); p != archivers.end(); ++p){
        ArchivePtr moduleArchive = new Archive;
        moduleArchive->setKeyQuoteStyle(DOUBLE_QUOTED);
        ArchiverMap::iterator q;
        for(q = p->second.begin(); q != p->second.end(); ++q){
            const string& objectName = q->first;
            ArchivePtr objArchive;
            if(objectName.empty()){
                objArchive = moduleArchive;
            } else {
                objArchive = new Archive;
            }
            objArchive->inheritSharedInfoFrom(*archive);
            ArchiverInfo& info = q->second;
            if(info.storeFunction(*objArchive)){
                if(!objectName.empty()){
                    moduleArchive->insert(objectName, objArchive);
                }
            }
        }
        if(!moduleArchive->empty()){
            const string& moduleName = p->first;
            archive->insert(moduleName, moduleArchive);
            stored = true;
        }
    }

    if(perspectiveCheck->isChecked() && !isSubProject){
        mainWindow->storeLayout(archive);
        stored = true;
    }

    if(stored){
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive);
        mv->notify(_("Saving the project file has been finished."));
        if(!isSubProject){
            setCurrentProjectFile(filename);
        }
    } else {
        mv->notify(_("Saving the project file failed."), MessageView::Error);
        clearCurrentProjectFile();
    }
}


void ProjectManager::overwriteCurrentProject()
{
    impl->overwriteCurrentProject();
}


void ProjectManager::Impl::overwriteCurrentProject()
{
    if(currentProjectFile.empty()){
        openDialogToSaveProject();
    } else {
        saveProject(currentProjectFile);
    } 
}

    
void ProjectManager::Impl::onProjectOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("project")){
        vector<string> projectFileNames = v["project"].as<vector<string>>();
        for(size_t i=0; i < projectFileNames.size(); ++i){
            loadProject(projectFileNames[i], nullptr, true, false, false);
        }
    }
}


void ProjectManager::Impl::onInputFileOptionsParsed(std::vector<std::string>& inputFiles)
{
    auto iter = inputFiles.begin();
    while(iter != inputFiles.end()){
        if(filesystem::path(*iter).extension().string() == ".cnoid"){
            loadProject(*iter, nullptr, true, false, false);
            iter = inputFiles.erase(iter);
        } else {
            ++iter;
        }
    }
}


void ProjectManager::Impl::openDialogToLoadProject()
{
    auto mw = MainWindow::instance();
    int numItems = RootItem::instance()->countDescendantItems();
    if(numItems > 0){
        QString title = _("Warning");
        QString message;
        QMessageBox::StandardButton clicked;
        if(currentProjectFile.empty()){
            if(numItems == 1){
                message = _("A project item exists. "
                            "Do you want to save it as a project file before loading a new project?");
            } else {
                message = _("Project items exist. "
                            "Do you want to save them as a project file before loading a new project?");
            }
            clicked = QMessageBox::warning(
                mw, title, message, QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Ignore);
        } else {
            message = _("Project \"%1\" exists. Do you want to save it before loading a new project?");
            clicked = QMessageBox::warning(
                mw, title, message.arg(currentProjectName.c_str()),
                QMessageBox::Save | QMessageBox::Cancel | QMessageBox::Ignore);
        }
        if(clicked == QMessageBox::Cancel){
            return;
        }
        if(clicked == QMessageBox::Save){
            overwriteCurrentProject();
        }
    }
    
    FileDialog dialog(mw);
    dialog.setWindowTitle(_("Open a Choreonoid project file"));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.updatePresetDirectories();
    
    if(dialog.exec()){
        string filename = dialog.selectedFiles().front().toStdString();
        loadProject(filename, nullptr, false, false, true);
    }
}


void ProjectManager::Impl::openDialogToSaveProject()
{
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Save a choreonoid project file"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.setOption(QFileDialog::DontConfirmOverwrite);
    
    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.updatePresetDirectories();

    if(!dialog.selectFilePath(currentProjectFile)){
        dialog.selectFile(currentProjectName);
    }

    dialog.sigAboutToFinished().connect(
        [&](int result){ return onSaveDialogAboutToFinished(dialog, result); });

    if(dialog.exec() == QDialog::Accepted){
        saveProject(getSaveFilename(dialog));
    }
    
}


std::string ProjectManager::Impl::getSaveFilename(FileDialog& dialog)
{
    std::string filename;
    auto filenames = dialog.selectedFiles();
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


bool ProjectManager::Impl::onSaveDialogAboutToFinished(FileDialog& dialog, int result)
{
    bool finished = true;
    if(result == QFileDialog::Accepted){
        auto filename = getSaveFilename(dialog);
        filesystem::path path(fromUTF8(filename));
        if(filesystem::exists(path)){
            dialog.fileDialog()->show();
            QString file(toUTF8(path.filename().string()).c_str());
            QString message(QString(_("%1 already exists. Do you want to replace it? ")).arg(file));
            auto button =
                QMessageBox::warning(&dialog, dialog.windowTitle(), message, QMessageBox::Ok | QMessageBox::Cancel);
            if(button == QMessageBox::Cancel){
                finished = false;
            }
        }
    }
    return finished;
}


void ProjectManager::Impl::onPerspectiveCheckToggled(bool on)
{
    managerConfig->write("store_perspective", perspectiveCheck->isChecked());
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
