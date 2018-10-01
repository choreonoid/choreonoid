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
#include "LazyCaller.h"
#include <cnoid/MainWindow>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <QFileDialog>
#include <QCoreApplication>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;

namespace {
ProjectManager* instance_ = 0;
}

namespace cnoid {

class ProjectManagerImpl
{
public:
    ProjectManagerImpl(ProjectManager* self, ExtensionManager* em);
    ~ProjectManagerImpl();
        
    template <class TObject>
    bool restoreObjectStates(
        Archive* projectArchive, Archive* states, const vector<TObject*>& objects, const char* nameSuffix);
        
    ItemList<> loadProject(const string& filename, Item* parentItem, bool isInvokingApplication);

    template<class TObject>
    bool storeObjects(Archive& parentArchive, const char* key, vector<TObject*> objects);
        
    void saveProject(const string& filename);
    void overwriteCurrentProject();
        
    void onProjectOptionsParsed(boost::program_options::variables_map& v);
    void onInputFileOptionsParsed(std::vector<std::string>& inputFiles);
    void openDialogToLoadProject();
    void openDialogToSaveProject();

    void onPerspectiveCheckToggled(bool on);
    void onHomeRelativeCheckToggled(bool on);
        
    void connectArchiver(
        const std::string& name,
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);

    ProjectManager* self;
    bool isLoadingProject;
    ItemTreeArchiver itemTreeArchiver;
    MainWindow* mainWindow;
    MessageView* mv;
    string currentProjectName;
    string lastAccessedProjectFile;
    Action* perspectiveCheck;
    Action* homeRelativeCheck;

    struct ArchiverInfo {
        std::function<bool(Archive&)> storeFunction;
        std::function<void(const Archive&)> restoreFunction;
    };
    typedef map<string, ArchiverInfo> ArchiverMap;
    typedef map<string, ArchiverMap> ArchiverMapMap;
    ArchiverMapMap archivers;
};

}


ProjectManager* ProjectManager::instance()
{
    return instance_;
}


void ProjectManager::initialize(ExtensionManager* em)
{
    if(!instance_){
        instance_ = em->manage(new ProjectManager(em));
    }
}


ProjectManager::ProjectManager(ExtensionManager* em)
{
    impl = new ProjectManagerImpl(this, em);
}


ProjectManagerImpl::ProjectManagerImpl(ProjectManager* self, ExtensionManager* em)
    : self(self)
{
    MappingPtr config = AppConfig::archive()->openMapping("ProjectManager");
    
    MenuManager& mm = em->menuManager();

    mm.setPath("/File");

    mm.addItem(_("Open Project"))
        ->sigTriggered().connect([&](){ openDialogToLoadProject(); });
    mm.addItem(_("Save Project"))
        ->sigTriggered().connect([&](){ overwriteCurrentProject(); });
    mm.addItem(_("Save Project As"))
        ->sigTriggered().connect([&](){ openDialogToSaveProject(); });

    mm.setPath(N_("Project File Options"));

    perspectiveCheck = mm.addCheckItem(_("Perspective"));
    perspectiveCheck->setChecked(config->get("storePerspective", true));
    perspectiveCheck->sigToggled().connect([&](bool on){ onPerspectiveCheckToggled(on); });

    homeRelativeCheck = mm.addCheckItem(_("Use HOME relative directories"));
    homeRelativeCheck->setChecked(config->get("useHomeRelative", false));
    homeRelativeCheck->sigToggled().connect([&](bool on){ onHomeRelativeCheckToggled(on); });

    mm.setPath("/File");
    mm.addSeparator();

    OptionManager& om = em->optionManager();
    om.addOption("project", boost::program_options::value<vector<string>>(), "load a project file");
    om.sigInputFileOptionsParsed().connect(
        [&](std::vector<std::string>& inputFiles){ onInputFileOptionsParsed(inputFiles); });
    om.sigOptionsParsed().connect(
        [&](boost::program_options::variables_map& v){ onProjectOptionsParsed(v); });

    isLoadingProject = false;
    mainWindow = MainWindow::instance();
    mv = MessageView::instance();
}


ProjectManager::~ProjectManager()
{
    delete impl;
    instance_ = 0;
}


ProjectManagerImpl::~ProjectManagerImpl()
{

}


template <class TObject>
bool ProjectManagerImpl::restoreObjectStates
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
                    format(_("The state of the \"%1%\" %2% was not completely restored.\n%3%"))
                    % name % nameSuffix % ex.message(),
                    MessageView::WARNING);
            }
        }
    }
    return restored;
}


bool ProjectManager::isLoadingProject() const
{
    return impl->isLoadingProject;
}
        

ItemList<> ProjectManager::loadProject(const std::string& filename, Item* parentItem)
{
    return impl->loadProject(filename, parentItem, false);
}


ItemList<> ProjectManagerImpl::loadProject(const std::string& filename, Item* parentItem, bool isInvokingApplication)
{
    ItemList<> loadedItems;
    
    isLoadingProject = true;
    
    bool loaded = false;
    YAMLReader reader;
    reader.setMappingClass<Archive>();

    try {
        mv->notify(format(_("Loading project file \"%1%\" ...")) % filename);
        if(!isInvokingApplication){
            mv->flush();
        }

        int numArchivedItems = 0;
        int numRestoredItems = 0;
        
        if(!reader.load(filename)){
            mv->put(reader.errorMessage() + "\n");

        } else if(reader.numDocuments() == 0){
            mv->putln(_("The project file is empty."), MessageView::WARNING);

        } else {
            Archive* archive = static_cast<Archive*>(reader.document()->toMapping());
            archive->initSharedInfo(filename);

            std::set<string> optionalPlugins;
            Listing& optionalPluginsNode = *archive->findListing("optionalPlugins");
            if(optionalPluginsNode.isValid()){
                for(int i=0; i < optionalPluginsNode.size(); ++i){
                    optionalPlugins.insert(optionalPluginsNode[i].toString());
                }
            }

            ViewManager::ViewStateInfo viewStateInfo;
            if(ViewManager::restoreViews(archive, "views", viewStateInfo)){
                loaded = true;
            }

            MainWindow* mainWindow = MainWindow::instance();
            if(isInvokingApplication){
                if(perspectiveCheck->isChecked()){
                    mainWindow->setInitialLayout(archive);
                }
                mainWindow->show();
                mv->flush();
                mainWindow->repaint();
            } else {
                if(perspectiveCheck->isChecked()){
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
                vector<ToolBar*> toolBars;
                mainWindow->getAllToolBars(toolBars);
                if(restoreObjectStates(archive, barStates, toolBars, "bar")){
                    loaded = true;
                }
            }

            itemTreeArchiver.reset();
            Archive* items = archive->findSubArchive("items");
            if(items->isValid()){
                items->inheritSharedInfoFrom(*archive);

                if(!parentItem){
                    parentItem = RootItem::instance();
                }
                loadedItems = itemTreeArchiver.restore(items, parentItem, optionalPlugins);
                
                numArchivedItems = itemTreeArchiver.numArchivedItems();
                numRestoredItems = itemTreeArchiver.numRestoredItems();
                mv->putln(format(_("%1% / %2% item(s) have been loaded.")) % numRestoredItems % numArchivedItems);

                if(numRestoredItems < numArchivedItems){
                    mv->putln(
                        format(_("%1% item(s) were not loaded.")) % (numArchivedItems - numRestoredItems),
                        MessageView::WARNING);
                }
                
                if(numRestoredItems > 0){
                    loaded = true;
                }
            }

            if(loaded){
                self->setCurrentProjectName(getBasename(filename));
                lastAccessedProjectFile = filename;

                mv->flush();
                
                archive->callPostProcesses();

                if(numRestoredItems == numArchivedItems){
                    mv->notify(format(_("Project \"%1%\" has been completely loaded.")) % filename);
                } else {
                    mv->notify(format(_("Project \"%1%\" has been partially loaded.")) % filename);
                }
            }
        }
    } catch (const ValueNode::Exception& ex){
        mv->put(ex.message());
    }

    isLoadingProject = false;
    
    if(!loaded){                
        mv->notify(
            format(_("Loading project \"%1%\" failed. Any valid objects were not loaded.")) % filename,
            MessageView::ERROR);
        lastAccessedProjectFile.clear();
    }

    return loadedItems;
}


void ProjectManager::setCurrentProjectName(const std::string& name)
{
    impl->currentProjectName = name;
    impl->mainWindow->setProjectTitle(name);
}
        

template<class TObject> bool ProjectManagerImpl::storeObjects
(Archive& parentArchive, const char* key, vector<TObject*> objects)
{
    bool result = true;
    
    if(!objects.empty()){
        MappingPtr archives = new Mapping();
        archives->setKeyQuoteStyle(DOUBLE_QUOTED);
        for(size_t i=0; i < objects.size(); ++i){
            TObject* object = objects[i];
            string name = object->objectName().toStdString();
            if(!name.empty()){
                ArchivePtr archive = new Archive();
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


void ProjectManager::saveProject(const string& filename)
{
    impl->saveProject(filename);
}


void ProjectManagerImpl::saveProject(const string& filename)
{
    YAMLWriter writer(filename);
    if(!writer.isOpen()){
        mv->put(
            format(_("Can't open file \"%1%\" for writing.\n")) % filename,
            MessageView::ERROR);
        return;
    }

    mv->putln();
    mv->notify(format(_("Saving a project to \"%1%\" ...")) % filename);
    mv->flush();
    
    itemTreeArchiver.reset();
    
    ArchivePtr archive = new Archive();
    archive->initSharedInfo(filename, homeRelativeCheck->isChecked());

    ArchivePtr itemArchive = itemTreeArchiver.store(archive, RootItem::mainInstance());

    if(itemArchive){
        archive->insert("items", itemArchive);
    }

    bool stored = ViewManager::storeViewStates(archive, "views");

    vector<ToolBar*> toolBars;
    mainWindow->getAllToolBars(toolBars);
    stored |= storeObjects(*archive, "toolbars", toolBars);

    ArchiverMapMap::iterator p;
    for(p = archivers.begin(); p != archivers.end(); ++p){
        ArchivePtr moduleArchive = new Archive();
        moduleArchive->setKeyQuoteStyle(DOUBLE_QUOTED);
        ArchiverMap::iterator q;
        for(q = p->second.begin(); q != p->second.end(); ++q){
            const string& objectName = q->first;
            ArchivePtr objArchive;
            if(objectName.empty()){
                objArchive = moduleArchive;
            } else {
                objArchive = new Archive();
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

    if(perspectiveCheck->isChecked()){
        mainWindow->storeLayout(archive);
        stored = true;
    }

    // Storing the file dialog directory from the project file.
    // This should be disabled.
    /*
      string currentFileDialogDirectory;
      if(AppConfig::archive()->read("currentFileDialogDirectory", currentFileDialogDirectory)){
      archive->writeRelocatablePath("currentFileDialogDirectory", currentFileDialogDirectory);
      }
    */

    if(stored){
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive);
        mv->notify(_("Saving a project file has been finished."));
        mainWindow->setProjectTitle(getBasename(filename));
        lastAccessedProjectFile = filename;
    } else {
        mv->notify(_("Saving a project file failed."), MessageView::ERROR);
        lastAccessedProjectFile.clear();
    }
}


void ProjectManager::overwriteCurrentProject()
{
    impl->overwriteCurrentProject();
}


void ProjectManagerImpl::overwriteCurrentProject()
{
    if(lastAccessedProjectFile.empty()){
        openDialogToSaveProject();
    } else {
        saveProject(lastAccessedProjectFile);
    } 
}

    
void ProjectManagerImpl::onProjectOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("project")){
        vector<string> projectFileNames = v["project"].as<vector<string>>();
        for(size_t i=0; i < projectFileNames.size(); ++i){
            loadProject(toActualPathName(projectFileNames[i]), nullptr, true);
        }
    }
}


void ProjectManagerImpl::onInputFileOptionsParsed(std::vector<std::string>& inputFiles)
{
    auto iter = inputFiles.begin();
    while(iter != inputFiles.end()){
        if(getExtension(*iter) == "cnoid"){
            loadProject(toActualPathName(*iter), nullptr, true);
            iter = inputFiles.erase(iter);
        } else {
            ++iter;
        }
    }
}


void ProjectManagerImpl::openDialogToLoadProject()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Open a Choreonoid project file"));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.setDirectory(AppConfig::archive()->get
                        ("currentFileDialogDirectory", shareDirectory()).c_str());
    
    if(dialog.exec()){
        AppConfig::archive()->writePath("currentFileDialogDirectory", dialog.directory().absolutePath().toStdString());
        string filename = getNativePathString(filesystem::path(dialog.selectedFiles().front().toStdString()));
        loadProject(filename, nullptr, false);
    }
}


void ProjectManagerImpl::openDialogToSaveProject()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Save a choreonoid project file"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    
    QStringList filters;
    filters << _("Project files (*.cnoid)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.setDirectory(AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str());
    if(!currentProjectName.empty()){
        dialog.selectFile(currentProjectName.c_str());
    }

    if(dialog.exec()){
        AppConfig::archive()->writePath("currentFileDialogDirectory", dialog.directory().absolutePath().toStdString());        
        filesystem::path path(dialog.selectedFiles().front().toStdString());
        string filename = getNativePathString(path);
        string ext = filesystem::extension(path);
        if(ext != ".cnoid"){
            filename += ".cnoid";
        }
        saveProject(filename);
    }
}


void ProjectManagerImpl::onPerspectiveCheckToggled(bool on)
{
    AppConfig::archive()->openMapping("ProjectManager")
        ->write("storePerspective", perspectiveCheck->isChecked());
}


void ProjectManagerImpl::onHomeRelativeCheckToggled(bool on)
{
    AppConfig::archive()->openMapping("ProjectManager")
        ->write("useHomeRelative", homeRelativeCheck->isChecked());
}
                                           

void ProjectManager::setArchiver(
    const std::string& moduleName,
    const std::string& name,
    std::function<bool(Archive&)> storeFunction,
    std::function<void(const Archive&)> restoreFunction)
{
    ProjectManagerImpl::ArchiverInfo& info = impl->archivers[moduleName][name];
    info.storeFunction = storeFunction;
    info.restoreFunction = restoreFunction;
}


void ProjectManager::resetArchivers(const std::string& moduleName)
{
    impl->archivers.erase(moduleName);
}


std::string ProjectManager::currentProjectFile() const
{
    if(impl->lastAccessedProjectFile.empty()){
        return "";
    } else {
        return filesystem::absolute(filesystem::path(impl->lastAccessedProjectFile)).string();
    }
}


std::string ProjectManager::currentProjectDirectory() const
{
    if(impl->lastAccessedProjectFile.empty()){
        return "";
    } else {
        filesystem::path projectFilePath(impl->lastAccessedProjectFile);
        return filesystem::absolute(projectFilePath.parent_path()).string();
    }
}
