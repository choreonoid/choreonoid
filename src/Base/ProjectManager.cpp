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
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
ProjectManager* instance_ = 0;
}

namespace cnoid {

class ProjectManagerImpl
{
public:
    ProjectManagerImpl(ExtensionManager* em);
    ~ProjectManagerImpl();
        
    template <class TObject>
    void restoreObjectStates(Archive* projectArchive, Archive* states, const vector<TObject*>& objects);
        
    void loadProject(const string& filename, bool isInvokingApplication);

    template<class TObject>
    bool storeObjects(Archive& parentArchive, const char* key, vector<TObject*> objects);
        
    void saveProject(const string& filename);
    void overwriteCurrentProject();
        
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    void openDialogToLoadProject();
    void openDialogToSaveProject();

    void onPerspectiveCheckToggled();
        
    void connectArchiver(
        const std::string& name,
        boost::function<bool(Archive&)> storeFunction,
        boost::function<void(const Archive&)> restoreFunction);

    ItemTreeArchiver itemTreeArchiver;
    MainWindow* mainWindow;
    MessageView* messageView;
    string lastAccessedProjectFile;
    Action* perspectiveCheck;

    struct ArchiverInfo {
        boost::function<bool(Archive&)> storeFunction;
        boost::function<void(const Archive&)> restoreFunction;
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
    impl = new ProjectManagerImpl(em);
}


ProjectManagerImpl::ProjectManagerImpl(ExtensionManager* em)
{
    MappingPtr config = AppConfig::archive()->openMapping("ProjectManager");
    
    MenuManager& mm = em->menuManager();

    mm.setPath("/File");

    mm.addItem(_("Open Project"))
        ->sigTriggered().connect(bind(&ProjectManagerImpl::openDialogToLoadProject, this));
    mm.addItem(_("Save Project"))
        ->sigTriggered().connect(bind(&ProjectManagerImpl::overwriteCurrentProject, this));
    mm.addItem(_("Save Project As"))
        ->sigTriggered().connect(bind(&ProjectManagerImpl::openDialogToSaveProject, this));
    
    perspectiveCheck = mm.setPath(N_("Project File Options")).addCheckItem(_("Perspective"));
    perspectiveCheck->setChecked(config->get("storePerspective", false));
    perspectiveCheck->sigToggled().connect(bind(&ProjectManagerImpl::onPerspectiveCheckToggled, this));

    mm.setPath("/File");
    mm.addSeparator();

    OptionManager& om = em->optionManager();
    om.addOption("project", program_options::value< vector<string> >(), "load a project file");
    om.addPositionalOption("project", 1);
    om.sigOptionsParsed().connect(bind(&ProjectManagerImpl::onSigOptionsParsed, this, _1));

    mainWindow = MainWindow::instance();
    messageView = MessageView::mainInstance();
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
void ProjectManagerImpl::restoreObjectStates
(Archive* projectArchive, Archive* states, const vector<TObject*>& objects)
{
    for(size_t i=0; i < objects.size(); ++i){
        TObject* object = objects[i];
        Archive* state = states->findSubArchive(object->objectName().toStdString());
        if(state->isValid()){
            state->inheritSharedInfoFrom(*projectArchive);
            object->restoreState(*state);
        }
    }
}


void ProjectManager::loadProject(const std::string& filename)
{
    impl->loadProject(filename, false);
}


void ProjectManagerImpl::loadProject(const std::string& filename, bool isInvokingApplication)
{
    YAMLReader reader;
    reader.setMappingClass<Archive>();

    try {
        messageView->putln();
        messageView->notify(str(fmt(_("Loading project file \"%1%\" ...")) % filename));

        if(!isInvokingApplication){
            messageView->flush();
        }
        
        bool result = false;
        if(!reader.load(filename)){
            messageView->put(reader.errorMessage() + "\n");

        } else if(reader.numDocuments() == 0){
            messageView->put(_("The project file is empty.\n"));

        } else {
            Archive* archive = static_cast<Archive*>(reader.document()->toMapping());
            archive->initSharedInfo(filename);

            ViewManager::ViewStateInfo viewStateInfo = ViewManager::restoreViews(archive, "views");

            MainWindow* mainWindow = MainWindow::instance();
            if(isInvokingApplication){
                if(perspectiveCheck->isChecked()){
                    mainWindow->setInitialLayout(archive);
                }
                mainWindow->show();
                messageView->flush();
                mainWindow->repaint();
            } else {
                if(perspectiveCheck->isChecked()){
                    mainWindow->restoreLayout(archive);
                }
            }

            // Restoring the file dialog directory from the project file.
            // This should be disabled.
            /*
              string currentFileDialogDirectory;
              if(archive->readRelocatablePath("currentFileDialogDirectory", currentFileDialogDirectory)){
              AppConfig::archive()->writePath(
              "currentFileDialogDirectory", currentFileDialogDirectory);
              }
            */

            Archive* items = archive->findSubArchive("items");
            if(items->isValid()){
                items->inheritSharedInfoFrom(*archive);
                result = itemTreeArchiver.restore(items, RootItem::mainInstance());
            }

            if(!ViewManager::restoreViewStates(viewStateInfo)){
                // load the old format (version 1.4 or earlier)
                Archive* viewStates = archive->findSubArchive("views");
                if(viewStates->isValid()){
                    restoreObjectStates(archive, viewStates, ViewManager::allViews());
                }
            }

            Archive* barStates = archive->findSubArchive("toolbars");
            if(barStates->isValid()){
                restoreObjectStates(archive, barStates, mainWindow->allToolBars());
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
                        }
                    }
                }
            }

            callLater(bind(&Archive::callPostProcesses, ArchivePtr(archive)));
        }
        if(result){
            messageView->notify(str(fmt(_("Project \"%1%\" has successfully been loaded.")) % filename));
            lastAccessedProjectFile = filename;
        } else {
            messageView->notify(str(fmt(_("Project \"%1%\" cannot be loaded.")) % filename));
            lastAccessedProjectFile.clear();
        }
    } catch (const ValueNode::Exception& ex){
        messageView->put(ex.message());
    }
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
    messageView->putln();
    messageView->notify(str(fmt(_("Saving a project to \"%1%\" ...\n")) % filename));
    messageView->flush();
    
    ArchivePtr archive = new Archive();
    archive->initSharedInfo(filename);

    ArchivePtr itemArchive = itemTreeArchiver.store(archive, RootItem::mainInstance());

    if(itemArchive){
        archive->insert("items", itemArchive);
    }

    bool stored = ViewManager::storeViewStates(archive, "views");
    
    stored |= storeObjects(*archive, "toolbars", mainWindow->allToolBars());

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
        MainWindow::instance()->storeLayout(archive);
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
        YAMLWriter writer(filename);
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive);
        messageView->notify(_("Saving a project file has been finished.\n"));
        lastAccessedProjectFile = filename;
    } else {
        messageView->notify(_("Saving a project file failed.\n"));
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

    
void ProjectManagerImpl::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("project")){
        vector<string> projectFileNames = v["project"].as< vector<string> >();
        for(size_t i=0; i < projectFileNames.size(); ++i){
            loadProject(toActualPathName(projectFileNames[i]), true);
        }
    }
}


void ProjectManagerImpl::openDialogToLoadProject()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Open a choreonoid project file"));
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
        loadProject(getNativePathString(filesystem::path(dialog.selectedFiles().front().toStdString())), false);
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


void ProjectManagerImpl::onPerspectiveCheckToggled()
{
    AppConfig::archive()->openMapping("ProjectManager")
        ->write("storePerspective", perspectiveCheck->isChecked());
}


void ProjectManager::setArchiver(
    const std::string& moduleName,
    const std::string& name,
    boost::function<bool(Archive&)> storeFunction,
    boost::function<void(const Archive&)> restoreFunction)
{
    ProjectManagerImpl::ArchiverInfo& info = impl->archivers[moduleName][name];
    info.storeFunction = storeFunction;
    info.restoreFunction = restoreFunction;
}


void ProjectManager::resetArchivers(const std::string& moduleName)
{
    impl->archivers.erase(moduleName);
}
