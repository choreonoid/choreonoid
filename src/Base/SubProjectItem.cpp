/**
   @author Shin'ichiro Nakaoka
*/

#include "SubProjectItem.h"
#include "ItemManager.h"
#include "ProjectManager.h"
#include "MessageView.h"
#include "ItemTreeView.h"
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
std::set<string> projectFilesBeingLoaded;
}

namespace cnoid {

class SubProjectItemImpl
{
public:
    SubProjectItem* self;
    std::string projectFileToLoad;
    Selection saveMode;
    bool isSavingSubProject;
    ScopedConnectionSet updateConnections;
    unique_ptr<ProjectManager> projectManager_;

    SubProjectItemImpl(SubProjectItem* self);
    SubProjectItemImpl(SubProjectItem* self, const SubProjectItemImpl& org);
    bool loadSubProject(const std::string& filename);
    ProjectManager* projectManager();
    void doLoadSubProject(const std::string& filename);
    void enableSubProjectUpdateDetection();
    void onSubProjectUpdated();
    bool saveSubProject(const std::string& filename);
};

}

void SubProjectItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<SubProjectItem>(N_("SubProjectItem"));
        im.addLoaderAndSaver<SubProjectItem>(
            _("SubProjet"), "PROJECT", "cnoid",
            [](SubProjectItem* item, const std::string& filename, std::ostream&, Item*){
                return item->impl->loadSubProject(filename);
            },
            [](SubProjectItem* item, const std::string& filename, std::ostream&, Item*){
                return item->impl->saveSubProject(filename);
            });
        initialized = true;
    }
}


SubProjectItem::SubProjectItem()
{
    impl = new SubProjectItemImpl(this);
}


SubProjectItemImpl::SubProjectItemImpl(SubProjectItem* self)
    : self(self),
      saveMode(SubProjectItem::N_SAVE_MODE, CNOID_GETTEXT_DOMAIN_NAME)
{
    isSavingSubProject = false;

    saveMode.setSymbol(SubProjectItem::MANUAL_SAVE, N_("Manual save"));
    saveMode.setSymbol(SubProjectItem::AUTOMATIC_SAVE, N_("Automatic save"));
    saveMode.select(SubProjectItem::MANUAL_SAVE);
}


SubProjectItem::SubProjectItem(const SubProjectItem& org)
    : Item(org)
{
    impl = new SubProjectItemImpl(this, *org.impl);
}


SubProjectItemImpl::SubProjectItemImpl(SubProjectItem* self, const SubProjectItemImpl& org)
    : SubProjectItemImpl(self)
{
    projectFileToLoad = org.projectFileToLoad;
    saveMode = org.saveMode;
}


SubProjectItem::~SubProjectItem()
{
    delete impl;
}


Item* SubProjectItem::doDuplicate() const
{
    return new SubProjectItem(*this);
}


void SubProjectItem::onConnectedToRoot()
{
    if(!impl->projectFileToLoad.empty()){
        impl->doLoadSubProject(impl->projectFileToLoad);
        impl->projectFileToLoad.clear();
    }
}


bool SubProjectItemImpl::loadSubProject(const std::string& filename)
{
    if(projectFilesBeingLoaded.find(filename) != projectFilesBeingLoaded.end()){
        MessageView::instance()->putln(
            format(_("Sub projects to load \"{}\" are recursively specified."),
            filename), MessageView::ERROR);
        return false;
    }

    if(self->isConnectedToRoot()){
        doLoadSubProject(filename);
        return true;
    } else {
        projectFileToLoad = filename;
        return true;
    }

    return false;
}


ProjectManager* SubProjectItemImpl::projectManager()
{
    if(!projectManager_){
        projectManager_.reset(new ProjectManager());
    }
    return projectManager_.get();
}


void SubProjectItemImpl::doLoadSubProject(const std::string& filename)
{
    projectFilesBeingLoaded.insert(filename);

    auto items = projectManager()->loadProject(filename, self);

    projectFilesBeingLoaded.erase(filename);

    if(!ProjectManager::isProjectBeingLoaded()){
        ItemTreeView::instance()->expandItem(self);
    }

    if(saveMode.is(SubProjectItem::AUTOMATIC_SAVE)){
        enableSubProjectUpdateDetection();
    }
}


void SubProjectItemImpl::enableSubProjectUpdateDetection()
{
    updateConnections.disconnect();

    ItemList<> subTreeItems;
    subTreeItems.extractSubTreeItems(self);

    for(auto& item : subTreeItems){
        updateConnections.add(
            item->sigNameChanged().connect(
                [&](const std::string&){ onSubProjectUpdated(); }));
        updateConnections.add(
            item->sigUpdated().connect(
                [&](){ onSubProjectUpdated(); }));
    }

    updateConnections.add(
        self->sigSubTreeChanged().connect(
            [&](){ onSubProjectUpdated(); }));
}


void SubProjectItemImpl::onSubProjectUpdated()
{
    self->suggestFileUpdate();
    self->notifyUpdate();
}
    

bool SubProjectItemImpl::saveSubProject(const std::string& filename)
{
    isSavingSubProject = true;
    projectManager()->saveProject(filename, self);
    isSavingSubProject = false;
    return true;
}


bool SubProjectItem::isSavingSubProject() const
{
    return impl->isSavingSubProject;
}


int SubProjectItem::saveMode() const
{
    return impl->saveMode.which();
}


void SubProjectItem::setSaveMode(int mode)
{
    if(impl->saveMode.select(mode)){
        if(mode == MANUAL_SAVE){
            impl->updateConnections.disconnect();
            setConsistentWithFile(true);
        } else {
            impl->enableSubProjectUpdateDetection();
        }
    }
}


void SubProjectItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Save mode"), impl->saveMode,
                [&](int index){ setSaveMode(index); return true; });

    if(impl->saveMode.is(AUTOMATIC_SAVE)){
        putProperty(_("Updated"), !isConsistentWithFile());
    }
}


bool SubProjectItem::store(Archive& archive)
{
    if(!impl->isSavingSubProject){
        if(overwrite()){
            archive.writeRelocatablePath("filename", filePath());
            archive.write("format", fileFormat());
            archive.write("saveMode", impl->saveMode.selectedSymbol(), DOUBLE_QUOTED);
        }
    }
    return true;
}


bool SubProjectItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("saveMode", symbol)){
        impl->saveMode.select(symbol);
    }
    string filename, format;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", format)){
        return load(filename, format);
    }
    return false;
}
