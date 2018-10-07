/**
   @author Shin'ichiro Nakaoka
*/

#include "SubProjectItem.h"
#include "ItemManager.h"
#include "ProjectManager.h"
#include "MessageView.h"
#include "ItemTreeView.h"
#include <cnoid/ConnectionSet>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace {
std::set<string> projectFilesBeingLoaded;
}

namespace cnoid {

class SubProjectItemImpl
{
public:
    SubProjectItem* self;
    std::string projectFileToLoad;
    bool isLoadingMainProject;
    bool isSavingSubProject;
    ScopedConnectionSet updateConnections;
    unique_ptr<ProjectManager> subProjectManager;

    SubProjectItemImpl(SubProjectItem* self);
    SubProjectItemImpl(SubProjectItem* self, const SubProjectItemImpl& org);
    ProjectManager* getProjectManager();
    ProjectManager* getSubProjectManager();
    void doLoadSubProject(const std::string& filename);
    void onSubProjectUpdated();
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
                return item->loadSubProject(filename);
            },
            [](SubProjectItem* item, const std::string& filename, std::ostream&, Item*){
                return item->saveSubProject(filename);
            });
        initialized = true;
    }
}


SubProjectItem::SubProjectItem()
{
    impl = new SubProjectItemImpl(this);
}


SubProjectItemImpl::SubProjectItemImpl(SubProjectItem* self)
    : self(self)
{
    isLoadingMainProject = false;
    isSavingSubProject = false;
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


bool SubProjectItem::loadSubProject(const std::string& filename)
{
    if(projectFilesBeingLoaded.find(filename) != projectFilesBeingLoaded.end()){
        MessageView::instance()->putln(
            format(_("Sub projects to load \"%1%\" are recursively specified."))
            % filename, MessageView::ERROR);
        return false;
    }

    if(isConnectedToRoot()){
        impl->doLoadSubProject(filename);
        return true;
    } else {
        impl->projectFileToLoad = filename;
        return true;
    }

    return false;
}


ProjectManager* SubProjectItemImpl::getProjectManager()
{
    auto mainProjectManager = ProjectManager::instance();
    isLoadingMainProject = mainProjectManager->isLoadingProject();
    if(!isLoadingMainProject){
        return mainProjectManager;
    }
    return getSubProjectManager();
}


ProjectManager* SubProjectItemImpl::getSubProjectManager()
{
    if(!subProjectManager){
        subProjectManager.reset(new ProjectManager());
    }
    return subProjectManager.get();
}


void SubProjectItemImpl::doLoadSubProject(const std::string& filename)
{
    projectFilesBeingLoaded.insert(filename);

    auto items = getProjectManager()->loadProject(filename, self);

    updateConnections.disconnect();
    for(auto& item : items){
        updateConnections.add(
            item->sigNameChanged().connect(
                [&](const std::string&){ onSubProjectUpdated(); }));
        updateConnections.add(
            item->sigUpdated().connect(
                [&](){ onSubProjectUpdated(); }));
    }
    self->sigSubTreeChanged().connect([&](){ onSubProjectUpdated(); });

    projectFilesBeingLoaded.erase(filename);

    if(!isLoadingMainProject){
        ItemTreeView::instance()->expandItem(self);
    }
}


void SubProjectItemImpl::onSubProjectUpdated()
{
    updateConnections.disconnect();
    self->suggestFileUpdate();
    self->notifyUpdate();
}
    

bool SubProjectItem::saveSubProject(const std::string& filename)
{
    impl->isSavingSubProject = true;
    impl->getSubProjectManager()->saveProject(filename, this);
    impl->isSavingSubProject = false;
    return true;
}


bool SubProjectItem::isSavingSubProject() const
{
    return impl->isSavingSubProject;
}


void SubProjectItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Updated"), !isConsistentWithFile());
}


bool SubProjectItem::store(Archive& archive)
{
    if(!impl->isSavingSubProject){
        if(overwrite()){
            archive.writeRelocatablePath("filename", filePath());
            archive.write("format", fileFormat());
        }
    }
    return true;
}


bool SubProjectItem::restore(const Archive& archive)
{
    std::string filename, format;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", format)){
        return load(filename, format);
    }
    return false;
}
