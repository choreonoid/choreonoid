#include "SubProjectItem.h"
#include "ItemManager.h"
#include "ProjectManager.h"
#include "MessageView.h"
#include "ItemTreeView.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/ConnectionSet>
#include <cnoid/Format>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

std::set<string> projectFilesBeingLoaded;

}

namespace cnoid {

class SubProjectItem::Impl
{
public:
    SubProjectItem* self;
    std::string projectFileToLoad;
    Selection saveMode;
    bool isSavingSubProject;
    unique_ptr<ProjectManager> projectManager_;

    Impl(SubProjectItem* self);
    Impl(SubProjectItem* self, const Impl& org);
    bool loadSubProject(const std::string& filename);
    ProjectManager* projectManager();
    void doLoadSubProject(const std::string& filename);
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
    impl = new Impl(this);
}


SubProjectItem::Impl::Impl(SubProjectItem* self)
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
    impl = new Impl(this, *org.impl);
}


SubProjectItem::Impl::Impl(SubProjectItem* self, const Impl& org)
    : Impl(self)
{
    projectFileToLoad = org.projectFileToLoad;
    saveMode = org.saveMode;
}


SubProjectItem::~SubProjectItem()
{
    delete impl;
}


Item* SubProjectItem::doCloneItem(CloneMap* /* cloneMap */) const
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


bool SubProjectItem::Impl::loadSubProject(const std::string& filename)
{
    if(projectFilesBeingLoaded.find(filename) != projectFilesBeingLoaded.end()){
        MessageView::instance()->putln(
            formatR(_("Sub projects to load \"{}\" are recursively specified."),
            filename), MessageView::Error);
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


ProjectManager* SubProjectItem::Impl::projectManager()
{
    if(!projectManager_){
        projectManager_.reset(new ProjectManager());
    }
    return projectManager_.get();
}


void SubProjectItem::Impl::doLoadSubProject(const std::string& filename)
{
    projectFilesBeingLoaded.insert(filename);

    auto pm = projectManager();
    pm->loadProject(filename, self);

    projectFilesBeingLoaded.erase(filename);

    if(!pm->isLoadingProject()){
        if(auto itemTreeView = ItemTreeView::findInstance()){
            itemTreeView->itemTreeWidget()->setExpanded(self);
        }
    }
}


bool SubProjectItem::Impl::saveSubProject(const std::string& filename)
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
            setConsistentWithFile(true);
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
        if(impl->saveMode.is(SubProjectItem::AUTOMATIC_SAVE)){
            if(!ProjectManager::checkIfItemsConsistentWithProjectArchive(this)){
                // To save the sub-tree as a sub-project file
                suggestFileUpdate();
            }
        }
        if(!archive.saveItemToFile(this)){
            return false;
        }
        archive.write("save_mode", impl->saveMode.selectedSymbol(), DOUBLE_QUOTED);
    }
    return true;
}


bool SubProjectItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read({ "save_mode", "saveMode" }, symbol)){
        impl->saveMode.select(symbol);
    }
    return archive.loadFileTo(this);
}
