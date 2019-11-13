#include "MultiCoordinateFrameListItem.h"
#include "CoordinateFrameListItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameList>
#include <cnoid/MultiCoordinateFrameSet>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class MultiCoordinateFrameListItem::Impl
{
public:
    MultiCoordinateFrameListItem* self;
    MultiCoordinateFrameSetPtr frameLists;
    vector<CoordinateFrameListItemPtr> frameListItems;

    Impl(MultiCoordinateFrameListItem* self);
    Impl(MultiCoordinateFrameListItem* self, std::initializer_list<std::string> frameSetNames);
    Impl(MultiCoordinateFrameListItem* self, const Impl& org);
};

}


void MultiCoordinateFrameListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<MultiCoordinateFrameListItem>(N_("MultiCoordinateFrameListItem"));
}


MultiCoordinateFrameListItem::MultiCoordinateFrameListItem()
{
    impl = new Impl(this);
}


MultiCoordinateFrameListItem::Impl::Impl(MultiCoordinateFrameListItem* self)
    : self(self)
{
    frameLists = new MultiCoordinateFrameSet;
}


MultiCoordinateFrameListItem::MultiCoordinateFrameListItem(std::initializer_list<std::string> frameSetNames)
{
    impl = new Impl(this, frameSetNames);
}


MultiCoordinateFrameListItem::Impl::Impl
(MultiCoordinateFrameListItem* self, std::initializer_list<std::string> frameSetNames)
    : self(self)
{
    const int n = frameSetNames.size();
    frameLists = new MultiCoordinateFrameSet(n);
    frameListItems.resize(n);

    int i = 0;
    for(auto& name : frameSetNames){
        auto item = new CoordinateFrameListItem;
        item->setName(name);
        frameLists->setFrameSet(i, item->frameList());
        self->addSubItem(item);
        frameListItems[i] = item;
        ++i;
    }
}


MultiCoordinateFrameListItem::MultiCoordinateFrameListItem(const MultiCoordinateFrameListItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


MultiCoordinateFrameListItem::Impl::Impl(MultiCoordinateFrameListItem* self, const Impl& org)
    : self(self)
{
    const int n = org.frameLists->numFrameSets();
    frameLists = new MultiCoordinateFrameSet(n);
    frameListItems.resize(n);

    int index = 0;
    for(int i=0; i < n; ++i){
        auto item = new CoordinateFrameListItem(*org.frameListItems[i]);
        frameLists->setFrameSet(i, item->frameList());
        self->addSubItem(item);
        frameListItems[i] = item;
    }
}


MultiCoordinateFrameListItem::~MultiCoordinateFrameListItem()
{
    delete impl;
}


Item* MultiCoordinateFrameListItem::doDuplicate() const
{
    return new MultiCoordinateFrameListItem(*this);
}


void MultiCoordinateFrameListItem::replaceFrameListContainer(MultiCoordinateFrameSet* container)
{
    const int n = impl->frameLists->numFrameSets();
    container->setNumFrameSets(n);
    for(int i=0; i < n; ++i){
        container->setFrameSet(i, impl->frameLists->frameSet(i));
    }
    impl->frameLists = container;
}
    

MultiCoordinateFrameSet* MultiCoordinateFrameListItem::frameSets()
{
    return impl->frameLists;
}


const MultiCoordinateFrameSet* MultiCoordinateFrameListItem::frameSets() const
{
    return impl->frameLists;
}


int MultiCoordinateFrameListItem::numFrameLists() const
{
    return impl->frameLists->numFrameSets();
}


CoordinateFrameList* MultiCoordinateFrameListItem::frameList(int index)
{
    return dynamic_cast<CoordinateFrameList*>(impl->frameLists->frameSet(index));
}


const CoordinateFrameList* MultiCoordinateFrameListItem::frameList(int index) const
{
    return dynamic_cast<CoordinateFrameList*>(impl->frameLists->frameSet(index));
}


CoordinateFrameListItem* MultiCoordinateFrameListItem::frameListItem(int index)
{
    return impl->frameListItems[index];
}


const CoordinateFrameListItem* MultiCoordinateFrameListItem::frameListItem(int index) const
{
    return impl->frameListItems[index];
}


void MultiCoordinateFrameListItem::setFrameListEnabled(int index, bool on)
{
    auto item = impl->frameListItems[index];
    if(on && !item->parentItem()){
        addSubItem(item);
    } else if(!on && item->parentItem()){
        item->detachFromParentItem();
    }
}
    

bool MultiCoordinateFrameListItem::isFrameListEnabled(int index) const
{
    return impl->frameListItems[index]->parentItem() != nullptr;
}
    

bool MultiCoordinateFrameListItem::store(Archive& archive)
{
    auto& nodes = *archive.createListing("frameLists");
    const int n = impl->frameListItems.size();
    for(int i=0; i < n; ++i){
        auto item = impl->frameListItems[i];
        if(item->frameList()->numFrames() > 0){
            ArchivePtr subArchive = new Archive;
            subArchive->inheritSharedInfoFrom(archive);
            subArchive->write("id", i);
            if(!item->store(*subArchive)){
                return false;
            }
            nodes.append(subArchive);
        }
    }
    return true;
}


bool MultiCoordinateFrameListItem::restore(const Archive& archive)
{
    bool loaded = false;
    
    auto& nodes = *archive.findListing("frameLists");
    if(nodes.isValid()){
        for(auto& node : nodes){
            auto listNode = node->toMapping();
            int id = listNode->find("id")->toInt();
            if(id >= 0 && id < impl->frameListItems.size()){
                auto& subArchive = *const_cast<Archive&>(archive).subArchive(listNode);
                if(impl->frameListItems[id]->restore(subArchive)){
                    loaded = true;
                }
            }
        }
    }
    return loaded;
}
