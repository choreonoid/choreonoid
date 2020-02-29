#include "CoordinateFrameListSuiteItem.h"
#include "CoordinateFrameListItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameList>
#include <cnoid/CoordinateFrameSetSuite>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameListSuiteItem::Impl
{
public:
    CoordinateFrameListSuiteItem* self;
    CoordinateFrameSetSuitePtr frameLists;
    vector<CoordinateFrameListItemPtr> frameListItems;

    Impl(CoordinateFrameListSuiteItem* self);
    Impl(CoordinateFrameListSuiteItem* self, std::initializer_list<std::string> frameSetNames);
    Impl(CoordinateFrameListSuiteItem* self, const Impl& org);
};

}


void CoordinateFrameListSuiteItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameListSuiteItem>(N_("CoordinateFrameListSuiteItem"));
}


CoordinateFrameListSuiteItem::CoordinateFrameListSuiteItem()
{
    impl = new Impl(this);
}


CoordinateFrameListSuiteItem::Impl::Impl(CoordinateFrameListSuiteItem* self)
    : self(self)
{
    frameLists = new CoordinateFrameSetSuite;
}


CoordinateFrameListSuiteItem::CoordinateFrameListSuiteItem(std::initializer_list<std::string> frameSetNames)
{
    impl = new Impl(this, frameSetNames);
}


CoordinateFrameListSuiteItem::Impl::Impl
(CoordinateFrameListSuiteItem* self, std::initializer_list<std::string> frameSetNames)
    : self(self)
{
    const int n = frameSetNames.size();
    frameLists = new CoordinateFrameSetSuite(n);
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


CoordinateFrameListSuiteItem::CoordinateFrameListSuiteItem(const CoordinateFrameListSuiteItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


CoordinateFrameListSuiteItem::Impl::Impl(CoordinateFrameListSuiteItem* self, const Impl& org)
    : self(self)
{
    const int n = org.frameLists->numFrameSets();
    frameLists = new CoordinateFrameSetSuite(n);
    frameListItems.resize(n);

    int index = 0;
    for(int i=0; i < n; ++i){
        auto item = new CoordinateFrameListItem(*org.frameListItems[i]);
        frameLists->setFrameSet(i, item->frameList());
        self->addSubItem(item);
        frameListItems[i] = item;
    }
}


CoordinateFrameListSuiteItem::~CoordinateFrameListSuiteItem()
{
    delete impl;
}


Item* CoordinateFrameListSuiteItem::doDuplicate() const
{
    return new CoordinateFrameListSuiteItem(*this);
}


void CoordinateFrameListSuiteItem::replaceFrameListContainer(CoordinateFrameSetSuite* container)
{
    const int n = impl->frameLists->numFrameSets();
    container->setNumFrameSets(n);
    for(int i=0; i < n; ++i){
        container->setFrameSet(i, impl->frameLists->frameSet(i));
    }
    impl->frameLists = container;
}
    

CoordinateFrameSetSuite* CoordinateFrameListSuiteItem::frameSetSuite()
{
    return impl->frameLists;
}


const CoordinateFrameSetSuite* CoordinateFrameListSuiteItem::frameSetSuite() const
{
    return impl->frameLists;
}


int CoordinateFrameListSuiteItem::numFrameLists() const
{
    return impl->frameLists->numFrameSets();
}


void CoordinateFrameListSuiteItem::setNumFrameLists(int n)
{
    impl->frameListItems.resize(n);
    impl->frameLists->setNumFrameSets(n);
}


CoordinateFrameList* CoordinateFrameListSuiteItem::frameList(int index)
{
    return dynamic_cast<CoordinateFrameList*>(impl->frameLists->frameSet(index));
}


const CoordinateFrameList* CoordinateFrameListSuiteItem::frameList(int index) const
{
    return dynamic_cast<CoordinateFrameList*>(impl->frameLists->frameSet(index));
}


CoordinateFrameListItem* CoordinateFrameListSuiteItem::frameListItem(int index)
{
    return impl->frameListItems[index];
}


const CoordinateFrameListItem* CoordinateFrameListSuiteItem::frameListItem(int index) const
{
    return impl->frameListItems[index];
}


void CoordinateFrameListSuiteItem::setFrameListItem(int index, CoordinateFrameListItem* item)
{
    if(index >= static_cast<int>(impl->frameListItems.size())){
        setNumFrameLists(index + 1);
    }
    impl->frameLists->setFrameSet(index, item->frameList());
    addSubItem(item);
    impl->frameListItems[index] = item;
}


void CoordinateFrameListSuiteItem::setFrameListEnabled(int index, bool on)
{
    auto item = impl->frameListItems[index];
    if(on && !item->parentItem()){
        addSubItem(item);
    } else if(!on && item->parentItem()){
        item->detachFromParentItem();
    }
}
    

bool CoordinateFrameListSuiteItem::isFrameListEnabled(int index) const
{
    return impl->frameListItems[index]->parentItem() != nullptr;
}
    

bool CoordinateFrameListSuiteItem::store(Archive& archive)
{
    ListingPtr nodes = new Listing;
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
            nodes->append(subArchive);
        }
    }
    if(!nodes->empty()){
        archive.insert("frame_lists", nodes);
    }
    return true;
}


bool CoordinateFrameListSuiteItem::restore(const Archive& archive)
{
    auto nodes = archive.findListing("frame_lists");
    if(!nodes->isValid()){
        nodes = archive.findListing("frameLists"); // Old
    }
    if(nodes->isValid()){
        for(auto& node : *nodes){
            auto listNode = node->toMapping();
            int id = listNode->find("id")->toInt();
            if(id >= 0 && id < impl->frameListItems.size()){
                auto& subArchive = *const_cast<Archive&>(archive).subArchive(listNode);
                if(!impl->frameListItems[id]->restore(subArchive)){
                    // Put error messages
                }
            }
        }
    }
    return true;
}
