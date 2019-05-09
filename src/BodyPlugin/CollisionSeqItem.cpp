/**
   @file
   @author Shizuko Hattori
*/

#include "CollisionSeqItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

}


namespace cnoid {

class CollisionSeqItemImpl
{
public:
    CollisionSeqItem* self;

    CollisionSeqItemImpl(CollisionSeqItem* self);
    ~CollisionSeqItemImpl();
    void initialize();

};
}


static bool fileIoSub(CollisionSeqItem* item, std::ostream& os, bool loaded, bool isLoading)
{
    if(!loaded){
        os << item->collisionSeq()->seqMessage();
    }
    return loaded;
}


static bool loadStandardYamlFormat(CollisionSeqItem* item, const std::string& filename, std::ostream& os)
{
    return fileIoSub(item, os, item->collisionSeq()->loadStandardYAMLformat(filename), true);
}


static bool saveAsStandardYamlFormat(CollisionSeqItem* item, const std::string& filename, std::ostream& os)
{
    return fileIoSub(item, os, item->collisionSeq()->saveAsStandardYAMLformat(filename), false);
}


void CollisionSeqItem::initislizeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(initialized){
        return;
    }

    ItemManager& im = ext->itemManager();

    im.registerClass<CollisionSeqItem>(N_("CollisionSeqItem"));
    im.addLoaderAndSaver<CollisionSeqItem>(
        _("Collision Data"), "COLLISION-DATA-YAML", "yaml",
        std::bind(loadStandardYamlFormat, _1, _2, _3),  std::bind(saveAsStandardYamlFormat, _1, _2, _3));

    initialized = true;
}


CollisionSeqItem::CollisionSeqItem()
    : collisionSeq_(new CollisionSeq(this))
{
    impl = new CollisionSeqItemImpl(this);
}


CollisionSeqItem::CollisionSeqItem(const CollisionSeqItem& org)
    : AbstractMultiSeqItem(org),
      collisionSeq_(new CollisionSeq(this))
{
    impl = new CollisionSeqItemImpl(this);
}


CollisionSeqItemImpl::CollisionSeqItemImpl(CollisionSeqItem* self)
    :self(self)
{
    initialize();
}


void CollisionSeqItemImpl::initialize()
{

}


CollisionSeqItem::~CollisionSeqItem()
{
    delete impl;
}


CollisionSeqItemImpl::~CollisionSeqItemImpl()
{

}


std::shared_ptr<AbstractMultiSeq> CollisionSeqItem::abstractMultiSeq()
{
    return collisionSeq_;
}


Item* CollisionSeqItem::doDuplicate() const
{
    return new CollisionSeqItem(*this);
}


bool CollisionSeqItem::store(Archive& archive)
{
    if(overwrite() || !filePath().empty()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        return true;
    }
    return false;
}


bool CollisionSeqItem::restore(const Archive& archive)
{
    std::string filename, format;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", format)){
        if(load(filename, format)){
            return true;
        }
    }
    return false;
}


