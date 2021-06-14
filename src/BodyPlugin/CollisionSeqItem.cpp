/**
   @file
   @author Shizuko Hattori
*/

#include "CollisionSeqItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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


static bool loadStandardYamlFormat(CollisionSeqItem* item, const std::string& filename, std::ostream& os)
{
    return item->collisionSeq()->loadStandardYAMLformat(filename, os);
}


static bool saveAsStandardYamlFormat(CollisionSeqItem* item, const std::string& filename, std::ostream& os)
{
    if(!item->collisionSeq()->saveAsStandardYAMLformat(filename)){
        os << item->collisionSeq()->seqMessage() << endl;
        return false;
    }
    return true;
}


void CollisionSeqItem::initislizeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(initialized){
        return;
    }

    ItemManager& im = ext->itemManager();

    im.registerClass<CollisionSeqItem, AbstractMultiSeqItem>(N_("CollisionSeqItem"));
    im.addLoaderAndSaver<CollisionSeqItem>(
        _("Collision Data"), "COLLISION-DATA-YAML", "yaml",
        [](CollisionSeqItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return loadStandardYamlFormat(item, filename, os);
        },
        [](CollisionSeqItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return saveAsStandardYamlFormat(item, filename, os);
        });

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
    if(overwrite()){
        return archive.writeFileInformation(this);
    }
    return false;
}


bool CollisionSeqItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}


