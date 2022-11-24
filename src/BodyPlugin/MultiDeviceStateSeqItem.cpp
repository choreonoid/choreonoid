#include "MultiDeviceStateSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

AbstractSeqItem* createMultiDeviceStateSeqItem(std::shared_ptr<AbstractSeq> seq)
{
    auto dseq = dynamic_pointer_cast<MultiDeviceStateSeq>(seq);
    if(dseq){
        auto item = new MultiDeviceStateSeqItem(dseq);
        item->setName("Devices");
        return item;
    }
    return nullptr;
}

}


void MultiDeviceStateSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiDeviceStateSeqItem, AbstractMultiSeqItem>(
        N_("MultiDeviceStateSeqItem"));

    BodyMotionItem::addExtraSeqItemFactory(MultiDeviceStateSeq::key(), createMultiDeviceStateSeqItem);
}

        
MultiDeviceStateSeqItem::MultiDeviceStateSeqItem()
    : seq_(std::make_shared<MultiDeviceStateSeq>())
{

}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(std::shared_ptr<MultiDeviceStateSeq> seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiDeviceStateSeq>(*org.seq_))
{

}


MultiDeviceStateSeqItem::~MultiDeviceStateSeqItem()
{

}


std::shared_ptr<AbstractMultiSeq> MultiDeviceStateSeqItem::abstractMultiSeq()
{
    return seq_;
}


Item* MultiDeviceStateSeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MultiDeviceStateSeqItem(*this);
}


bool MultiDeviceStateSeqItem::store(Archive& archive)
{
    return false;
}


bool MultiDeviceStateSeqItem::restore(const Archive& archive)
{
    return false;
}
