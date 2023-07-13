#include "MultiDeviceStateSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void MultiDeviceStateSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiDeviceStateSeqItem, AbstractMultiSeqItem>(
        N_("MultiDeviceStateSeqItem"));

    BodyMotionItem::registerExtraSeqItemFactory(
        MultiDeviceStateSeq::key(),
        [](std::shared_ptr<AbstractSeq> seq) -> AbstractSeqItem* {
            MultiDeviceStateSeqItem* item = nullptr;
            if(auto dseq = dynamic_pointer_cast<MultiDeviceStateSeq>(seq)){
                item = new MultiDeviceStateSeqItem(dseq);
                item->setName("DeviceState");
            }
            return item;
        });
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
