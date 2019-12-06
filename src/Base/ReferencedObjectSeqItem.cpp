/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ReferencedObjectSeqItem.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;


void ReferencedObjectSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ReferencedObjectSeqItem, AbstractSeqItem>(N_("ReferencedObjectSeqItem"));
}


ReferencedObjectSeqItem::ReferencedObjectSeqItem()
    : seq_(std::make_shared<ReferencedObjectSeq>())
{

}


ReferencedObjectSeqItem::ReferencedObjectSeqItem(std::shared_ptr<ReferencedObjectSeq> seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


ReferencedObjectSeqItem::ReferencedObjectSeqItem(const ReferencedObjectSeqItem& org)
    : AbstractSeqItem(org),
      seq_(std::make_shared<ReferencedObjectSeq>(*org.seq_))
{

}


ReferencedObjectSeqItem::ReferencedObjectSeqItem
(const ReferencedObjectSeqItem& org, std::shared_ptr<ReferencedObjectSeq> cloneSeq)
    : AbstractSeqItem(org),
      seq_(cloneSeq)
{

}



ReferencedObjectSeqItem::~ReferencedObjectSeqItem()
{

}


std::shared_ptr<AbstractSeq> ReferencedObjectSeqItem::abstractSeq()
{
    return seq_;
}


Item* ReferencedObjectSeqItem::doDuplicate() const
{
    return new ReferencedObjectSeqItem(*this);
}
