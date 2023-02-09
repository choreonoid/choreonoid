#include "MultiVector3SeqItem.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

void MultiVector3SeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiVector3SeqItem, AbstractMultiSeqItem>(N_("MultiVector3SeqItem"));
}


MultiVector3SeqItem::MultiVector3SeqItem()
    : seq_(std::make_shared<MultiVector3Seq>())
{

}


MultiVector3SeqItem::MultiVector3SeqItem(std::shared_ptr<MultiVector3Seq> seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


MultiVector3SeqItem::MultiVector3SeqItem(const MultiVector3SeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiVector3Seq>(*org.seq_))
{

}


MultiVector3SeqItem::MultiVector3SeqItem(const MultiVector3SeqItem& org, std::shared_ptr<MultiVector3Seq> cloneSeq)
    : AbstractMultiSeqItem(org),
      seq_(cloneSeq)
{

}



MultiVector3SeqItem::~MultiVector3SeqItem()
{

}


std::shared_ptr<AbstractMultiSeq> MultiVector3SeqItem::abstractMultiSeq()
{
    return seq_;
}


Item* MultiVector3SeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MultiVector3SeqItem(*this);
}
