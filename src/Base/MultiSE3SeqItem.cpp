#include "MultiSE3SeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;


void MultiSE3SeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiSE3SeqItem, AbstractMultiSeqItem>(
        N_("MultiSE3SeqItem"));
    
    ext->itemManager().addCreationPanel<MultiSE3SeqItem>(
        new MultiSeqItemCreationPanel(_("Number of SE3 values in a frame")));
}


MultiSE3SeqItem::MultiSE3SeqItem()
    : seq_(std::make_shared<MultiSE3Seq>())
{

}


MultiSE3SeqItem::MultiSE3SeqItem(std::shared_ptr<MultiSE3Seq> seq)
    : seq_(seq)
{

}


MultiSE3SeqItem::MultiSE3SeqItem(const MultiSE3SeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiSE3Seq>(*org.seq_))
{

}


Item* MultiSE3SeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MultiSE3SeqItem(*this);
}


std::shared_ptr<AbstractMultiSeq> MultiSE3SeqItem::abstractMultiSeq()
{
    return seq_;
}
