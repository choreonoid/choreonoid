#include "MultiSE3MatrixSeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;


void MultiSE3MatrixSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiSE3MatrixSeqItem, AbstractMultiSeqItem>(
        N_("MultiSE3MatrixSeqItem"));
    
    ext->itemManager().addCreationPanel<MultiSE3MatrixSeqItem>(
        new MultiSeqItemCreationPanel(_("Number of SE3 values in a frame")));
}


MultiSE3MatrixSeqItem::MultiSE3MatrixSeqItem()
    : seq_(std::make_shared<MultiSE3MatrixSeq>())
{

}


MultiSE3MatrixSeqItem::MultiSE3MatrixSeqItem(std::shared_ptr<MultiSE3MatrixSeq> seq)
    : seq_(seq)
{

}


MultiSE3MatrixSeqItem::MultiSE3MatrixSeqItem(const MultiSE3MatrixSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiSE3MatrixSeq>(*org.seq_))
{

}


Item* MultiSE3MatrixSeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MultiSE3MatrixSeqItem(*this);
}


std::shared_ptr<AbstractMultiSeq> MultiSE3MatrixSeqItem::abstractMultiSeq()
{
    return seq_;
}
