/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Vector3SeqItem.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

void Vector3SeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<Vector3SeqItem>(N_("Vector3SeqItem"));
}


Vector3SeqItem::Vector3SeqItem()
    : seq_(std::make_shared<Vector3Seq>())
{

}


Vector3SeqItem::Vector3SeqItem(std::shared_ptr<Vector3Seq> seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


Vector3SeqItem::Vector3SeqItem(const Vector3SeqItem& org)
    : AbstractSeqItem(org),
      seq_(std::make_shared<Vector3Seq>(*org.seq_))
{

}


Vector3SeqItem::Vector3SeqItem(const Vector3SeqItem& org, std::shared_ptr<Vector3Seq> cloneSeq)
    : AbstractSeqItem(org),
      seq_(cloneSeq)
{

}



Vector3SeqItem::~Vector3SeqItem()
{

}


std::shared_ptr<AbstractSeq> Vector3SeqItem::abstractSeq()
{
    return seq_;
}


bool Vector3SeqItem::loadPlainFormat(const std::string& filename)
{
    bool loaded = seq_->loadPlainFormat(filename);
    notifyUpdate();
    return loaded;
}


bool Vector3SeqItem::saveAsPlainFormat(const std::string& filename)
{
    return seq_->saveAsPlainFormat(filename);
}


Item* Vector3SeqItem::doDuplicate() const
{
    return new Vector3SeqItem(*this);
}
