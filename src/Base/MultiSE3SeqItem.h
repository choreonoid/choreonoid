#ifndef CNOID_BASE_MULTI_SE3_SEQ_ITEM_H
#define CNOID_BASE_MULTI_SE3_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/MultiSE3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiSE3SeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiSE3SeqItem();
    MultiSE3SeqItem(std::shared_ptr<MultiSE3Seq> seq);

    std::shared_ptr<MultiSE3Seq> seq() { return seq_; }
    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override;

protected:
    MultiSE3SeqItem(const MultiSE3SeqItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<MultiSE3Seq> seq_;
};

typedef ref_ptr<MultiSE3SeqItem> MultiSE3SeqItemPtr;

}

#endif
