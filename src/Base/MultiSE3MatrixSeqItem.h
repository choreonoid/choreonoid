#ifndef CNOID_BASE_MULTI_SE3_MATRIX_SEQ_ITEM_H
#define CNOID_BASE_MULTI_SE3_MATRIX_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/MultiSE3MatrixSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiSE3MatrixSeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiSE3MatrixSeqItem();
    MultiSE3MatrixSeqItem(std::shared_ptr<MultiSE3MatrixSeq> seq);

    std::shared_ptr<MultiSE3MatrixSeq> seq() { return seq_; }
    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override;

protected:
    MultiSE3MatrixSeqItem(const MultiSE3MatrixSeqItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<MultiSE3MatrixSeq> seq_;
};

typedef ref_ptr<MultiSE3MatrixSeqItem> MultiSE3MatrixSeqItemPtr;

}

#endif
