#ifndef CNOID_BASE_MULTI_VALUE_SEQ_ITEM_H
#define CNOID_BASE_MULTI_VALUE_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/MultiValueSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiValueSeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiValueSeqItem();
    MultiValueSeqItem(std::shared_ptr<MultiValueSeq> seq);

    std::shared_ptr<MultiValueSeq> seq() { return seq_; }
    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override;

    void resetSeq(std::shared_ptr<MultiValueSeq> seq);

protected:
    MultiValueSeqItem(const MultiValueSeqItem& org);
    MultiValueSeqItem(const MultiValueSeqItem& org, std::shared_ptr<MultiValueSeq> seq);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<MultiValueSeq> seq_;
};

typedef ref_ptr<MultiValueSeqItem> MultiValueSeqItemPtr;

}

#endif
