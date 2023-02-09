#ifndef CNOID_BASE_MULTI_SEQ_ITEM_H
#define CNOID_BASE_MULTI_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/MultiSeq>
#include "exportdecl.h"

namespace cnoid {

template <typename MultiSeqType>
class MultiSeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext) { }
        
    typedef ref_ptr< MultiSeqItem<MultiSeqType> > Ptr;
        
    MultiSeqItem() : seq_(std::make_shared<MultiSeqType>()) { }
        
    MultiSeqItem(std::shared_ptr<MultiSeqType> seq) : seq_(seq) { }

    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override { return seq_; }

    typename std::shared_ptr<MultiSeqType> seq() { return seq_; }

    virtual ~MultiSeqItem() { }
 
protected:
    MultiSeqItem(const MultiSeqItem<MultiSeqType>& org)
        : AbstractMultiSeqItem(org),
          seq_(std::make_shared<MultiSeqType>(*org.seq_)) { }

    /**
       For the copy constructor of inherited classes
    */
    MultiSeqItem(const MultiSeqItem<MultiSeqType>& org, std::shared_ptr<MultiSeqType> newSeq)
        : AbstractMultiSeqItem(org),
          seq_(newSeq) { }

    void resetSeq(std::shared_ptr<MultiSeqType> seq) { seq_ = seq; }

    virtual Item* doCloneItem(CloneMap* /* cloneMap */) const override {
        return new MultiSeqItem<MultiSeqType>(*this);
    }

private:
    std::shared_ptr<MultiSeqType> seq_;
};

}

#endif
