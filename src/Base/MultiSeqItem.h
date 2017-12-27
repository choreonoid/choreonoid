/**
   @file
   @author Shin'ichiro Nakaoka
*/

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
        
    MultiSeqItem(typename MultiSeqType::Ptr seq) : seq_(seq) { }

    virtual AbstractMultiSeqPtr abstractMultiSeq() { return seq_; }

    typename MultiSeqType::Ptr seq() { return seq_; }

    MultiSeqItem(const MultiSeqItem<MultiSeqType>& org)
        : AbstractMultiSeqItem(org),
          seq_(std::make_shared<MultiSeqType>(*org.seq_)) { }

    virtual ~MultiSeqItem() { }
 
protected:
    /**
       For the copy constructor of inherited classes
    */
    MultiSeqItem(const MultiSeqItem<MultiSeqType>& org, typename MultiSeqType::Ptr newSeq)
        : AbstractMultiSeqItem(org),
          seq_(newSeq) { }

    void resetSeq(typename MultiSeqType::Ptr seq) { seq_ = seq; }
        
    virtual Item* doDuplicate() const override {
        return new MultiSeqItem<MultiSeqType>(*this);
    }

private:
    typename MultiSeqType::Ptr seq_;
};

}

#endif
