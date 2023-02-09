#ifndef CNOID_BASE_MULTI_VECTOR3_SEQ_ITEM_H
#define CNOID_BASE_MULTI_VECTOR3_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/MultiVector3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiVector3SeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    MultiVector3SeqItem();
    MultiVector3SeqItem(std::shared_ptr<MultiVector3Seq> seq);

    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override;
        
    std::shared_ptr<MultiVector3Seq> seq() { return seq_; }

    bool loadPlainFormat(const std::string& filename);
    bool saveAsPlainFormat(const std::string& filename);

protected:
    MultiVector3SeqItem(const MultiVector3SeqItem& org);

    /**
       This is for the copy constructor of an inherited class
    */
    MultiVector3SeqItem(const MultiVector3SeqItem& org, std::shared_ptr<MultiVector3Seq> cloneSeq);
        
    virtual ~MultiVector3SeqItem();

    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
            
    std::shared_ptr<MultiVector3Seq> seq_;
};

typedef ref_ptr<MultiVector3SeqItem> MultiVector3SeqItemPtr;
}

#endif
