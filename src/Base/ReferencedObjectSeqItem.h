/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_REFERENCED_OBJECT_SEQ_ITEM_H
#define CNOID_BASE_REFERENCED_OBJECT_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/ReferencedObjectSeq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ReferencedObjectSeqItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    ReferencedObjectSeqItem();
    ReferencedObjectSeqItem(const ReferencedObjectSeqItem& org);
    ReferencedObjectSeqItem(std::shared_ptr<ReferencedObjectSeq> seq);

    virtual std::shared_ptr<AbstractSeq> abstractSeq() override;
        
    std::shared_ptr<ReferencedObjectSeq> seq() { return seq_; }

    void resetSeq();

protected:
    /**
       This is for the copy constructor of an inherited class
    */
    ReferencedObjectSeqItem(const ReferencedObjectSeqItem& org, std::shared_ptr<ReferencedObjectSeq> cloneSeq);
        
    virtual ~ReferencedObjectSeqItem();

    virtual Item* doDuplicate() const override;
            
    std::shared_ptr<ReferencedObjectSeq> seq_;
};

typedef ref_ptr<ReferencedObjectSeqItem> ReferencedObjectSeqItemPtr;

}

#endif
