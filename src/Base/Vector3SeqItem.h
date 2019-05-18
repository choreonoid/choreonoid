/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VECTOR3_SEQ_ITEM_H
#define CNOID_BASE_VECTOR3_SEQ_ITEM_H

#include "AbstractSeqItem.h"
#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Vector3SeqItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    Vector3SeqItem();
    Vector3SeqItem(const Vector3SeqItem& org);
    Vector3SeqItem(std::shared_ptr<Vector3Seq> seq);

    virtual std::shared_ptr<AbstractSeq> abstractSeq() override;
        
    std::shared_ptr<Vector3Seq> seq() { return seq_; }

    bool loadPlainFormat(const std::string& filename);
    bool saveAsPlainFormat(const std::string& filename);

protected:
    /**
       This is for the copy constructor of an inherited class
    */
    Vector3SeqItem(const Vector3SeqItem& org, std::shared_ptr<Vector3Seq> cloneSeq);
        
    virtual ~Vector3SeqItem();

    virtual Item* doDuplicate() const override;
            
    std::shared_ptr<Vector3Seq> seq_;
};

typedef ref_ptr<Vector3SeqItem> Vector3SeqItemPtr;
}

#endif
