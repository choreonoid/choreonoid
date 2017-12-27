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
    Vector3SeqItem(Vector3SeqPtr seq);

    virtual AbstractSeqPtr abstractSeq() override;
        
    Vector3SeqPtr seq() { return seq_; }

    bool loadPlainFormat(const std::string& filename);
    bool saveAsPlainFormat(const std::string& filename);

protected:
    /**
       This is for the copy constructor of an inherited class
    */
    Vector3SeqItem(const Vector3SeqItem& org, Vector3SeqPtr cloneSeq);
        
    virtual ~Vector3SeqItem();

    virtual Item* doDuplicate() const override;
            
    Vector3SeqPtr seq_;
};

typedef ref_ptr<Vector3SeqItem> Vector3SeqItemPtr;
}

#endif
