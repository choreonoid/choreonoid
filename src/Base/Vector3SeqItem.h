/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_VECTOR3_SEQ_ITEM_H_INCLUDED
#define CNOID_GUIBASE_VECTOR3_SEQ_ITEM_H_INCLUDED

#include "AbstractSeqItem.h"
#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Vector3SeqItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    Vector3SeqItem();
    Vector3SeqItem(Vector3SeqPtr seq);

    virtual AbstractSeqPtr abstractSeq();
        
    inline const Vector3SeqPtr& seq() { return seq_; }

    bool loadPlainFormat(const std::string& filename);
    bool saveAsPlainFormat(const std::string& filename);

protected:
    Vector3SeqItem(const Vector3SeqItem& org);

    /**
       This is for the copy constructor of an inherited class
    */
    Vector3SeqItem(const Vector3SeqItem& org, Vector3SeqPtr cloneSeq);
        
    virtual ~Vector3SeqItem();

    virtual ItemPtr doDuplicate() const;
            
    Vector3SeqPtr seq_;
};

typedef ref_ptr<Vector3SeqItem> Vector3SeqItemPtr;
}

#endif
