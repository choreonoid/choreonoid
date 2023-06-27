#ifndef CNOID_UTIL_REFERENCED_OBJECT_SEQ_H
#define CNOID_UTIL_REFERENCED_OBJECT_SEQ_H

#include "Seq.h"
#include "Referenced.h"
#include "exportdecl.h"

namespace cnoid {

class CloneMap;

class CNOID_EXPORT ReferencedObjectSeq : public Seq<ReferencedPtr>
{
public:
    typedef Seq<ReferencedPtr> BaseSeqType;
    typedef typename BaseSeqType::value_type value_type;
            
    ReferencedObjectSeq(int nFrames = 0);

    /**
       This cunstructor performs a deep copy of elements when cloneMap is given.
       Otherwise, it performes a shallow copy.
    */
    ReferencedObjectSeq(const ReferencedObjectSeq& org, CloneMap* cloneMap);
    
    virtual ~ReferencedObjectSeq();

    using BaseSeqType::operator=;
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;
};

}

#endif
