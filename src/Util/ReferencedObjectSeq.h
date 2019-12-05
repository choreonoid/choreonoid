/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_REFERENCED_OBJECT_SEQ_H
#define CNOID_UTIL_REFERENCED_OBJECT_SEQ_H

#include "Seq.h"
#include "Referenced.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ReferencedObjectSeq : public Seq<ReferencedPtr>
{
public:
    typedef Seq<ReferencedPtr> BaseSeqType;
    typedef typename BaseSeqType::value_type value_type;
            
    ReferencedObjectSeq(int nFrames = 0);
    ReferencedObjectSeq(const ReferencedObjectSeq& org);
    virtual ~ReferencedObjectSeq();

    using BaseSeqType::operator=;
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;
};

}

#endif
