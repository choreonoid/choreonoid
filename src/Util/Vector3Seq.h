/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VECTOR3_SEQ_H
#define CNOID_UTIL_VECTOR3_SEQ_H

#include "Seq.h"
#include "EigenUtil.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Vector3Seq : public Seq<Vector3>
{
public:
    typedef Seq<Vector3> BaseSeqType;
            
    Vector3Seq(int nFrames = 0);
    Vector3Seq(const Vector3Seq& org);
    virtual ~Vector3Seq();

    using BaseSeqType::operator=;
    virtual AbstractSeqPtr cloneSeq() const;
        
    virtual bool loadPlainFormat(const std::string& filename);
    virtual bool saveAsPlainFormat(const std::string& filename);

protected:
    virtual Vector3 defaultValue() const { return Vector3::Zero(); }

    virtual bool doWriteSeq(YAMLWriter& writer);
    virtual bool doReadSeq(const Mapping& archive);
};

typedef std::shared_ptr<Vector3Seq> Vector3SeqPtr;
}

#endif
