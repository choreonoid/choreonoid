/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VECTOR3_SEQ_H
#define CNOID_UTIL_VECTOR3_SEQ_H

#include "Seq.h"
#include "EigenUtil.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Vector3Seq : public Seq<Vector3>
{
public:
    typedef Seq<Vector3> BaseSeqType;
    typedef typename BaseSeqType::value_type value_type;
            
    Vector3Seq(int nFrames = 0);
    Vector3Seq(const Vector3Seq& org);
    virtual ~Vector3Seq();

    using BaseSeqType::operator=;
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const;
        
    bool loadPlainFormat(const std::string& filename, std::ostream& os = nullout());
    bool saveAsPlainFormat(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual Vector3 defaultValue() const override;
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> writeAdditionalPart) override;
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<Vector3Seq> Vector3SeqPtr;
#endif

}

#endif
