/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_VECTOR3_SEQ_H
#define CNOID_UTIL_MULTI_VECTOR3_SEQ_H

#include "MultiSeq.h"
#include "EigenTypes.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLWriter;

class CNOID_EXPORT MultiVector3Seq : public MultiSeq<Vector3, Eigen::aligned_allocator<Vector3>>
{
    typedef MultiSeq<Vector3, Eigen::aligned_allocator<Vector3>> BaseSeqType;

public:
    MultiVector3Seq();
    MultiVector3Seq(int numFrames, int numParts = 1);
    MultiVector3Seq(const MultiVector3Seq& org);
    virtual ~MultiVector3Seq();
    using BaseSeqType::operator=;
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;
    void copySeqProperties(const MultiVector3Seq& source);

protected:
    virtual Vector3 defaultValue() const override;
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<MultiVector3Seq> MultiVector3SeqPtr;
#endif

}

#endif
