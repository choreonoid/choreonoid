/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_AFFINE3_SEQ_H
#define CNOID_UTIL_MULTI_AFFINE3_SEQ_H

#include "MultiSeq.h"
#include "EigenTypes.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLWriter;

class CNOID_EXPORT MultiAffine3Seq : public MultiSeq<Affine3, Eigen::aligned_allocator<Affine3>>
{
    typedef MultiSeq<Affine3, Eigen::aligned_allocator<Affine3>> BaseSeqType;

public:
    typedef std::shared_ptr<MultiAffine3Seq> Ptr;
    typedef std::shared_ptr<const MultiAffine3Seq> ConstPtr;

    MultiAffine3Seq();
    MultiAffine3Seq(int numFrames, int numParts = 1);
    MultiAffine3Seq(const MultiAffine3Seq& org);
    virtual ~MultiAffine3Seq();

    using BaseSeqType::operator=;

    virtual AbstractSeqPtr cloneSeq() const override;
        
    bool loadPlainFormat(const std::string& filename, std::ostream& os = nullout());
    bool saveTopPartAsPlainFormat(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual Affine3 defaultValue() const { return Affine3::Identity(); }

    virtual bool doReadSeq(const Mapping& archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer) override;
};

typedef MultiAffine3Seq::Ptr MultiAffine3SeqPtr;
typedef MultiAffine3Seq::ConstPtr ConstMultiAffine3SeqPtr;

}

#endif
