/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SE3_MATRIX_SEQ_H
#define CNOID_UTIL_MULTI_SE3_MATRIX_SEQ_H

#include "MultiSeq.h"
#include "EigenTypes.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLWriter;

class CNOID_EXPORT MultiSE3MatrixSeq : public MultiSeq<Affine3, Eigen::aligned_allocator<Affine3>>
{
    typedef MultiSeq<Affine3, Eigen::aligned_allocator<Affine3>> BaseSeqType;

public:
    typedef std::shared_ptr<MultiSE3MatrixSeq> Ptr;
    typedef std::shared_ptr<const MultiSE3MatrixSeq> ConstPtr;

    MultiSE3MatrixSeq();
    MultiSE3MatrixSeq(int numFrames, int numParts = 1);
    MultiSE3MatrixSeq(const MultiSE3MatrixSeq& org);
    virtual ~MultiSE3MatrixSeq();

    using BaseSeqType::operator=;

    virtual AbstractSeqPtr cloneSeq() const override;
        
    bool loadPlainFormat(const std::string& filename, std::ostream& os = nullout());
    bool saveTopPartAsPlainFormat(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual Affine3 defaultValue() const { return Affine3::Identity(); }

    virtual bool doReadSeq(const Mapping& archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer) override;
};

typedef MultiSE3MatrixSeq::Ptr MultiSE3MatrixSeqPtr;
typedef MultiSE3MatrixSeq::ConstPtr ConstMultiSE3MatrixSeqPtr;

}

#endif
