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

class CNOID_EXPORT MultiSE3MatrixSeq : public MultiSeq<Isometry3, Eigen::aligned_allocator<Isometry3>>
{
    typedef MultiSeq<Isometry3, Eigen::aligned_allocator<Isometry3>> BaseSeqType;

public:
    MultiSE3MatrixSeq();
    MultiSE3MatrixSeq(int numFrames, int numParts = 1);
    MultiSE3MatrixSeq(const MultiSE3MatrixSeq& org);
    virtual ~MultiSE3MatrixSeq();

    using BaseSeqType::operator=;

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;
        
protected:
    virtual Isometry3 defaultValue() const override;
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<MultiSE3MatrixSeq> MultiSE3MatrixSeqPtr;
#endif

}

#endif
