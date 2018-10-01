/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SE3_SEQ_H
#define CNOID_UTIL_MULTI_SE3_SEQ_H

#include "MultiSeq.h"
#include "EigenTypes.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class Listing;
class YAMLWriter;
        
class CNOID_EXPORT MultiSE3Seq : public MultiSeq<SE3, Eigen::aligned_allocator<SE3>>
{
    typedef MultiSeq<SE3, Eigen::aligned_allocator<SE3>> BaseSeqType;

public:
    typedef std::shared_ptr<MultiSE3Seq> Ptr;
    typedef std::shared_ptr<const MultiSE3Seq> ConstPtr;

    MultiSE3Seq();
    MultiSE3Seq(int numFrames, int numParts = 1);
    MultiSE3Seq(const MultiSE3Seq& org);
    virtual ~MultiSE3Seq();

    using BaseSeqType::operator=;

    virtual AbstractSeqPtr cloneSeq() const override;
        
    bool loadPlainMatrixFormat(const std::string& filename, std::ostream& os = nullout());
    bool loadPlainRpyFormat(const std::string& filename, std::ostream& os = nullout());
    bool saveTopPartAsPlainMatrixFormat(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual SE3 defaultValue() const override;
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;
};

typedef MultiSE3Seq::Ptr MultiSE3SeqPtr;
typedef MultiSE3Seq::ConstPtr ConstMultiSE3SeqPtr;

}

#endif
