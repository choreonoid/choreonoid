/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_VALUE_SEQ_H
#define CNOID_UTIL_MULTI_VALUE_SEQ_H

#include "MultiSeq.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiValueSeq : public MultiSeq<double>
{
    typedef MultiSeq<double> BaseSeqType;
            
public:
    MultiValueSeq();
    MultiValueSeq(int numFrames, int numParts = 1);
    MultiValueSeq(const MultiValueSeq& org);
    virtual ~MultiValueSeq();

    using BaseSeqType::operator=;
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;
        
    bool loadPlainFormat(const std::string& filename, std::ostream& os = nullout());
    bool saveAsPlainFormat(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> writeAdditionalPart) override;
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<MultiValueSeq> MultiValueSeqPtr;
#endif

}

#endif
