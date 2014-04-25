/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_VALUE_SEQ_H_INCLUDED
#define CNOID_UTIL_MULTI_VALUE_SEQ_H_INCLUDED

#include "MultiSeq.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiValueSeq : public MultiSeq<double>
{
    typedef MultiSeq<double> BaseSeqType;
            
public:
    typedef boost::shared_ptr<MultiValueSeq> Ptr;

    MultiValueSeq();
    MultiValueSeq(int numFrames, int numParts = 1);
    MultiValueSeq(const MultiValueSeq& org);
    virtual ~MultiValueSeq();

    virtual AbstractSeqPtr cloneSeq() const;
        
    virtual bool loadPlainFormat(const std::string& filename);
    virtual bool saveAsPlainFormat(const std::string& filename);

protected:
    virtual bool doWriteSeq(YAMLWriter& writer);
    virtual bool doReadSeq(const Mapping& archive);
};

typedef MultiValueSeq::Ptr MultiValueSeqPtr;        
}

#endif
