/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3MatrixSeq.h"
#include "PlainSeqFileLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "EigenUtil.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


MultiSE3MatrixSeq::MultiSE3MatrixSeq()
    : MultiSE3MatrixSeq::BaseSeqType("MultiSE3MatrixSeq")
{

}


MultiSE3MatrixSeq::MultiSE3MatrixSeq(int numFrames, int numParts)
    : MultiSE3MatrixSeq::BaseSeqType("MultiSE3MatrixSeq", numFrames, numParts)
{

}


MultiSE3MatrixSeq::MultiSE3MatrixSeq(const MultiSE3MatrixSeq& org)
    : MultiSE3MatrixSeq::BaseSeqType(org)
{

}


MultiSE3MatrixSeq::~MultiSE3MatrixSeq()
{

}


std::shared_ptr<AbstractSeq> MultiSE3MatrixSeq::cloneSeq() const
{
    return std::make_shared<MultiSE3MatrixSeq>(*this);
}


Affine3 MultiSE3MatrixSeq::defaultValue() const
{
    return Affine3::Identity();
}
