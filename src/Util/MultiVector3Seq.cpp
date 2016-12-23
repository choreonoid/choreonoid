/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiVector3Seq.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


MultiVector3Seq::MultiVector3Seq()
    : BaseSeqType("MultiVector3Seq")
{

}


MultiVector3Seq::MultiVector3Seq(int numFrames, int numParts)
    : BaseSeqType("MultiVector3Seq", numFrames, numParts)
{

}
        

MultiVector3Seq::MultiVector3Seq(const MultiVector3Seq& org)
    : BaseSeqType(org)
{

}


AbstractSeqPtr MultiVector3Seq::cloneSeq() const
{
    return std::make_shared<MultiVector3Seq>(*this);
}


void MultiVector3Seq::copySeqProperties(const MultiVector3Seq& source)
{
    BaseSeqType::copySeqProperties(source);
}
    

MultiVector3Seq::~MultiVector3Seq()
{

}
        

bool MultiVector3Seq::doWriteSeq(YAMLWriter& writer)
{
    if(BaseSeqType::doWriteSeq(writer)){
        writer.putKey("frames");
        writer.startListing();
        const int m = numParts();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            Frame f = frame(i);
            writer.startFlowStyleListing();
            for(int j=0; j < m; ++j){
                writer.startFlowStyleListing();
                const Vector3& p = f[j];
                writer.putScalar(p.x());
                writer.putScalar(p.y());
                writer.putScalar(p.z());
                writer.endListing();
            }
            writer.endListing();
        }
        writer.endListing();
        return true;
    }
    return false;
}


bool MultiVector3Seq::doReadSeq(const Mapping& archive)
{
    if(BaseSeqType::doReadSeq(archive)){
        const string& type = archive["type"].toString();
        if(type == seqType()){
            const Listing& values = *archive.findListing("frames");
            if(values.isValid()){
                const int nParts = archive["numParts"].toInt();
                const int nFrames = values.size();
                setDimension(nFrames, nParts);
                
                for(int i=0; i < nFrames; ++i){
                    const Listing& frameNode = *values[i].toListing();
                    Frame f = frame(i);
                    const int n = std::min(frameNode.size(), nParts);
                    for(int j=0; j < n; ++j){
                        const Listing& node = *frameNode[j].toListing();
                        if(node.size() == 3){
                            f[j] << node[0].toDouble(), node[1].toDouble(), node[2].toDouble();
                        } else {
                            node.throwException("Element is not a three dimension vector");
                        }
                    }
                }
            }
        }
        return true;
    }
    return false;
}
