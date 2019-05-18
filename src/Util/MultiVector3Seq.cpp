/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiVector3Seq.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "GeneralSeqReader.h"
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


std::shared_ptr<AbstractSeq> MultiVector3Seq::cloneSeq() const
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
        

Vector3 MultiVector3Seq::defaultValue() const
{
    return Vector3::Zero();
}


bool MultiVector3Seq::doReadSeq(const Mapping* archive, std::ostream& os)
{
    GeneralSeqReader reader(os);

    return reader.read<MultiVector3Seq>(
        archive, this,
        [](const ValueNode& node, Vector3& value){
            const Listing& v = *node.toListing();
            if(v.size() != 3){
                v.throwException(_("The number of elements specified as a 3D vector is invalid."));
            }
            value << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
        });
}


bool MultiVector3Seq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    return BaseSeqType::doWriteSeq(
        writer,
        [&](){
            if(additionalPartCallback) additionalPartCallback();

            writer.putKey("frames");
            writer.startListing();
            const int n = numFrames();
            const int m = numParts();
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
        });
}
