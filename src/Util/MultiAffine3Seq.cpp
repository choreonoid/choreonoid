/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiAffine3Seq.h"
#include "PlainSeqFormatLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "EigenUtil.h"
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;


MultiAffine3Seq::MultiAffine3Seq()
    : MultiAffine3Seq::BaseSeqType("MultiAffine3Seq")
{

}


MultiAffine3Seq::MultiAffine3Seq(int numFrames, int numParts)
    : MultiAffine3Seq::BaseSeqType("MultiAffine3Seq", numFrames, numParts)
{

}


MultiAffine3Seq::MultiAffine3Seq(const MultiAffine3Seq& org)
    : MultiAffine3Seq::BaseSeqType(org)
{

}


MultiAffine3Seq::~MultiAffine3Seq()
{

}


AbstractSeqPtr MultiAffine3Seq::cloneSeq() const
{
    return std::make_shared<MultiAffine3Seq>(*this);
}


bool MultiAffine3Seq::loadPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        addSeqMessage(loader.errorMessage());
        return false;
    }

    if(loader.numParts() < 12){
        addSeqMessage(filename +
                      "does not have 12-columns "
                      "(3 for position vectors, 9 for attitde matrices)");
        return false;
    }
  
    setDimension(loader.numFrames(), 1);
    setTimeStep(loader.timeStep());

    int i = 0;
    Part base = part(0);
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        base[i].translation() << data[1], data[2], data[3];
        base[i].linear() <<
            data[ 4], data[ 5], data[ 6],
            data[ 7], data[ 8], data[ 9],
            data[10], data[11], data[12];
        ++i;
    }

    return true;
}


bool MultiAffine3Seq::saveTopPartAsPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    boost::format f("%1$.4f");
    const int nFrames = numFrames();

    if(nFrames > 0 && numParts() > 0){

        ofstream os(filename.c_str());
        if(!os){
            addSeqMessage(filename + " cannot be opened.");
            return false;
        }

        const double r = frameRate();

        Part base = part(0);
        for(int i=0; i < nFrames; ++i){
            os << (f % (i / r));
            const Affine3& T = base[i];
            for(int j=0; j < 3; ++j){
                os << " " << T.translation()[j];
            }
            for(int j=0; j < 3; ++j){
                for(int k=0; k < 3; ++k){
                    double m = T.linear()(j, k);
                    if(fabs(m) < 1.0e-14){
                        m = 0.0;
                    }
                    os << " " << m;
                }
            }
            os << " 0 0 0 0 0 0"; // velocity elements (dv, omega)
            os << "\n";
        }

        return true;
    }

    return false;
}


static inline void writeAffine3(YAMLWriter& writer, const Affine3& value)
{
    writer.startFlowStyleListing();

    writer.putScalar(value.translation().x());
    writer.putScalar(value.translation().y());
    writer.putScalar(value.translation().z());

    Vector3 rpy(rpyFromRot(value.linear()));
    writer.putScalar(rpy[0]);
    writer.putScalar(rpy[1]);
    writer.putScalar(rpy[2]);

    writer.endListing();
}
    

bool MultiAffine3Seq::doWriteSeq(YAMLWriter& writer)
{
    if(BaseSeqType::doWriteSeq(writer)){

        writer.putKeyValue("format", "XYZRPY");
    
        writer.putKey("frames");
        writer.startListing();
        const int m = numParts();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            Frame f = frame(i);
            writer.startFlowStyleListing();
            for(int j=0; j < m; ++j){
                writeAffine3(writer, f[j]);
            }
            writer.endListing();
        }
        writer.endListing();

        return true;
    }
    
    return false;
}


static void readAffine3(const Listing& node, Affine3& out_value)
{
    if(node.size() == 6){

        Affine3::TranslationPart t = out_value.translation();
        t[0] = node[0].toDouble();
        t[1] = node[1].toDouble();
        t[2] = node[2].toDouble();

        const double r = node[3].toDouble();
        const double p = node[4].toDouble();
        const double y = node[5].toDouble();
        
        out_value.linear() = rotFromRpy(r, p, y);
    }
}


bool MultiAffine3Seq::doReadSeq(const Mapping& archive)
{
    if(BaseSeqType::doReadSeq(archive)){

        const string& type = archive["type"].toString();
        if((type == seqType() || type == "MultiSe3Seq") && (archive["format"].toString() == "XYZRPY")){

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
                        readAffine3(*frameNode[j].toListing(), f[j]);
                    }
                }
                return true;
            }
        }
    }
    return false;
}
