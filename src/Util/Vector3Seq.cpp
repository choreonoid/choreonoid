/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Vector3Seq.h"
#include "PlainSeqFormatLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;


Vector3Seq::Vector3Seq(int nFrames)
    : BaseSeqType("Vector3Seq", nFrames)
{

}


Vector3Seq::Vector3Seq(const Vector3Seq& org)
    : BaseSeqType(org)
{

}


AbstractSeqPtr Vector3Seq::cloneSeq() const
{
    return std::make_shared<Vector3Seq>(*this);
}


Vector3Seq::~Vector3Seq()
{

}


bool Vector3Seq::loadPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        addSeqMessage(loader.errorMessage());
        return false;
    }

    if(loader.numParts() < 3){
        addSeqMessage(filename + "does not have 3 columns for 3d vector elements");
        return false;
    }
  
    setNumFrames(loader.numFrames());
    setFrameRate(1.0 / loader.timeStep());

    int frame = 0;
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        (*this)[frame++] << data[1], data[2], data[3];
    }

    return true;
}


bool Vector3Seq::saveAsPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    ofstream os(filename.c_str());
    os.setf(ios::fixed);

    if(!os){
        addSeqMessage(filename + " cannot be opened.");
        return false;
    }

    static boost::format f("%1$.4f %2$.6f %3$.6f %4$.6f\n");

    const int n = numFrames();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        const Vector3& v = (*this)[i];
        os << (f % (i / r) % v.x() % v.y() % v.z());
    }
    
    return true;
}


bool Vector3Seq::doWriteSeq(YAMLWriter& writer)
{
    if(BaseSeqType::doWriteSeq(writer)){
        writer.putKey("frames");
        writer.startListing();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            writer.startFlowStyleListing();
            const Vector3& v = (*this)[i];
            for(int j=0; j < 3; ++j){
                writer.putScalar(v[j]);
            }
            writer.endListing();
        }
        writer.endListing();
        return true;
    }
    return false;
}


bool Vector3Seq::doReadSeq(const Mapping& archive)
{
    if(BaseSeqType::doReadSeq(archive)){
        if(archive["type"].toString() == seqType()){
            const Listing& frames = *archive.findListing("frames");
            if(!frames.isValid()){
                addSeqMessage("Valid \"frames\" field of Vector3Seq is not found.");
            } else {
                const int n = frames.size();
                setNumFrames(n);
                for(int i=0; i < n; ++i){
                    const Listing& frame = *frames[i].toListing();
                    Vector3& v = (*this)[i];
                    for(int j=0; j < 3; ++j){
                        v[j] = frame[j].toDouble();
                    }
                }
                return true;
            }
        }
    }
    return false;
}
