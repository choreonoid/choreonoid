/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeq.h"
#include "PlainSeqFormatLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"

using namespace std;
using namespace cnoid;


MultiValueSeq::MultiValueSeq()
    : BaseSeqType("MultiValueSeq")
{

}


MultiValueSeq::MultiValueSeq(int numFrames, int numParts)
    : BaseSeqType("MultiValueSeq", numFrames, numParts)
{

}


MultiValueSeq::MultiValueSeq(const MultiValueSeq& org)
    : BaseSeqType(org)
{

}


AbstractSeqPtr MultiValueSeq::cloneSeq() const
{
    return std::make_shared<MultiValueSeq>(*this);
}


MultiValueSeq::~MultiValueSeq()
{

}


bool MultiValueSeq::loadPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        addSeqMessage(loader.errorMessage());
        return false;
    }

    setDimension(loader.numFrames(), loader.numParts());
    setFrameRate(1.0 / loader.timeStep());

    int i = 0;
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        copy((it->begin() + 1), it->end(), frame(i++).begin());
    }

    return true;
}


bool MultiValueSeq::saveAsPlainFormat(const std::string& filename)
{
    clearSeqMessage();
    ofstream os(filename.c_str());
    os.setf(ios::fixed);

    if(!os){
        addSeqMessage(filename + " cannot be opened.");
        return false;
    }

    const int n = numFrames();
    const int m = numParts();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        os << (i / r);
        Frame v = frame(i);
        for(int j=0; j < m; ++j){
            os << " " << v[j];
        }
        os << "\n";
    }
    
    return true;
}


bool MultiValueSeq::doWriteSeq(YAMLWriter& writer)
{
    if(BaseSeqType::doWriteSeq(writer)){
        writer.putKey("frames");
        writer.startListing();
        const int n = numFrames();
        const int m = numParts();
        for(int i=0; i < n; ++i){
            writer.startFlowStyleListing();
            Frame v = frame(i);
            for(int j=0; j < m; ++j){
                writer.putScalar(v[j]);
            }
            writer.endListing();
        }
        writer.endListing();
        return true;
    }
    return false;
}


bool MultiValueSeq::doReadSeq(const Mapping& archive)
{
    if(BaseSeqType::doReadSeq(archive) && (archive["type"].toString() == seqType())){

        const int nParts = archive["numParts"].toInt();
        int nFrames;
        if(archive.read("numFrames", nFrames)){
            if(nFrames == 0){
                setDimension(0, nParts);
                return true;
            }
        }
        const Listing& values = *archive.findListing("frames");
        if(!values.isValid()){
            addSeqMessage("Actual frame data is missing.");
        } else {
            const int nFrames = values.size();
            setDimension(nFrames, nParts);
            for(int i=0; i < nFrames; ++i){
                const Listing& frameNode = *values[i].toListing();
                const int n = std::min(frameNode.size(), nParts);
                Frame v = frame(i);
                for(int j=0; j < n; ++j){
                    v[j] = frameNode[j].toDouble();
                }
            }
            return true;
        }
        
    }
    return false;
}
