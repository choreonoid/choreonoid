/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeq.h"
#include "PlainSeqFileLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;


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


bool MultiValueSeq::doReadSeq(const Mapping& archive, std::ostream& os)
{
    if(!BaseSeqType::doReadSeq(archive, os)){
        return false;
    }

    checkSeqType(archive);
    
    const int nParts = readNumParts(archive);

    const Listing& frames = *archive.findListing("frames");
    if(!frames.isValid()){
        setDimension(0, nParts);
    } else {
        const int nFrames = frames.size();
        setDimension(nFrames, nParts);
        for(int i=0; i < nFrames; ++i){
            const Listing& frameNode = *frames[i].toListing();
            const int n = std::min(frameNode.size(), nParts);
            Frame v = frame(i);
            for(int j=0; j < n; ++j){
                v[j] = frameNode[j].toDouble();
            }
        }
    }
    
    return true;
}


bool MultiValueSeq::doWriteSeq(YAMLWriter& writer)
{
    if(!BaseSeqType::doWriteSeq(writer)){
        return false;
    }

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


bool MultiValueSeq::loadPlainFormat(const std::string& filename, std::ostream& os)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename, os)){
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


bool MultiValueSeq::saveAsPlainFormat(const std::string& filename, std::ostream& os)
{
    ofstream file(filename.c_str());
    file.setf(ios::fixed);

    if(!file){
        os << format(_("\"%1%\" cannot be opened.")) % filename << endl;
        return false;
    }

    const int n = numFrames();
    const int m = numParts();
    const double r = frameRate();

    for(int i=0; i < n; ++i){
        file << (i / r);
        Frame v = frame(i);
        for(int j=0; j < m; ++j){
            file << " " << v[j];
        }
        file << "\n";
    }
    
    return true;
}


bool MultiValueSeq::importTimedFrameSeq(const Mapping& archive, std::ostream& os)
{
    bool result = false;
    try {
        if(!archive.get("hasFrameTime", false)){
            result = doReadSeq(archive, os);
        } else {
            result = importTimedFrameSeqMain(archive, os);
        }
    }
    catch (ValueNode::Exception& ex) {
        os << ex.message();
    }

    return result;
}


bool MultiValueSeq::importTimedFrameSeqMain(const Mapping& archive, std::ostream& os)
{
    checkSeqType(archive);
    readSeqContent(archive);

    if(frameRate() <= 0.0){
        double r;
        if(archive.read("frameRate", r)){
            setFrameRate(r);
        }
    }
    if(frameRate() <= 0.0){
        os << _("Invalid frame rate") << endl;
        return false;
    }
    
    const int nParts = readNumParts(archive);
    setDimension(0, nParts);
    
    const int frameSize = nParts + 1;
    
    const Listing& frames = *archive.findListing("frames");
    if(frames.isValid()){
        const int nFrames = frames.size();
        for(int i=0; i < nFrames; ++i){
            const Listing& values = *frames[i].toListing();
            if(values.size() != frameSize){
                values.throwException("Invalid frame size");
            }
            double time = values[0].toDouble();
            int frameIndex = frameOfTime(time);
            if(frameIndex >= numFrames()){
                setNumFrames(frameIndex + 1, true);
            }
            Frame v = frame(frameIndex);
            for(int j=0; j < nParts; ++j){
                v[j] = values[j+1].toDouble();
            }
        }
    }

    return true;
}
