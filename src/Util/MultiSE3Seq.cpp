/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3Seq.h"
#include "PlainSeqFormatLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "EigenUtil.h"
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;


MultiSE3Seq::MultiSE3Seq()
    : MultiSE3Seq::BaseSeqType("MultiSE3Seq")
{

}


MultiSE3Seq::MultiSE3Seq(int numFrames, int numParts)
    : MultiSE3Seq::BaseSeqType("MultiSE3Seq", numFrames, numParts)
{

}


MultiSE3Seq::MultiSE3Seq(const MultiSE3Seq& org)
    : MultiSE3Seq::BaseSeqType(org)
{

}


AbstractSeqPtr MultiSE3Seq::cloneSeq() const
{
    return std::make_shared<MultiSE3Seq>(*this);
}


MultiSE3Seq::~MultiSE3Seq()
{

}


bool MultiSE3Seq::loadPlainMatrixFormat(const std::string& filename)
{
    clearSeqMessage();
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        addSeqMessage(loader.errorMessage());
        return false;
    }

    int n = loader.numParts();
    if(n < 12 || (n % 12) != 0){
        addSeqMessage(filename +
                      "does not have elements in multiple of twelve "
                      "(each 3 for position vectors, 9 for attitde matrices)");
        return false;
    }
    int m = n / 12;
  
    setDimension(loader.numFrames(), m);
    setTimeStep(loader.timeStep());

    int f = 0;
    Part base = part(0);
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        int i = 0;
        Frame frame = MultiSE3Seq::frame(f++);
        for(int j=0; j < m; ++j){
            SE3& x = frame[j];
            x.translation() << data[i], data[i+1], data[i+2];
            i += 3;
            Matrix3 R;
            R <<
                data[i],   data[i+1], data[i+2],
                data[i+3], data[i+4], data[i+5],
                data[i+6], data[i+7], data[i+8];
            i += 9;
            x.rotation() = R;
        }
    }

    return true;
}


bool MultiSE3Seq::loadPlainRpyFormat(const std::string& filename)
{
    clearSeqMessage();
    PlainSeqFileLoader loader;

    if(!loader.load(filename)){
        addSeqMessage(loader.errorMessage());
        return false;
    }

    int n = loader.numParts();
    if(n != 3){
        addSeqMessage(filename +
                      "does not have a multiple of 3 elements (R,P,Y)");
        return false;
    }

    setDimension(loader.numFrames(), 1);
    setTimeStep(loader.timeStep());

    int f = 0;
    Part base = part(0);
    for(PlainSeqFileLoader::iterator it = loader.begin(); it != loader.end(); ++it){
        vector<double>& data = *it;
        Frame frame = MultiSE3Seq::frame(f++);
        SE3& x = frame[0];
        x.translation() << 0, 0, 0;
        double r, p, y;
        //First element is time
        r = data[1];
        p = data[2];
        y = data[3];
        Matrix3 R = rotFromRpy(r, p, y);
        x.rotation() = R;
    }

    return true;
}



bool MultiSE3Seq::saveTopPartAsPlainMatrixFormat(const std::string& filename)
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
            const SE3& x = base[i];
            for(int j=0; j < 3; ++j){
                os << " " << x.translation()[j];
            }
            Matrix3 R(x.rotation());
            for(int j=0; j < 3; ++j){
                for(int k=0; k < 3; ++k){
                    double m = R(j, k);
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


static inline void writeSE3(YAMLWriter& writer, const SE3& value)
{
    writer.startFlowStyleListing();

    const Vector3& p = value.translation();
    writer.putScalar(p.x());
    writer.putScalar(p.y());
    writer.putScalar(p.z());

    const Quat& q = value.rotation();
    writer.putScalar(q.w());
    writer.putScalar(q.x());
    writer.putScalar(q.y());
    writer.putScalar(q.z());

    writer.endListing();
}
    

bool MultiSE3Seq::doWriteSeq(YAMLWriter& writer)
{
    if(BaseSeqType::doWriteSeq(writer)){

        writer.putKeyValue("format", "XYZQWQXQYQZ");
    
        writer.putKey("frames");
        writer.startListing();
        const int m = numParts();
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            Frame f = frame(i);
            writer.startFlowStyleListing();
            for(int j=0; j < m; ++j){
                writeSE3(writer, f[j]);
            }
            writer.endListing();
        }
        writer.endListing();
        
        return true;
    }
    return false;
}


bool MultiSE3Seq::doReadSeq(const Mapping& archive)
{
    if(BaseSeqType::doReadSeq(archive)){
        const string& type = archive["type"].toString();
        if(type == seqType() || type == "MultiSe3Seq" || type == "MultiAffine3Seq"){
            string formatString = archive["format"].toString();
            const Listing& values = *archive.findListing("frames");
            if(values.isValid()){
                const int nParts = archive["numParts"].toInt();
                const int nFrames = values.size();
                setDimension(nFrames, nParts);
                if(formatString == "XYZQWQXQYQZ"){
                    readPosQuatSeq(nParts, nFrames, values, true);
                } else if(formatString == "XYZQXQYQZQW"){
                    readPosQuatSeq(nParts, nFrames, values, false);
                } else if(formatString == "XYZRPY"){
                    readPosRpySeq(nParts, nFrames, values);
                } else {
                    addSeqMessage(
                        str(format(("Unknown format \"%1%\" cannot be loaded into MultiSE3Seq."))
                            % formatString));
                    return false;
                }
            }
        }
        return true;
    }
    return false;
}


void MultiSE3Seq::readPosQuatSeq(int nParts, int nFrames, const Listing& values, bool isWfirst)
{
    for(int i=0; i < nFrames; ++i){
        const Listing& frameNode = *values[i].toListing();
        Frame f = frame(i);
        const int n = std::min(frameNode.size(), nParts);
        for(int j=0; j < n; ++j){
            const Listing& node = *frameNode[j].toListing();
            SE3& x = f[j];
            if(node.size() == 7){
                x.translation() << node[0].toDouble(), node[1].toDouble(), node[2].toDouble();
                if(isWfirst){
                    x.rotation() = Quat(node[3].toDouble(),
                                        node[4].toDouble(), node[5].toDouble(), node[6].toDouble());
                } else {
                    x.rotation() = Quat(node[6].toDouble(),
                                        node[3].toDouble(), node[4].toDouble(), node[5].toDouble());
                }
            }
        }
    }
}


void MultiSE3Seq::readPosRpySeq(int nParts, int nFrames, const Listing& values)
{
    for(int i=0; i < nFrames; ++i){
        const Listing& frameNode = *values[i].toListing();
        Frame f = frame(i);
        const int n = std::min(frameNode.size(), nParts);
        for(int j=0; j < n; ++j){
            const Listing& node = *frameNode[j].toListing();
            if(node.size() == 6){
                SE3& x = f[j];
                x.translation() << node[0].toDouble(), node[1].toDouble(), node[2].toDouble();
                const double r = node[3].toDouble();
                const double p = node[4].toDouble();
                const double y = node[5].toDouble();
                x.rotation() = rotFromRpy(r, p, y);
            }
        }
    }
}
