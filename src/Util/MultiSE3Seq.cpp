/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3Seq.h"
#include "PlainSeqFileLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "EigenUtil.h"
#include <boost/format.hpp>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

enum SE3Format { XYZQWQXQYQZ, XYZQXQYQZQW, XYZRPY };

}


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


bool MultiSE3Seq::doReadSeq(const Mapping& archive, std::ostream& os)
{
    bool result = false;

    if(!BaseSeqType::doReadSeq(archive, os)){
        return false;
    }

    double version;
    if(!archive.read("formatVersion", version)){
        version = 1.0;
    }
    string formatKey;
    function<bool(const string& type)> checkType;
    if(version >= 2.0){
        formatKey = "SE3Format";
        checkType = [](const string& type){
            return (type == "MultiSE3Seq");
        };
    } else {
        formatKey = "format";
        checkType = [](const string& type){
            return (type == "MultiSE3Seq" || type == "MultiSe3Seq" || type == "MultiAffine3Seq");
        };
    }

    const int nParts = archive.get<int>("numParts");
    
    if(checkType(archive.get<string>("type"))){
        string se3format = archive.get<string>(formatKey);
        const Listing& frames = *archive.findListing("frames");
        if(!frames.isValid()){
            setDimension(0, nParts);
            result = true;
        } else {
            const int nFrames = frames.size();
            setDimension(nFrames, nParts);

            if(se3format == "XYZQWQXQYQZ"){
                readFrames(nParts, nFrames, frames, true, true);
                result = true;
            } else if(se3format == "XYZQXQYQZQW"){
                readFrames(nParts, nFrames, frames, true, false);
                result = true;
            } else if(se3format == "XYZRPY"){
                readFrames(nParts, nFrames, frames, false, false);
                result = true;
            }
            if(!result){
                os << format(_("SE3 format \"%1%\" is unsupported.")) % se3format << endl;
            }
        }
    }

    return result;
}


void MultiSE3Seq::readFrames(int nParts, int nFrames, const Listing& frames, bool isQuaternion, bool isWXYZ)
{
    for(int i=0; i < nFrames; ++i){
        const Listing& values = *frames[i].toListing();
        auto f = frame(i);
        const int n = std::min(values.size(), nParts);
        for(int j=0; j < n; ++j){
            const Listing& v = *values[j].toListing();
            SE3& s = f[j];
            if(isQuaternion){
                if(v.size() != 7){
                    v.throwException(_("The number of elements specified as a SE3 value is invalid."));
                } else {
                    s.translation() << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
                    if(isWXYZ){
                        s.rotation() = Quat(
                            v[3].toDouble(), v[4].toDouble(), v[5].toDouble(), v[6].toDouble());
                    } else {
                        s.rotation() = Quat(
                            v[6].toDouble(), v[3].toDouble(), v[4].toDouble(), v[5].toDouble());
                    }
                }
            } else { // RPY
                if(v.size() != 6){
                    v.throwException(_("The number of elements specified as a SE3 value is invalid."));
                } else {
                    s.translation() << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
                    s.rotation() = rotFromRpy(v[3].toDouble(), v[4].toDouble(), v[5].toDouble());
                }
            }
        }
    }
}


static void writeSE3(YAMLWriter& writer, const SE3& value)
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
    if(!BaseSeqType::doWriteSeq(writer)){
        return false;
    }

    double version = writer.info("formatVersion", 2.0);

    string formatKey;
    if(version >= 2.0){
        formatKey = "SE3Format";
    } else {
        formatKey = "format";
    }

    writer.putKeyValue(formatKey, "XYZQWQXQYQZ");
    
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


bool MultiSE3Seq::loadPlainMatrixFormat(const std::string& filename, std::ostream& os)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename, os)){
        return false;
    }

    int n = loader.numParts();
    if(n < 12 || (n % 12) != 0){
        os << format(_("\"%1%\" does not have elements in multiple of twelve (each 3 for position vectors, 9 for attitde matrices)"))
            % filename << endl;
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


bool MultiSE3Seq::loadPlainRpyFormat(const std::string& filename, std::ostream& os)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename, os)){
        return false;
    }

    int n = loader.numParts();
    if(n != 3){
        os << format(_("\"%1%\" does not have a multiple of 3 elements (R,P,Y)")) % filename << endl;
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


bool MultiSE3Seq::saveTopPartAsPlainMatrixFormat(const std::string& filename, std::ostream& os)
{
    boost::format f("%1$.4f");
    const int nFrames = numFrames();

    if(nFrames > 0 && numParts() > 0){

        ofstream file(filename.c_str());
        if(!file){
            os << format(_("\"%1%\" cannot be opened.")) % filename << endl;
            return false;
        }

        const double r = frameRate();

        Part base = part(0);
        for(int i=0; i < nFrames; ++i){
            file << (f % (i / r));
            const SE3& x = base[i];
            for(int j=0; j < 3; ++j){
                file << " " << x.translation()[j];
            }
            Matrix3 R(x.rotation());
            for(int j=0; j < 3; ++j){
                for(int k=0; k < 3; ++k){
                    double m = R(j, k);
                    if(fabs(m) < 1.0e-14){
                        m = 0.0;
                    }
                    file << " " << m;
                }
            }
            file << " 0 0 0 0 0 0"; // velocity elements (dv, omega)
            file << "\n";
        }

        return true;
    }

    return false;
}
