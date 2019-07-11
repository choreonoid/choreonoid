/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3Seq.h"
#include "PlainSeqFileLoader.h"
#include "ValueTree.h"
#include "YAMLWriter.h"
#include "EigenUtil.h"
#include "GeneralSeqReader.h"
#include <fmt/format.h>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


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


std::shared_ptr<AbstractSeq> MultiSE3Seq::cloneSeq() const
{
    return std::make_shared<MultiSE3Seq>(*this);
}


MultiSE3Seq::~MultiSE3Seq()
{

}


SE3 MultiSE3Seq::defaultValue() const
{
    return SE3(Vector3::Zero(), Quat::Identity());
}


bool MultiSE3Seq::doReadSeq(const Mapping* archive, std::ostream& os)
{
    GeneralSeqReader reader(os);

    reader.setCustomSeqTypeChecker(
        [&](GeneralSeqReader& reader, const string& type){
            if(reader.formatVersion() >= 2.0){
                return reader.checkSeqType(type);
            } else {
                return (type == "MultiSE3Seq" || type == "MultiSe3Seq" || type == "MultiAffine3Seq");
            }
        });

    if(!reader.readHeaders(archive, this)){
        return false;
    }
    
    string se3format;
    if(reader.formatVersion() >= 2.0){
        se3format = archive->get<string>("SE3Format");
    } else {
        se3format = archive->get<string>("format");
    }

    bool result = false;
    static const char* illegal_number_of_SE3_elements_message =
        _("The number of elements specified as a SE3 value is invalid.");
    
    if(se3format == "XYZQWQXQYQZ"){
        result = reader.readFrames<MultiSE3Seq>(
            archive, this,
            [](const ValueNode& node, SE3& value){
                const Listing& v = *node.toListing();
                if(v.size() != 7){
                    v.throwException(illegal_number_of_SE3_elements_message);
                }
                value.translation() << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
                value.rotation() = Quat(v[3].toDouble(), v[4].toDouble(), v[5].toDouble(), v[6].toDouble());
            });

    } else if(se3format == "XYZQXQYQZQW" && reader.formatVersion() < 2.0){
        result = reader.readFrames<MultiSE3Seq>(
            archive, this,
            [](const ValueNode& node, SE3& value){
                const Listing& v = *node.toListing();
                if(v.size() != 7){
                    v.throwException(illegal_number_of_SE3_elements_message);
                }
                value.translation() << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
                value.rotation() = Quat(v[6].toDouble(), v[3].toDouble(), v[4].toDouble(), v[5].toDouble());
            });

    } else if(se3format == "XYZRPY"){
        result = reader.readFrames<MultiSE3Seq>(
            archive, this,
            [](const ValueNode& node, SE3& value){
                const Listing& v = *node.toListing();
                if(v.size() != 6){
                    v.throwException(illegal_number_of_SE3_elements_message);
                }
                value.translation() << v[0].toDouble(), v[1].toDouble(), v[2].toDouble();
                value.rotation() = rotFromRpy(v[3].toDouble(), v[4].toDouble(), v[5].toDouble());
            });

    } else {
        os << format(_("SE3 format \"{}\" is not supported."), se3format) << endl;
    }

    return result;
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
    

bool MultiSE3Seq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    double version = writer.getOrCreateInfo("formatVersion", 2.0);
    
    return BaseSeqType::doWriteSeq(
        writer,
        [&](){
            string formatKey;
            if(version >= 2.0){
                formatKey = "SE3Format";
            } else {
                formatKey = "format";
            }

            writer.putKeyValue(formatKey, "XYZQWQXQYQZ");

            if(additionalPartCallback) additionalPartCallback();
    
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
        });
}


bool MultiSE3Seq::loadPlainMatrixFormat(const std::string& filename, std::ostream& os)
{
    PlainSeqFileLoader loader;

    if(!loader.load(filename, os)){
        return false;
    }

    int n = loader.numParts();
    if(n < 12 || (n % 12) != 0){
        os << format(_("\"{}\" does not have elements in multiple of twelve (each 3 for position vectors, 9 for attitde matrices)"),
            filename) << endl;
        return false;
    }
    int m = n / 12;
  
    setDimension(loader.numFrames(), m);
    setTimeStep(loader.timeStep());

    int f = 0;
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
        os << format(_("\"{}\" does not have a multiple of 3 elements (R,P,Y)"), filename) << endl;
        return false;
    }

    setDimension(loader.numFrames(), 1);
    setTimeStep(loader.timeStep());

    int f = 0;
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
    const int nFrames = numFrames();

    if(nFrames > 0 && numParts() > 0){

        ofstream file(filename.c_str());
        if(!file){
            os << format(_("\"{}\" cannot be opened."), filename) << endl;
            return false;
        }

        const double r = frameRate();

        Part base = part(0);
        for(int i=0; i < nFrames; ++i){
            file << format("{0:.4f}", (i / r));
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
