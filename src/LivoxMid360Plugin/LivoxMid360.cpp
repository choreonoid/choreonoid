#include "LivoxMid360.h"
#include <cnoid/StdBodyFileUtil>
#include <cnoid/StdBodyLoader>
#include <cnoid/StdSceneReader>
#include <cnoid/ValueTree>
#include <cnoid/MathUtil>
#include <fstream>

using namespace std;
using namespace cnoid;


LivoxMid360::LivoxMid360()
{
    setNumSamples(24000);
    angularPrecision_ = radian(0.5);
    sphericalAngleSeq_ = make_shared<std::vector<Vector2f>>();
}


LivoxMid360::LivoxMid360(const LivoxMid360& org, bool copyStateOnly)
    : RangeSensor(org, copyStateOnly)
{
    copyLivoxMid360StateFrom(org, false, false);

    if(!copyStateOnly){
        sphericalAngleSeq_ = org.sphericalAngleSeq_;
    }
}


Referenced* LivoxMid360::doClone(CloneMap*) const
{
    return new LivoxMid360(*this, false);
}


const char* LivoxMid360::typeName() const
{
    return "LivoxMid360";
}


void LivoxMid360::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(LivoxMid360)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyLivoxMid360StateFrom(static_cast<const LivoxMid360&>(other), true, true);
}


void LivoxMid360::copyLivoxMid360StateFrom(const LivoxMid360& other, bool doCopyRangeSensorState, bool doCopyRangeData)
{
    if(doCopyRangeSensorState){
        RangeSensor::copyRangeSensorStateFrom(other, true, doCopyRangeData);
    }
    setNumSamples(other.numSamples_);
    angularPrecision_ = other.angularPrecision_;
}


DeviceState* LivoxMid360::cloneState() const
{
    return new LivoxMid360(*this, true);
}


void LivoxMid360::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(LivoxMid360))){
        RangeSensor::forEachActualType(func);
    }
}


void LivoxMid360::setNumSamples(int n)
{
    if(n >= 1){
        setNumPitchSamples(1);
        setNumYawSamples(n);
        numSamples_ = n;
    }
}


bool LivoxMid360::readSpecifications(const Mapping* info, const std::filesystem::path& baseDirPath)
{
    if(!RangeSensor::readSpecifications(info)){
        return false;
    }

    int n;
    if(info->read("samples", n)){
        setNumSamples(n);
    } else {
        setNumSamples(numYawSamples() * numPitchSamples());
    }
    
    info->readAngle("angular_precision", angularPrecision_);

    string filename;
    if(info->read("angle_seq_file", filename)){
        std::filesystem::path filePath(filename);
        if(filePath.is_relative()){
            filePath = baseDirPath / filePath;
        }
        if(!loadSphericalAngleSeqFile(filePath.string())){
            info->throwException("Illegal scan angle csv file");
        }
    }

    return true;
}


bool LivoxMid360::loadSphericalAngleSeqFile(const std::string& filename)
{
    std::ifstream file(filename);
    
    if(!file.is_open()){
        return false;
    }
    
    sphericalAngleSeq_ = make_shared<std::vector<Vector2f>>();

    // Skip header line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    float time, azimuth, zenith;
    char comma;
    
    while(file >> time >> comma >> azimuth >> comma >> zenith){
        if(file.fail()){
            return false;
        }
        if(comma != ','){
            return false;
        }
        double pitch = (M_PI / 2.0) - radian(zenith);
        double yaw = 2.0 * M_PI - radian(azimuth);
        sphericalAngleSeq_->emplace_back(yaw, pitch);
    }
    
    return true;
}


bool LivoxMid360::writeSpecifications(Mapping* info) const
{
    if(!RangeSensor::writeSpecifications(info)){
        return false;
    }
    
    info->write("samples", numSamples_);
    info->write("angular_precision", angularPrecision_);

    if(!sphericalAngleSeqFilename.empty()){
        info->write("angle_seq_file", sphericalAngleSeqFilename);
    }

    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<LivoxMid360>
registerLivoxMid360(
    "LivoxMid360",
     [](StdBodyLoader* loader, const Mapping* info){
         LivoxMid360Ptr sensor = new LivoxMid360;
         auto baseDirPath = loader->sceneReader()->baseDirPath();
         if(sensor->readSpecifications(info, baseDirPath)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const LivoxMid360* sensor){
        return sensor->writeSpecifications(info);
    });
}
