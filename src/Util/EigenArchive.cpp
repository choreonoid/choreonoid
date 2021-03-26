#include "EigenArchive.h"
#include "gettext.h"

using namespace cnoid;

template<typename Scalar>
static void readSingleAngleAxis(const Listing& elements, Eigen::AngleAxis<Scalar>& aa, bool isDegree)
{
    if(elements.size() != 4){
        elements.throwException(_("Angle-axis value must have four elements representing the axis and the angle"));
    }
    aa.axis() << elements[0].to<Scalar>(), elements[1].to<Scalar>(), elements[2].to<Scalar>();

    auto size = aa.axis().norm();
    if(size < 1.0e-6){
        elements.throwException(_("Rotation axis is the zero vector"));
    }
    aa.axis() /= size; // normalize

    aa.angle() = elements[3].to<Scalar>();
    
    if(isDegree){
        aa.angle() = radian(aa.angle());
    }
}


template<typename Scalar>
static bool readAngleAxis_(const Mapping* mapping, const std::string& key, Eigen::AngleAxis<Scalar>& aa, bool isDegree)
{
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    
    auto& elements = *mapping->findListing(key);
    if(elements.empty()){
        return false;
    }
    if(!elements[0].isListing()){
        readSingleAngleAxis(elements, aa, isDegree);
    } else {
        readSingleAngleAxis(*elements[0].toListing(), aa, isDegree);
        for(int i = 1; i < elements.size(); ++i){
            AngleAxis aa2;
            readSingleAngleAxis(*elements[i].toListing(), aa2, isDegree);
            aa = aa * aa2;
        }
    }
    return true;
}    


bool cnoid::readAngleAxis(const Mapping* mapping, const std::string& key, Eigen::AngleAxisd& aa)
{
    return readAngleAxis_<double>(mapping, key, aa, !mapping->isForcedRadianMode());
}


bool cnoid::readAngleAxis(const Mapping* mapping, std::initializer_list<const char*> keys, Eigen::AngleAxisd& aa)
{
    for(auto& key : keys){
        if(readAngleAxis(mapping, key, aa)){
            return true;
        }
    }
    return false;
}


bool cnoid::readAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa)
{
    return readAngleAxis_<double>(&mapping, key, aa, !mapping.isForcedRadianMode());
}


bool cnoid::readDegreeAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa)
{
    return readAngleAxis_<double>(&mapping, key, aa, true);
}


bool cnoid::readRadianAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa)
{
    return readAngleAxis_<double>(&mapping, key, aa, false);
}


bool cnoid::read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& aa)
{
    return readAngleAxis_<double>(&mapping, key, aa, !mapping.isForcedRadianMode());
}


template<typename Scalar, int mode>
static Listing& writeAngleAxis_(Mapping& mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& aa)
{
    return *writeAngleAxis_(&mapping, key, aa);
}


template<typename Scalar, int mode>
static Listing* writeAngleAxis_(Mapping* mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& aa)
{
    auto s = mapping->createFlowStyleListing(key);
    s->append(aa.axis()[0]);
    s->append(aa.axis()[1]);
    s->append(aa.axis()[2]);
    if(mode == 0){
        s->append(aa.angle());
    } else if(mode == 1){
        s->append(degree(aa.angle()));
    }
    return s;
}


Listing* cnoid::writeAngleAxis(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return writeAngleAxis_<double, 0>(mapping, key, aa);
}


Listing& cnoid::writeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return *writeAngleAxis_<double, 0>(&mapping, key, aa);
}


Listing* cnoid::writeDegreeAngleAxis(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return writeAngleAxis_<double, 1>(mapping, key, aa);
}


Listing& cnoid::writeDegreeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return *writeAngleAxis_<double, 1>(&mapping, key, aa);
}


Listing* cnoid::writeRadianAngleAxis_(Mapping* mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return writeAngleAxis_<double, 0>(mapping, key, aa);
}


Listing& cnoid::write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& aa)
{
    return *writeAngleAxis_<double, 0>(&mapping, key, aa);
}


template<typename ValueType>
static bool read__(const Mapping& mapping, const std::string& key, std::function<void(const ValueType& value)> setterFunc)
{
    ValueType value;
    if(read(mapping, key, value)){
        setterFunc(value);
        return true;
    }
    return false;
}


bool cnoid::read(const Mapping& mapping, const std::string& key, std::function<void(const Eigen::Vector3d& value)> setterFunc)
{
    return read__(mapping, key, setterFunc);
}
