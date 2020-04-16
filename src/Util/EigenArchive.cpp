#include "EigenArchive.h"

using namespace cnoid;

template<typename Scalar, int mode>
static bool readAngleAxis_(const Mapping& mapping, const std::string& key, Eigen::AngleAxis<Scalar>& r)
{
    const Listing& s = *mapping.findListing(key);
    if(s.isValid() && s.size() == 4){
        r.axis() << s[0].to<Scalar>(), s[1].to<Scalar>(), s[2].to<Scalar>();
        if(mode == 0){
            r.angle() = s[3].toAngle();
        } else if(mode == 1){
            r.angle() = radian(s[3].to<Scalar>());
        }
        return true;
    }
    return false;
}


bool cnoid::readAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r)
{
    return readAngleAxis_<double, 0>(mapping, key, r);
}


bool cnoid::readDegreeAngleAxis(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r)
{
    return readAngleAxis_<double, 1>(mapping, key, r);
}


bool cnoid::read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r)
{
    return readAngleAxis_<double, 0>(mapping, key, r);
}


template<typename Scalar, int mode>
static Listing& writeAngleAxis_(Mapping& mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& r)
{
    Listing& s = *mapping.createFlowStyleListing(key);
    s.append(r.axis()[0]);
    s.append(r.axis()[1]);
    s.append(r.axis()[2]);
    if(mode == 0){
        s.append(r.angle());
    } else if(mode == 1){
        s.append(degree(r.angle()));
    }
    return s;
}


Listing& cnoid::writeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r)
{
    return writeAngleAxis_<double, 0>(mapping, key, r);
}


Listing& cnoid::writeDegreeAngleAxis(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r)
{
    return writeAngleAxis_<double, 1>(mapping, key, r);
}


Listing& cnoid::write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r)
{
    return writeAngleAxis_<double, 0>(mapping, key, r);
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
