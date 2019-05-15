#include "EigenArchive.h"

using namespace cnoid;

template<typename Scalar>
static bool read_(const Mapping& mapping, const std::string& key, Eigen::AngleAxis<Scalar>& r)
{
    const Listing& s = *mapping.findListing(key);
    if(s.isValid() && s.size() == 4){
        r.axis() << s[0].to<Scalar>(), s[1].to<Scalar>(), s[2].to<Scalar>();
        r.angle() = s[3].toAngle();
        return true;
    }
    return false;
}


bool cnoid::read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r)
{
    return read_(mapping, key, r);
}


bool cnoid::read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisf& r)
{
    return read_(mapping, key, r);
}


template<typename Scalar>
static Listing& write_(Mapping& mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& r)
{
    Listing& s = *mapping.createFlowStyleListing(key);
    s.setDoubleFormat("%.9g");
    s.append(r.axis()[0]);
    s.append(r.axis()[1]);
    s.append(r.axis()[2]);
    s.append(r.angle());
    return s;
}


Listing& cnoid::write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r)
{
    return write_(mapping, key, r);
}


Listing& cnoid::write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisf& r)
{
    return write_(mapping, key, r);
}


template<typename ValueType>
static bool read_(const Mapping& mapping, const std::string& key, std::function<void(const ValueType& value)> setterFunc)
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
    return read_(mapping, key, setterFunc);
}
