/**
   \file
   \author Ikumi Susa
*/
#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVERT_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVERT_H

#include <cnoid/ValueTree>
#include <unordered_map>

namespace cnoid{
namespace agxConvert{

inline bool setVector(ValueNodePtr const vnptr, Vector2& vec)
{
    if(!vnptr) return false;
    if(!vnptr->isListing())  return false;
    const Listing& l  = *vnptr->toListing();
    if(l.size() != 2) return false;
    vec = Vector2(l[0].toDouble(), l[1].toDouble());
    return true;
}

inline bool setVector(ValueNodePtr const vnptr, Vector3& vec)
{
    if(!vnptr) return false;
    if(!vnptr->isListing())  return false;
    const Listing& l  = *vnptr->toListing();
    if(l.size() != 3) return false;
    vec = Vector3(l[0].toDouble(), l[1].toDouble(), l[2].toDouble());
    return true;
}

inline bool setVector(ValueNodePtr const vnptr, std::vector<std::string>&vs)
{
    if(!vnptr) return false;
    if(!vnptr->isListing()) return false;
    Listing& list = *vnptr->toListing();
    for(auto it : list){
        vs.push_back(it->toString());
    }
    return true;
}

inline bool setVector(ValueNodePtr const vnptr, const unsigned int checkSize, std::vector<std::string>&vs)
{
    if(!setVector(vnptr, vs)) return false;
    return vs.size() == checkSize;
}

template<typename T>
inline bool setValue(const std::string& key, const std::unordered_map<std::string, T>& map,
    const std::string& defaultValue, T& value)
{
    auto it = map.find(key);
    if(it != map.end()){
        value = it->second;
        return true;
    }else{
        value = map.at(defaultValue);
        return false;
    }
}

template<typename T>
inline bool setValue(ValueNodePtr const vnptr, const std::unordered_map<std::string, T>& map,
    const std::string& defaultValue, T& value)
{
    if(!vnptr) return false;
    const std::string str = vnptr->toString();
    return setValue(str, map, defaultValue, value);
}

template<typename T>
inline void setValue(ValueNodePtr const vnptr, const std::unordered_map<std::string, T>& map,
const std::string& defaultValue, T&value, const std::string& errMessage)
{
    if(vnptr->isValid()){
        if(!setValue(vnptr, map, defaultValue, value))
            vnptr->throwException(errMessage);
    }
}

inline agx::Vec3 toAGX(const Vector3& v)
{
    return agx::Vec3(v.x(), v.y(), v.z());
}

inline Vector3 toCnoid(const agx::Vec3& v)
{
    return Vector3(v.x(), v.y(), v.z());
}

}
}

#endif
