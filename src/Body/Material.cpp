/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Material.h"
#include <mutex>
#include <unordered_map>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

std::mutex idMutex;
std::unordered_map<std::string, int> nameToIdMap;
std::vector<std::string> idToNameMap;

// register "default" material as id = 0
struct DefaultIdIntialization {
    DefaultIdIntialization(){
        nameToIdMap["default"] = 0;
        nameToIdMap["Default"] = 0;
        idToNameMap.push_back("default");
    }
};

}


int Material::idOfName(const std::string& name)
{
    std::lock_guard<std::mutex> guard(idMutex);

    static DefaultIdIntialization defaultIdInitialization;

    if(name.empty()){
        return 0;
    }

    int id;
    auto iter = nameToIdMap.find(name);
    if(iter != nameToIdMap.end()){
        id = iter->second;
    } else {
        id = idToNameMap.size();
        nameToIdMap.insert(make_pair(name, id));
        idToNameMap.push_back(name);
    }
    return id;
}


std::string Material::nameOfId(int id)
{
    std::lock_guard<std::mutex> guard(idMutex);
    if(id < static_cast<int>(idToNameMap.size())){
        return idToNameMap[id];
    }
    return std::string();
}


Material::Material()
{
    roughness_ = 0.5;
    viscosity_ = 1.0;
    info_ = new Mapping;
}


Material::Material(const Material& org)
    : name_(org.name_)
{
    roughness_ = org.roughness_;
    viscosity_ = org.viscosity_;
    info_ = org.info_->cloneMapping();
}


Material::Material(const Mapping* info)
{
    roughness_ = 0.5;
    viscosity_ = 1.0;

    info_ = info->cloneMapping();
    info_->extract("name", name_);
    info_->extract("roughness", roughness_);
    info_->extract("viscosity", viscosity_);
}


Material::~Material()
{

}


template<> bool Material::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> int Material::info(const std::string& key, const int& defaultValue) const
{
    int value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> double Material::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}
