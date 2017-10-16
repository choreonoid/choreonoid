/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Material.h"
#include <mutex>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

std::mutex idMutex;
std::unordered_map<std::string, int> idMap;
int idCounter = 0;

// register "default" material as id = 0
struct IdMapInitialization {
    IdMapInitialization(){
        Material::id("default");
    }
} idMapInitialization;

}


Material::Material()
{
    roughness_ = 0.5;
    viscosity_ = 0.0;
    info_ = new Mapping;
}


Material::Material(const Material& org)
{
    roughness_ = org.roughness_;
    viscosity_ = org.viscosity_;
    info_ = org.info_->cloneMapping();
}


Material::~Material()
{

}


int Material::id(const std::string name)
{
    std::lock_guard<std::mutex> guard(idMutex);

    int id;
    auto iter = idMap.find(name);
    if(iter != idMap.end()){
        id = iter->second;
    } else {
        id = idCounter++;
        idMap.insert(make_pair(name, idCounter));
    }
    return id;
}
        

template<> double Material::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> bool Material::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}
