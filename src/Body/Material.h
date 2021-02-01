/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_H
#define CNOID_BODY_MATERIAL_H

#include <cnoid/Referenced>
#include <cnoid/ValueTree>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Material : public Referenced
{
public:
    Material();
    Material(const Mapping* info);
    Material(const Material& org);
    ~Material();

    static int idOfName(const std::string& name);
    [[deprecated("Use idOfName")]]
    static int id(const std::string& name) { return idOfName(name); }
    
    static std::string nameOfId(int id);
    [[deprecated("Use nameOfId")]]
    static std::string name(int id) { return nameOfId(id); }
    
    const std::string& name() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    double roughness() const { return roughness_; }
    void setRoughness(double r) { roughness_ = r; }
    double viscosity() const { return viscosity_; }
    void setViscosity(double v) { viscosity_ = v; }

    Mapping* info() { return info_; }
    const Mapping* info() const { return info_; }
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    
    void resetInfo(Mapping* info) { info_ = info; }
    
private:
    std::string name_;
    double roughness_;
    double viscosity_;
    MappingPtr info_;
};

template<> CNOID_EXPORT bool Material::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT int Material::info(const std::string& key, const int& defaultValue) const;
template<> CNOID_EXPORT double Material::info(const std::string& key, const double& defaultValue) const;

typedef ref_ptr<Material> MaterialPtr;

}

#endif
