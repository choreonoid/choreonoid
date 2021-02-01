/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTACT_MATERIAL_H
#define CNOID_BODY_CONTACT_MATERIAL_H

#include "Material.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ContactMaterial : public Referenced
{
  public:
    ContactMaterial();
    ContactMaterial(const Mapping* info);
    ContactMaterial(const ContactMaterial& org);
    ~ContactMaterial();

    double friction() const { return staticFriction_; }
    void setFriction(double mu){ staticFriction_ = dynamicFriction_ = mu; }
        
    double staticFriction() const { return staticFriction_; }
    void setStaticFriction(double mu) { staticFriction_ = mu; }
    
    double dynamicFriction() const { return dynamicFriction_; }
    void setDynamicFriction(double mu) { dynamicFriction_ = mu; }
    
    double restitution() const { return restitution_; }
    void setRestitution(double r) { restitution_ = r; }

    Mapping* info() { return info_; }
    const Mapping* info() const { return info_; }
    template<typename T> T info(const std::string& key, const T& defaultValue) const;

  private:
    double staticFriction_;
    double dynamicFriction_;
    double restitution_;
    MappingPtr info_;

    void init();
};

template<> CNOID_EXPORT bool ContactMaterial::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT int ContactMaterial::info(const std::string& key, const int& defaultValue) const;
template<> CNOID_EXPORT double ContactMaterial::info(const std::string& key, const double& defaultValue) const;

typedef ref_ptr<ContactMaterial> ContactMaterialPtr;

}

#endif
