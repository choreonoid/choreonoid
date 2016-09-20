/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTACT_ATTRIBUTE_H
#define CNOID_BODY_CONTACT_ATTRIBUTE_H

namespace cnoid {

class ContactAttribute
{
public:
    double staticFriction() const { return staticFriction_; }
    double dynamicFriction() const { return dynamicFriction_; }
    double restitution() const { return restitution_; }

    void setStaticFriction(double mu) { staticFriction_ = mu; }
    void setDynamicFriction(double mu) { dynamicFriction_ = mu; }
    void setRestitution(double r) { restitution_ = r; }
    
private:
    double staticFriction_;
    double dynamicFriction_;
    double restitution_;
};

}

#endif
