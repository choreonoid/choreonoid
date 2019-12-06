#ifndef CNOID_UTIL_POLYMORPHIC_ID_MANAGER_H
#define CNOID_UTIL_POLYMORPHIC_ID_MANAGER_H

#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PolymorphicIdManagerBase
{
public:
    PolymorphicIdManagerBase();
    ~PolymorphicIdManagerBase();

    int registerTypeAsTypeInfo(const std::type_info& type, const std::type_info& superType);
    int findPolymorphicId(const std::type_info& type) const;
    int findSuperTypePolymorphicId(int polymorphicId) const;
    int numPolymorphicTypes() const;

private:
    class Impl;
    Impl* impl;
};

template<class BaseType>
class PolymorphicIdManager : public PolymorphicIdManagerBase
{
public:
    PolymorphicIdManager() {
        registerTypeAsTypeInfo(typeid(BaseType), typeid(BaseType));
    }
    
    template<class TargetType, class SuperType = BaseType>
    int registerType() {
        return registerTypeAsTypeInfo(typeid(TargetType), typeid(SuperType));
    }

    int findPolymorphicId(BaseType* object) const;
};

}

#endif

