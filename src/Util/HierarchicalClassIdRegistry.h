#ifndef CNOID_UTIL_HIERARCHICAL_CLASS_ID_REGISTRY_H
#define CNOID_UTIL_HIERARCHICAL_CLASS_ID_REGISTRY_H

#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT HierarchicalClassIdRegistryBase
{
public:
    HierarchicalClassIdRegistryBase();
    ~HierarchicalClassIdRegistryBase();

    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType);
    int classId(const std::type_info& type, int unknownClassId = -1) const;
    int superClassId(int classId) const;
    int numRegisteredClasses() const;

private:
    class Impl;
    Impl* impl;
};

template<class BaseClass>
class HierarchicalClassIdRegistry : public HierarchicalClassIdRegistryBase
{
public:
    HierarchicalClassIdRegistry() {
        registerClassAsTypeInfo(typeid(BaseClass), typeid(BaseClass));
    }
    
    template<class TargetClass, class SuperClass = BaseClass>
    int registerClass() {
        return registerClassAsTypeInfo(typeid(TargetClass), typeid(SuperClass));
    }

    int classId(const BaseClass* object) const {
        return HierarchicalClassIdRegistryBase::classId(typeid(*object), 0);
    }
};

}

#endif

