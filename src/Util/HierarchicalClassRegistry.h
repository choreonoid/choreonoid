#ifndef CNOID_UTIL_HIERARCHICAL_CLASS_REGISTRY_H
#define CNOID_UTIL_HIERARCHICAL_CLASS_REGISTRY_H

#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT HierarchicalClassRegistryBase
{
public:
    HierarchicalClassRegistryBase();
    ~HierarchicalClassRegistryBase();

    //! \return The class ID of registered class
    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType);

    int superClassId(int classId) const;
    int numRegisteredClasses() const;

protected:
    int getClassId(const std::type_info& type, int unknownClassId = -1) const;

private:
    class Impl;
    Impl* impl;
};

template<class BaseClass>
class HierarchicalClassRegistry : public HierarchicalClassRegistryBase
{
public:
    HierarchicalClassRegistry() {
        registerClassAsTypeInfo(typeid(BaseClass), typeid(BaseClass));
    }

    HierarchicalClassRegistry(const HierarchicalClassRegistry& org) = delete;
    
    template<class TargetClass, class SuperClass = BaseClass>
    HierarchicalClassRegistry<BaseClass>& registerClass() {
        registerClassAsTypeInfo(typeid(TargetClass), typeid(SuperClass));
        return *this;
    }

    template<class Object>
    bool hasRegistration() const {
        return getClassId(typeid(Object)) >= 0;
    }

    template<class Object>
    int classId(int unknownClassId = -1) const {
        return getClassId(typeid(Object), unknownClassId);
    }

    int classId(const std::type_info& type, int unknownClassId = -1) const {
        return getClassId(type, unknownClassId);
    }

    int classId(const BaseClass* object, int unknownClassId = -1) const {
        return getClassId(typeid(*object), unknownClassId);
    }
};

}

#endif

