#ifndef CNOID_UTIL_HIERARCHICAL_CLASS_REGISTRY_H
#define CNOID_UTIL_HIERARCHICAL_CLASS_REGISTRY_H

#include <typeinfo>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT HierarchicalClassRegistryBase
{
public:
    ~HierarchicalClassRegistryBase();

    void reserve(int n);

    //! \return The class ID of registered class
    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType, const char* name = nullptr);

    [[deprecated]]
    int superClassId(int classId) const { return getSuperClassId(classId); }
    int getSuperClassId(int classId) const;
    std::string getClassName(int classId) const;
    int numRegisteredClasses() const;

protected:
    HierarchicalClassRegistryBase(const std::type_info& baseType, const char* baseTypeName);
    int getClassId(const std::type_info& type, int unknownClassId = -1) const;

private:
    class Impl;
    Impl* impl;
};

template<class BaseClass>
class HierarchicalClassRegistry : public HierarchicalClassRegistryBase
{
public:
    HierarchicalClassRegistry(const char* baseClassName = nullptr)
        : HierarchicalClassRegistryBase(typeid(BaseClass), baseClassName) {
    }

    HierarchicalClassRegistry(const HierarchicalClassRegistry& org) = delete;
    
    template<class TargetClass, class SuperClass = BaseClass>
    HierarchicalClassRegistry<BaseClass>& registerClass(const char* name = nullptr) {
        registerClassAsTypeInfo(typeid(TargetClass), typeid(SuperClass), name);
        return *this;
    }

    template<class Object>
    bool hasRegistration() const {
        return HierarchicalClassRegistryBase::getClassId(typeid(Object)) >= 0;
    }

    template<class Object>
    int getClassId(int unknownClassId = -1) const {
        return HierarchicalClassRegistryBase::getClassId(typeid(Object), unknownClassId);
    }

    int getClassId(const std::type_info& type, int unknownClassId = -1) const {
        return HierarchicalClassRegistryBase::getClassId(type, unknownClassId);
    }

    int getClassId(const BaseClass* object, int unknownClassId = -1) const {
        return HierarchicalClassRegistryBase::getClassId(typeid(*object), unknownClassId);
    }

    template<class Object>
    [[deprecated]]
    int classId(int unknownClassId = -1) const {
        return getClassId<Object>(unknownClassId);
    }

    [[deprecated]]
    int classId(const std::type_info& type, int unknownClassId = -1) const {
        return getClassId(type, unknownClassId);
    }

    [[deprecated]]
    int classId(const BaseClass* object, int unknownClassId = -1) const {
        return getClassId(object, unknownClassId);
    }
};

}

#endif

