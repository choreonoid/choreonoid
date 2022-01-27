#include "HierarchicalClassRegistry.h"
#include <mutex>
#include <unordered_map>
#include <typeindex>
#include <vector>

using namespace std;
using namespace cnoid;

namespace cnoid {

class HierarchicalClassRegistryBase::Impl
{
public:
    std::mutex polymorphicIdMutex;
    typedef std::unordered_map<std::type_index, int> PolymorphicIdMap;
    PolymorphicIdMap polymorphicIdMap;
    std::vector<int> superClassPolymorphicIdMap;
    std::vector<std::string> classNameMap;

    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType, const char* name);
};

}


HierarchicalClassRegistryBase::HierarchicalClassRegistryBase()
{
    impl = new Impl;
}


HierarchicalClassRegistryBase::~HierarchicalClassRegistryBase()
{
    delete impl;
}


void HierarchicalClassRegistryBase::reserve(int n)
{
    impl->polymorphicIdMap.reserve(n);
    impl->superClassPolymorphicIdMap.reserve(n);
    impl->classNameMap.reserve(n);
}


int HierarchicalClassRegistryBase::registerClassAsTypeInfo
(const std::type_info& type, const std::type_info& superType, const char* name)
{
    return impl->registerClassAsTypeInfo(type, superType, name);
}


int HierarchicalClassRegistryBase::Impl::registerClassAsTypeInfo
(const std::type_info& type, const std::type_info& superType, const char* name)
{
    std::lock_guard<std::mutex> lock(polymorphicIdMutex);

    int superClassId;
    auto iter = polymorphicIdMap.find(superType);
    if(iter != polymorphicIdMap.end()){
        superClassId = iter->second;
    } else {
        superClassId = polymorphicIdMap.size();
        polymorphicIdMap[superType] = superClassId;
    }
    const bool hasSuperClass = (type != superType);
    int id;
    
    if(!hasSuperClass){
        id = superClassId;
    } else {
        id = polymorphicIdMap.size();
        polymorphicIdMap[type] = id;
    }
    if(id >= static_cast<int>(superClassPolymorphicIdMap.size())){
        superClassPolymorphicIdMap.resize(id + 1, -1);
    }
    if(hasSuperClass){
        superClassPolymorphicIdMap[id] = superClassId;
    }

    if(name){
        if(id >= static_cast<int>(classNameMap.size())){
            classNameMap.resize(id + 1);
        }
        classNameMap[id] = name;
    }

    return id;
}


int HierarchicalClassRegistryBase::getClassId(const std::type_info& type, int unknownClassId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);

    auto iter = impl->polymorphicIdMap.find(type);
    if(iter != impl->polymorphicIdMap.end()){
        return iter->second;
    }
    return unknownClassId;
}


int HierarchicalClassRegistryBase::getSuperClassId(int polymorphicId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->superClassPolymorphicIdMap[polymorphicId];
}


int HierarchicalClassRegistryBase::numRegisteredClasses() const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->polymorphicIdMap.size();
}


std::string HierarchicalClassRegistryBase::getClassName(int classId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    while(classId > 0 && classId < static_cast<int>(impl->classNameMap.size())){
        const auto& name = impl->classNameMap[classId];
        if(!name.empty()){
            return name;
        }
        classId = impl->superClassPolymorphicIdMap[classId];
    }
    return string();
}
