#include "HierarchicalClassRegistry.h"
#include "Format.h"
#include <mutex>
#include <unordered_map>
#include <typeindex>
#include <vector>
#include <stdexcept>
#include <typeindex>

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
    std::type_index baseTypeIndex;

    Impl(const std::type_info& baseType, const char* baseTypeName);
    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType, const char* name);
};

}


HierarchicalClassRegistryBase::HierarchicalClassRegistryBase(const std::type_info& baseType, const char* baseTypeName)
{
    impl = new Impl(baseType, baseTypeName);
}


HierarchicalClassRegistryBase::Impl::Impl(const std::type_info& baseType, const char* baseTypeName)
    : baseTypeIndex(baseType)
{
    std::lock_guard<std::mutex> lock(polymorphicIdMutex);

    int baseClassId = 0;
    polymorphicIdMap[baseType] = baseClassId;
    superClassPolymorphicIdMap.push_back(-1);
    if(baseTypeName){
        classNameMap.push_back(baseTypeName);
    }
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

    if(type == superType){
        if(std::type_index(type) == baseTypeIndex){
            if(name){
                if(classNameMap.empty()){
                    classNameMap.resize(1);
                }
                classNameMap[0] = name;
            }
            return 0;
        }
        if(name){
            throw std::invalid_argument(
                formatC("Cannot register type {0} as its own supertype in HierarchicalClassRegistry.", name));
        } else {
            throw std::invalid_argument(
                "Cannot register a type as its own supertype in HierarchicalClassRegistry.");
        }
    }

    auto it = polymorphicIdMap.find(superType);
    if(it == polymorphicIdMap.end()){
        if(name){
            throw std::invalid_argument(
                formatC("Specified supertype of {0} is not registered in HierarchicalClassRegistry.", name));
        } else {
            throw std::invalid_argument(
                formatC("Specified supertype of {0} is not registered in HierarchicalClassRegistry.", type.name()));
        }
    }
    int superClassId = it->second;
    
    int id = polymorphicIdMap.size();
    polymorphicIdMap[type] = id;

    if(id >= static_cast<int>(superClassPolymorphicIdMap.size())){
        superClassPolymorphicIdMap.resize(id + 1, -1);
    }
    superClassPolymorphicIdMap[id] = superClassId;

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
