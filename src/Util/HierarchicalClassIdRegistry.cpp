#include "HierarchicalClassIdRegistry.h"
#include <mutex>
#include <unordered_map>
#include <typeindex>
#include <vector>

using namespace std;
using namespace cnoid;

namespace cnoid {

class HierarchicalClassIdRegistryBase::Impl
{
public:
    std::mutex polymorphicIdMutex;
    typedef std::unordered_map<std::type_index, int> PolymorphicIdMap;
    PolymorphicIdMap polymorphicIdMap;
    std::vector<int> superClassPolymorphicIdMap;

    int registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType);
};

}


HierarchicalClassIdRegistryBase::HierarchicalClassIdRegistryBase()
{
    impl = new Impl;
}


HierarchicalClassIdRegistryBase::~HierarchicalClassIdRegistryBase()
{
    delete impl;
}


int HierarchicalClassIdRegistryBase::registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType)
{
    return impl->registerClassAsTypeInfo(type, superType);
}


int HierarchicalClassIdRegistryBase::Impl::registerClassAsTypeInfo(const std::type_info& type, const std::type_info& superType)
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
    int id;
    if(type == superType){
        id = superClassId;
    } else {
        id = polymorphicIdMap.size();
        polymorphicIdMap[type] = id;
        if(id >= static_cast<int>(superClassPolymorphicIdMap.size())){
            superClassPolymorphicIdMap.resize(id + 1, -1);
        }
        superClassPolymorphicIdMap[id] = superClassId;
    }

    return id;
}


int HierarchicalClassIdRegistryBase::classId(const std::type_info& type, int unknownClassId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);

    auto iter = impl->polymorphicIdMap.find(type);
    if(iter != impl->polymorphicIdMap.end()){
        return iter->second;
    }
    return unknownClassId;
}


int HierarchicalClassIdRegistryBase::superClassId(int polymorphicId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->superClassPolymorphicIdMap[polymorphicId];
}


int HierarchicalClassIdRegistryBase::numRegisteredClasses() const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->polymorphicIdMap.size();
}
