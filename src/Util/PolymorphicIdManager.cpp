#include "PolymorphicIdManager.h"
#include <mutex>
#include <unordered_map>
#include <typeindex>
#include <vector>

using namespace std;
using namespace cnoid;

namespace cnoid {

class PolymorphicIdManagerBase::Impl
{
public:
    std::mutex polymorphicIdMutex;
    typedef std::unordered_map<std::type_index, int> PolymorphicIdMap;
    PolymorphicIdMap polymorphicIdMap;
    std::vector<int> superTypePolymorphicIdMap;

    int registerTypeAsTypeInfo(const std::type_info& type, const std::type_info& superType);
};

}


PolymorphicIdManagerBase::PolymorphicIdManagerBase()
{
    impl = new Impl;
}


PolymorphicIdManagerBase::~PolymorphicIdManagerBase()
{
    delete impl;
}


int PolymorphicIdManagerBase::registerTypeAsTypeInfo(const std::type_info& type, const std::type_info& superType)
{
    return impl->registerTypeAsTypeInfo(type, superType);
}


int PolymorphicIdManagerBase::Impl::registerTypeAsTypeInfo(const std::type_info& type, const std::type_info& superType)
{
    std::lock_guard<std::mutex> lock(polymorphicIdMutex);

    int superTypeId;
    auto iter = polymorphicIdMap.find(superType);
    if(iter != polymorphicIdMap.end()){
        superTypeId = iter->second;
    } else {
        superTypeId = polymorphicIdMap.size();
        polymorphicIdMap[superType] = superTypeId;
    }
    int id;
    if(type == superType){
        id = superTypeId;
    } else {
        id = polymorphicIdMap.size();
        polymorphicIdMap[type] = id;
        if(id >= static_cast<int>(superTypePolymorphicIdMap.size())){
            superTypePolymorphicIdMap.resize(id + 1, -1);
        }
        superTypePolymorphicIdMap[id] = superTypeId;
    }

    return id;
}


int PolymorphicIdManagerBase::findPolymorphicId(const std::type_info& type) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);

    auto iter = impl->polymorphicIdMap.find(type);
    if(iter != impl->polymorphicIdMap.end()){
        return iter->second;
    }
    return -1;
}


int PolymorphicIdManagerBase::findSuperTypePolymorphicId(int polymorphicId) const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->superTypePolymorphicIdMap[polymorphicId];
}


int PolymorphicIdManagerBase::numPolymorphicTypes() const
{
    std::lock_guard<std::mutex> lock(impl->polymorphicIdMutex);
    return impl->polymorphicIdMap.size();
}
