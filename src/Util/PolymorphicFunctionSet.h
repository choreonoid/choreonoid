/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>
#include "HierarchicalClassRegistry.h"
#include <vector>

namespace cnoid {

template<class ObjectBase>
class PolymorphicFunctionSet
{
public:
    typedef std::function<void(ObjectBase* obj)> Function;
    
    struct FunctionInfo {
        Function func;
        int implementedClassId;
        FunctionInfo(){
            implementedClassId = -1;
        }
        void set(Function func, int implementedClassId){
            this->func = func;
            this->implementedClassId = implementedClassId;
        }
    };
    
private:
    HierarchicalClassRegistry<ObjectBase>& registry;
    int validDispatchTableSize;
    std::vector<FunctionInfo> dispatchTable;

    int setSuperClassFunction(int id)
    {
        if(dispatchTable[id].func){
            return id;
        }
        int superClassId = registry.superClassId(id);
        if(superClassId < 0){
            return -1;
        }
        int implementedClassId = setSuperClassFunction(superClassId);
        if(implementedClassId >= 0){
            dispatchTable[id].implementedClassId = implementedClassId;
        }
        return implementedClassId;
    }

public:
    PolymorphicFunctionSet(HierarchicalClassRegistry<ObjectBase>& registry)
        : registry(registry)
    {
        validDispatchTableSize = 0;
    }

    bool empty() const {
        return dispatchTable.empty();
    }

    void setFunction(const std::type_info& type, Function func)
    {
        int id = registry.classId(type);
        if(id >= 0){
            if(validDispatchTableSize > id + 1){
                validDispatchTableSize = id + 1;
            }
            if(id >= static_cast<int>(dispatchTable.size())){
                dispatchTable.resize(id + 1);
            }
            dispatchTable[id].func = func;
        }
    }

    template <class Object>
    void setFunction(Function func)
    {
        setFunction(typeid(Object), func);
    }

    template <class Object>
    void setFunction(std::function<void(Object* obj)> func)
    {
        setFunction<Object>([func](ObjectBase* obj){ func(static_cast<Object*>(obj)); });
    }

    template <class Object>
    void resetFunction(bool doUpdate = false)
    {
        int id = registry.template classId<Object>();
        if(id >= 0 && id < dispatchTable.size()){
            if(validDispatchTableSize > id){
                validDispatchTableSize = id;
            }
            dispatchTable[id].set(nullptr, -1);
            if(doUpdate){
                updateDispatchTable();
            }
        }
    }

    bool updateDispatchTable(int idToCheck = 0)
    {
        const size_t numClasses = registry.numRegisteredClasses();
        if(numClasses == validDispatchTableSize){
            return idToCheck < numClasses;
        }
        
        if(dispatchTable.size() != numClasses){
            dispatchTable.resize(numClasses);
        }

        const int n = dispatchTable.size();
        for(int i = validDispatchTableSize; i < n; ++i){
            setSuperClassFunction(i);
        }
        validDispatchTableSize = n;
        
        return idToCheck < validDispatchTableSize;
    }

    bool hasFunctionFor(ObjectBase* obj) const
    {
        auto id = obj->classId();
        if(id >= validDispatchTableSize){
            if(!const_cast<PolymorphicFunctionSet*>(this)->updateDispatchTable(id)){
                return false;
            }
        }
        auto& info = dispatchTable[id];
        return (info.func != nullptr) || (info.implementedClassId >= 0);
    }

    inline void dispatch(ObjectBase* obj, const int id) const
    {
        if(id >= validDispatchTableSize){
            if(!const_cast<PolymorphicFunctionSet*>(this)->updateDispatchTable(id)){
                return;
            }
        }
        auto& info = dispatchTable[id];
        if(info.func){
            info.func(obj);
        } else if(info.implementedClassId >= 0){
            dispatchTable[info.implementedClassId].func(obj);
        }
    }

    inline void dispatch(ObjectBase* obj) const
    {
        dispatch(obj, obj->classId());
    }

    template <class Object>
    inline void dispatchAs(Object* obj) const
    {
        dispatch(obj, registry.template classId<Object>(0));
    }

    class Dispatcher {
    public:
        Dispatcher(const PolymorphicFunctionSet<ObjectBase>& pfs) : pfs(pfs) { }
        template<class Object> void dispatchAs(Object* obj) const {
            pfs.dispatchAs<Object>(obj);
        }
    private:
        const PolymorphicFunctionSet<ObjectBase>& pfs;
    };

    Dispatcher dispatcher() const {
        return Dispatcher(*this);
    }
};

}

#endif
