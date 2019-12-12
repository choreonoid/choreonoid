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
    
private:
    HierarchicalClassRegistry<ObjectBase>& registry;
    std::vector<Function> dispatchTable;
    std::vector<bool> isFixed;
    bool isDirty;

public:
    PolymorphicFunctionSet(HierarchicalClassRegistry<ObjectBase>& registry)
        : registry(registry)
    {
        isDirty = false;
    }

    bool empty() const {
        return dispatchTable.empty();
    }

    void setFunction(const std::type_info& type, Function func)
    {
        int id = registry.template classId(type);
        if(id >= 0){
            if(id >= static_cast<int>(dispatchTable.size())){
                dispatchTable.resize(id + 1);
                isFixed.resize(id + 1, false);
            }
            dispatchTable[id] = func;
            isFixed[id] = true;
            isDirty = true;
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
            dispatchTable[id] = nullptr;
            isFixed[id] = false;
            if(doUpdate){
                updateDispatchTable();
            }
        }
    }

    void updateDispatchTable()
    {
        const size_t numTypes = registry.numRegisteredClasses();
        if(dispatchTable.size() == numTypes && !isDirty){
            return;
        }
        
        if(dispatchTable.size() < numTypes){
            dispatchTable.resize(numTypes);
            isFixed.resize(numTypes, false);
        }
    
        const int n = dispatchTable.size();
        for(int i=0; i < n; ++i){
            if(!isFixed[i]){
                dispatchTable[i] = nullptr;
            }
        }
        for(int i=0; i < n; ++i){
            if(!dispatchTable[i]){
                int id = i;
                while(true){
                    int superTypeId = registry.superClassId(id);
                    if(superTypeId < 0){
                        break;
                    }
                    if(dispatchTable[superTypeId]){
                        dispatchTable[i] = dispatchTable[superTypeId];
                        break;
                    }
                    id = superTypeId;
                }
            }
        }
    }

    inline void dispatch(ObjectBase* obj)
    {
        const int id = obj->classId();
        if(id >= static_cast<int>(dispatchTable.size())){
            updateDispatchTable();
        }
        const auto& func = dispatchTable[id];
        if(func){
            func(obj);
        } 
    }

    template <class Object>
    inline void dispatchAs(Object* obj){
        const auto& func = dispatchTable[registry.template classId<Object>(0)];
        if(func){
            func(obj);
        }
    }

    class Dispatcher {
    public:
        Dispatcher(PolymorphicFunctionSet<ObjectBase>& pfs) : pfs(pfs) { }
        template<class Object> void dispatchAs(Object* obj){
            pfs.dispatchAs<Object>(obj);
        }
    private:
        PolymorphicFunctionSet<ObjectBase>& pfs;
    };

    Dispatcher dispatcher(){
        return Dispatcher(*this);
    }
};

}

#endif
