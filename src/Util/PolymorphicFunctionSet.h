/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>
#include <vector>

namespace cnoid {

template<class ObjectBase>
class PolymorphicFunctionSet
{
public:
    typedef std::function<void(ObjectBase* obj)> Function;
    
private:
    std::vector<Function> dispatchTable;
    std::vector<bool> isFixed;
    bool isDirty;

public:
    PolymorphicFunctionSet() {
        const int n = ObjectBase::numPolymorphicTypes();
        dispatchTable.resize(n);
        isFixed.resize(n, false);
        isDirty = true;
    }

    template <class Object>
    void setFunction(Function func){
        int id = ObjectBase::template findPolymorphicId<Object>();
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
    void setFunction(std::function<void(Object* obj)> func){
        setFunction<Object>([func](ObjectBase* obj){ func(static_cast<Object*>(obj)); });
    }

    template <class Object>
    void resetFunction(bool doUpdate = false){
        int id = ObjectBase::template findPolymorphicId<Object>();
        if(id >= 0 && id < dispatchTable.size()){
            dispatchTable[id] = nullptr;
            isFixed[id] = false;
            if(doUpdate){
                updateDispatchTable();
            }
        }
    }

    void updateDispatchTable() {

        const size_t numTypes = ObjectBase::numPolymorphicTypes();
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
                    int superTypeId = ObjectBase::findSuperTypePolymorphicId(id);
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

    inline void dispatch(ObjectBase* obj){
        const int id = obj->polymorhicId();
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
        const auto& func = dispatchTable[ObjectBase::template findPolymorphicId<Object>()];
        if(func){
            func(obj);
        }
    }
};

}

#endif
