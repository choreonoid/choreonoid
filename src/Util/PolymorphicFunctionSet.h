/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>

namespace cnoid {

template<class ObjectBase>
class PolymorphicFunctionSet
{
public:
    typedef std::function<void(ObjectBase* obj)> Function;
    
private:
    std::vector<Function> dispatchTable;
    std::vector<bool> dispatchTableFixedness;
    bool isDispatchTableDirty;

public:
    PolymorphicFunctionSet() {
        const int n = ObjectBase::numPolymorphicTypes();
        dispatchTable.resize(n);
        dispatchTableFixedness.resize(n, false);
        isDispatchTableDirty = true;
    }

    template <class Object>
    void setFunction(Function func){
        int id = ObjectBase::template findPolymorphicId<Object>();
        if(id >= 0){
            if(id >= dispatchTable.size()){
                dispatchTable.resize(id + 1);
                dispatchTableFixedness.resize(id + 1, false);
            }
            dispatchTable[id] = func;
            dispatchTableFixedness[id] = true;
            isDispatchTableDirty = true;
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
            dispatchTableFixedness[id] = false;
            if(doUpdate){
                updateDispatchTable();
            }
        }
    }

    void updateDispatchTable() {

        const int numTypes = ObjectBase::numPolymorphicTypes();
        if(dispatchTable.size() == numTypes && !isDispatchTableDirty){
            return;
        }
        
        if(dispatchTable.size() < numTypes){
            dispatchTable.resize(numTypes);
            dispatchTableFixedness.resize(numTypes, false);
        }
    
        const int n = dispatchTable.size();
        for(int i=0; i < n; ++i){
            if(!dispatchTableFixedness[i]){
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
        if(id >= dispatchTable.size()){
            updateDispatchTable();
        }
        const Function& func = dispatchTable[id];
        if(func){
            func(obj);
        } 
    }

    template <class Object>
    inline void dispatchAs(Object* obj){
        const Function& func = dispatchTable[ObjectBase::template findPolymorphicId<Object>()];
        if(func){
            func(obj);
        }
    }
};

}

#endif
