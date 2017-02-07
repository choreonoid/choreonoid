/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_FUNCTION_SET_H

#include <functional>

namespace cnoid {

template<class Processor, class ObjectBase>
class PolymorphicFunctionSet
{
    static const bool USE_EMPTY_FUNCTION_FOR_UNSPECIFIED_TYPES = false;
    
public:
    typedef std::function<void(Processor* proc, ObjectBase* obj)> Function;
    
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
    void setFunction(std::function<void(Processor* proc, Object* obj)> func){
        setFunction<Object>([func](Processor* proc, ObjectBase* obj){ func(proc, static_cast<Object*>(obj)); });
    }

    void updateDispatchTable() {
        if(!isDispatchTableDirty){
            return;
        }
        
        const int numTypes = ObjectBase::numPolymorphicTypes();
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
        if(USE_EMPTY_FUNCTION_FOR_UNSPECIFIED_TYPES){
            for(int i=0; i < n; ++i){
                if(!dispatchTable[i]){
                    dispatchTable[i] = [](Processor* proc, ObjectBase* obj){ };
                }
            }
        }
    }

    void dispatch(Processor* proc, ObjectBase* obj){
        if(USE_EMPTY_FUNCTION_FOR_UNSPECIFIED_TYPES){
            dispatchTable[obj->polymorhicId()](proc, obj);
        } else {
            const Function& func = dispatchTable[obj->polymorhicId()];
            if(func){
                func(proc, obj);
            }
        }
    }

    template <class Object>
    void dispatchAs(Processor* proc, Object* obj){
        const Function& func = dispatchTable[ObjectBase::template findPolymorphicId<Object>()];
        if(func){
            func(proc, obj);
        }
    }
};

}

#endif
