#ifndef CNOID_UTIL_CLONE_MAP_H
#define CNOID_UTIL_CLONE_MAP_H

#include "Referenced.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class ClonableReferenced;

/**
   \todo Support std::shared_ptr as a pointer to clone objects
*/
class CNOID_EXPORT CloneMap
{
public:
    typedef std::function<Referenced*(const Referenced* org)> CloneFunction;

    CloneMap();
    CloneMap(const CloneFunction& cloneFunction);
    CloneMap(const CloneMap& org);
    virtual ~CloneMap();

    void clear();

    void setClone(const Referenced* org, Referenced* clone);

    template<class ObjectType>
    ObjectType* findClone(const ObjectType* org){
        return static_cast<ObjectType*>(findClone_(org));
    }

    template<class ObjectType>
    static ObjectType* findClone(const ObjectType* org, CloneMap* cloneMap){
        return cloneMap ? cloneMap->findClone(org) : nullptr;
    }

    template<class ObjectType>
    ObjectType* findClone(ref_ptr<ObjectType> org){
        return findClone<ObjectType>(org.get());
    }

    template<class ObjectType>
    ObjectType* getClone(const ObjectType* org){
        return static_cast<ObjectType*>(findOrCreateClone_(org));
    }

    template<class ObjectType>
    static ObjectType* getClone(const ObjectType* org, CloneMap* cloneMap){
        return cloneMap ? cloneMap->getClone(org) : static_cast<ObjectType*>(getClone_(org));
    }

    template<class ObjectType>
    ObjectType* getClone(ref_ptr<ObjectType> org){
        return getClone<ObjectType>(org.get());
    }

    template<class ObjectType>
    static ObjectType* getClone(ref_ptr<ObjectType> org, CloneMap* cloneMap){
        if(cloneMap){
            return cloneMap->getClone(org);
        } else {
            return static_cast<ObjectType*>(getClone_(org));
        }
    }

    /*
    template<class ObjectType>
    ObjectType* getClone(ref_ptr<const ObjectType> org){
        return getClone<ObjectType>(org.get());
    }
    */

    template<class ObjectType>
    ObjectType* getClone(const ObjectType* org, std::function<ObjectType*(const ObjectType* org)> cloneFunction){
        return static_cast<ObjectType*>(
            findOrCreateClone_(
                org,
                [cloneFunction](const Referenced* org) -> Referenced* {
                    return cloneFunction(static_cast<const ObjectType*>(org));
                }));
    }

    template<class ObjectType>
    ObjectType* getClone(ref_ptr<ObjectType> org, std::function<ObjectType*(const ObjectType* org)> cloneFunction){
        return getClone(org.get(), cloneFunction);
    }

    /*
    template<class ObjectType>
    ObjectType* getClone(ref_ptr<const ObjectType> org, const CloneFunction& cloneFunction){
        return getClone(org.get(), cloneFunction);
    }
    */

    template<class ObjectType>
    ObjectType* findCloneOrReplaceLater(
        const ObjectType* org, std::function<void(ObjectType* clone)> replaceFunction){
        return static_cast<ObjectType*>(
            findCloneOrReplaceLater_(
                org,
                [replaceFunction](Referenced* clone){
                    replaceFunction(static_cast<ObjectType*>(clone)); }));
    }

    void replacePendingObjects();

    void setOriginalAsClone(const Referenced* org);

    class FlagId {
        int id_;
    public:
        FlagId(const char* name) : id_(getFlagId(name)) { }
        operator int() const { return id_; }
    };

    bool flag(int id) const { return flags & (1u << id); }
    void setFlag(int id, bool on) { 
        if(on) flags |= (1u << id);
        else flags &= ~(1u << id);
    }

private:
    class Impl;
    Impl* impl;
    uint32_t flags = 0;

    Referenced* findClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const ClonableReferenced* org);
    Referenced* findOrCreateClone_(const Referenced* org, const CloneFunction& cloneFunction);
    Referenced* findCloneOrReplaceLater_(
        const Referenced* org, std::function<void(Referenced* clone)> replaceFunction);

    static Referenced* getClone_(const ClonableReferenced* org);

    static int getFlagId(const char* name);
};

}

#endif
