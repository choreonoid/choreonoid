#ifndef CNOID_UTIL_CLONE_MAP_H
#define CNOID_UTIL_CLONE_MAP_H

#include "Referenced.h"
#include <functional>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CloneableReferenced;

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
    ObjectType* getClone(const ObjectType* org){
        return static_cast<ObjectType*>(findOrCreateClone_(org));
    }

    template<class ObjectType>
    ObjectType* getClone(ref_ptr<ObjectType> org){
        return getClone<ObjectType>(org.get());
    }

    /*
    template<class ObjectType>
    ObjectType* getClone(ref_ptr<const ObjectType> org){
        return getClone<ObjectType>(org.get());
    }
    */

    template<class ObjectType>
    ObjectType* getClone(const ObjectType* org, const CloneFunction& cloneFunction){
        return static_cast<ObjectType*>(findOrCreateClone_(org, cloneFunction));
    }

    template<class ObjectType>
    ObjectType* getClone(ref_ptr<ObjectType> org, const CloneFunction& cloneFunction){
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

    bool flag(int id) const { return flags[id]; }
    void setFlag(int id, bool on) { flags[id] = on; }

private:
    class Impl;
    Impl* impl;
    std::vector<bool> flags;

    Referenced* findClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const CloneableReferenced* org);
    Referenced* findOrCreateClone_(const Referenced* org, const CloneFunction& cloneFunction);
    Referenced* findCloneOrReplaceLater_(
        const Referenced* org, std::function<void(Referenced* clone)> replaceFunction);

    static int getFlagId(const char* name);
};

}

#endif
