#ifndef CNOID_UTIL_CLONE_MAP_H
#define CNOID_UTIL_CLONE_MAP_H

#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Referenced;
class CloneMappableReferenced;

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
    ObjectType* getClone(const ObjectType* org, const CloneFunction& cloneFunction){
        return static_cast<ObjectType*>(findOrCreateClone_(org, cloneFunction));
    }
    
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

private:
    class Impl;
    Impl* impl;

    Referenced* findClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const Referenced* org);
    Referenced* findOrCreateClone_(const CloneMappableReferenced* org);
    Referenced* findOrCreateClone_(const Referenced* org, const CloneFunction& cloneFunction);
    Referenced* findCloneOrReplaceLater_(
        const Referenced* org, std::function<void(Referenced* clone)> replaceFunction);
};

}

#endif
