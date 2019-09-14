#ifndef CNOID_UTIL_CLONE_MAP_H
#define CNOID_UTIL_CLONE_MAP_H

#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Referenced;

class CNOID_EXPORT CloneMap
{
public:
    typedef std::function<Referenced*(const Referenced* org)> CloneFunction;

    CloneMap(CloneFunction cloneFunction);
    CloneMap(const CloneMap& org);
    virtual ~CloneMap();

    void clear();

    template<class ObjectType>
    ObjectType* findClone(const ObjectType* org){
        return static_cast<ObjectType*>(findClone(org));
    }

    template<class ObjectType>
    ObjectType* getClone(const ObjectType* org){
        return static_cast<ObjectType*>(findOrCreateClone(org));
    }

    template<class ObjectType>
    ObjectType* findCloneOrReplaceLater(
        const ObjectType* org, std::function<void(Referenced* clone)> replaceFunction)
    {
        return static_cast<ObjectType*>(findCloneOrReplaceLater(org, replaceFunction));
    }

    void replacePendingObjects();

    void setOriginalAsClone(const Referenced* org);

protected:
    Referenced* findClone(const Referenced* org);
    Referenced* findOrCreateClone(const Referenced* org);
    Referenced* findCloneOrReplaceLater(
        const Referenced* org, std::function<void(Referenced* clone)> replaceFunction);

private:
    class Impl;
    Impl* impl;
};

}

#endif
