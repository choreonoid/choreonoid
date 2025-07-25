#include "CloneMap.h"
#include "ClonableReferenced.h"
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <stdexcept>

using namespace std;
using namespace cnoid;

namespace {

unordered_map<string, int> flagNameToIdMap;
int idCounter = 0;
mutex flagMapMutex;

}

namespace cnoid {

class CloneMap::Impl
{
public:
    unordered_map<ReferencedPtr, ReferencedPtr> orgToCloneMap;

    CloneFunction cloneFunction;

    struct ReplaceFunctionInfo
    {
        ReferencedPtr object;
        function<void(Referenced* clone)> replaceFunction;
        ReplaceFunctionInfo(Referenced* object, function<void(Referenced* clone)> replaceFunction)
            : object(object), replaceFunction(replaceFunction)
        { }
    };

    vector<ReplaceFunctionInfo> replaceFunctions;
    
    Impl();
    Impl(const CloneFunction& cloneFunction);
    Impl(const Impl& org);
};

}


CloneMap::CloneMap()
{
    impl = new Impl;
}


CloneMap::Impl::Impl()
{

}


CloneMap::CloneMap(const CloneFunction& cloneFunction)
{
    impl = new Impl(cloneFunction);
}


CloneMap::Impl::Impl(const CloneFunction& cloneFunction)
    : cloneFunction(cloneFunction)
{

}


CloneMap::CloneMap(const CloneMap& org)
    : flags(org.flags)
{
    impl = new Impl(*org.impl);
}


CloneMap::Impl::Impl(const Impl& org)
    : orgToCloneMap(org.orgToCloneMap)
      //cloneFunction(org.cloneFunction)
{

}


CloneMap::~CloneMap()
{
    delete impl;
}


void CloneMap::clear()
{
    impl->orgToCloneMap.clear();
    impl->replaceFunctions.clear();
}


void CloneMap::setClone(const Referenced* org, Referenced* clone)
{
    impl->orgToCloneMap[const_cast<Referenced*>(org)] = clone;
}


Referenced* CloneMap::findClone_(const Referenced* org)
{
    auto iter = impl->orgToCloneMap.find(const_cast<Referenced*>(org));
    if(iter != impl->orgToCloneMap.end()){
        return iter->second;
    }
    return nullptr;
}


Referenced* CloneMap::findOrCreateClone_(const Referenced* org)
{
    if(!org){
        return nullptr;
    }
    ReferencedPtr& clone = impl->orgToCloneMap[const_cast<Referenced*>(org)];
    if(!clone){
        clone = impl->cloneFunction(org);
    }
    return clone;
}


Referenced* CloneMap::findOrCreateClone_(const ClonableReferenced* org)
{
    if(!org){
        return nullptr;
    }
    ReferencedPtr& clone = impl->orgToCloneMap[const_cast<ClonableReferenced*>(org)];
    if(!clone){
        clone = org->doClone(this);
    }
    return clone;
}


Referenced* CloneMap::findOrCreateClone_
(const Referenced* org, const CloneFunction& cloneFunction)
{
    if(!org){
        return nullptr;
    }
    ReferencedPtr& clone = impl->orgToCloneMap[const_cast<Referenced*>(org)];
    if(!clone){
        clone = cloneFunction(org);
    }
    return clone;
}


Referenced* CloneMap::findCloneOrReplaceLater_
(const Referenced* org, std::function<void(Referenced* clone)> replaceFunction)
{
    if(!org){
        return nullptr;
    }
    auto clone = findClone_(org);
    if(!clone){
        impl->replaceFunctions.push_back(
            Impl::ReplaceFunctionInfo(const_cast<Referenced*>(org), replaceFunction));
    }
    return clone;
}


void CloneMap::replacePendingObjects()
{
    auto& functions = impl->replaceFunctions;
    auto it = functions.begin();
    while(it != functions.end()){
        auto& info = *it;
        auto clone = findClone_(info.object);
        if(clone){
            info.replaceFunction(clone);
            it = functions.erase(it);
        } else {
            ++it;
        }
    }
}


void CloneMap::setOriginalAsClone(const Referenced* org)
{
    impl->orgToCloneMap[const_cast<Referenced*>(org)] = const_cast<Referenced*>(org);
}


Referenced* CloneMap::getClone_(const ClonableReferenced* org)
{
    return org->doClone(nullptr);
}


int CloneMap::getFlagId(const char* name)
{
    lock_guard<mutex> guard(flagMapMutex);

    auto iter = flagNameToIdMap.find(name);
    if(iter != flagNameToIdMap.end()){
        return iter->second;
    }
    if(idCounter >= 32){
        throw std::runtime_error("CloneMap: Maximum number of flags (32) exceeded");
    }
    int id = idCounter;
    flagNameToIdMap[name] = idCounter++;
    return id;
}
