#include "ArchiveSession.h"
#include "Uuid.h"
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class CallbackSet
{
public:
    std::function<bool(Referenced* object, bool isImmediate)> onResolved;
    std::function<bool()> onNotResolved;

    CallbackSet(
        const std::function<bool(Referenced* object, bool isImmediate)>& onResolved,
        const std::function<bool()>& onNotResolved)
        : onResolved(onResolved), onNotResolved(onNotResolved)
    { }
};

class RefInfo
{
public:
    ReferencedPtr object;
    vector<CallbackSet> callbacks;
    bool doResolveLater;
    bool hasFailure;

    RefInfo() : hasFailure(false) { }
    RefInfo(Referenced* object) : object(object), hasFailure(false) { }
};

typedef unordered_map<Uuid, RefInfo> ObjectMap;

}

namespace cnoid {

class ArchiveSession::Impl
{
public:
    ObjectMap objectMap;
    Signal<void()> sigSessionFinalized;

    bool resolvePendingReferences(bool doFinalize);    
};

}


ArchiveSession::ArchiveSession()
{
    impl = new Impl;
}


ArchiveSession::~ArchiveSession()
{
    delete impl;
}


void ArchiveSession::initialize()
{
    impl->objectMap.clear();
}


bool ArchiveSession::addReference(const Uuid& uuid, Referenced* object, bool doUnreferenceImmediately)
{
    auto inserted = impl->objectMap.emplace(uuid, object);
    RefInfo& info = inserted.first->second;
    if(inserted.second){
        if(doUnreferenceImmediately && !info.doResolveLater){
            auto& callbacks = info.callbacks;
            auto it = callbacks.begin();
            while(it != callbacks.end()){
                auto& callbackSet = *it;
                if(callbackSet.onResolved){
                    if(!callbackSet.onResolved(object, false)){
                        info.hasFailure = true;
                    }
                    it = callbacks.erase(it);
                } else {
                    ++it;
                }
            }
        }
    } else {
        if(!info.object){
            info.object = object;
        } else if(info.object != object){
            putWarning(format(_("UUID \"{0}\" is a duplicate of the existing ID.\n"), uuid.toString()));
            return false;
        }
    }
    return true;
}


void ArchiveSession::resolveReference_
(const Uuid& uuid,
 std::function<bool(Referenced* object, bool isImmediate)> onResolved,
 std::function<bool()> onNotResolved,
 bool doResolveLater)
{
    bool processed = false;

    if(!doResolveLater){
        auto p = impl->objectMap.find(uuid);
        if(p != impl->objectMap.end()){
            RefInfo& info = p->second;
            if(info.object){
                if(!onResolved(info.object, true)){
                    info.hasFailure = true;
                }
                processed = true;
            }
        }
    }
    
    if(!processed){
        RefInfo& info = impl->objectMap[uuid];
        info.callbacks.emplace_back(onResolved, onNotResolved);
        info.doResolveLater = doResolveLater;
    }
}


void ArchiveSession::putWarning(const std::string& message)
{
    std::cerr << message << std::flush;
}


void ArchiveSession::putError(const std::string& message)
{
    std::cerr << message << std::flush;
}


void ArchiveSession::resolvePendingReferences()
{
    impl->resolvePendingReferences(false);
}


bool ArchiveSession::Impl::resolvePendingReferences(bool doFinalize)
{
    bool hasFailure = false;

    for(auto& kv : objectMap){
        RefInfo& info = kv.second;
        if(info.object){
            auto& callbacks = info.callbacks;
            auto it = callbacks.begin();
            while(it != callbacks.end()){
                auto& callbackSet = *it;
                if(!callbackSet.onResolved){
                    ++it;
                } else {
                    if(!callbackSet.onResolved(info.object, false)){
                        info.hasFailure = true;
                    }
                    if(!doFinalize){
                        it = callbacks.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        } else if(doFinalize){
            for(auto& callbackSet : info.callbacks){
                if(callbackSet.onNotResolved){
                    if(!callbackSet.onNotResolved()){
                        info.hasFailure = true;
                    }
                }
            }
        }
        if(info.hasFailure){
            hasFailure = true;
        }
    }

    return !hasFailure;
}


bool ArchiveSession::finalize()
{
    bool isComplete = impl->resolvePendingReferences(true);

    impl->sigSessionFinalized();

    impl->objectMap.clear();

    return isComplete;
}


SignalProxy<void()> ArchiveSession::sigSessionFinalized()
{
    return impl->sigSessionFinalized;
}
