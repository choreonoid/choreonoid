#include "ArchiveSession.h"
#include "Uuid.h"
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
    std::function<bool(Referenced* object)> onObjectFound;
    std::function<bool()> onObjectNotFound;

    CallbackSet(
        const std::function<bool(Referenced* object)>& onObjectFound,
        const std::function<bool()>& onObjectNotFound)
        : onObjectFound(onObjectFound), onObjectNotFound(onObjectNotFound)
    { }
};

class RefInfo
{
public:
    ReferencedPtr object;
    vector<CallbackSet> callbacks;

    RefInfo() { }
    RefInfo(Referenced* object) : object(object) { }
};

typedef unordered_map<Uuid, RefInfo> ObjectMap;

}

namespace cnoid {

class ArchiveSession::Impl
{
public:
    ObjectMap objectMap;
    Signal<void()> sigSessionFinalized;
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


bool ArchiveSession::addReference(const Uuid& uuid, Referenced* object, bool doWarnOverlap)
{
    auto inserted = impl->objectMap.emplace(uuid, object);
    if(!inserted.second){
        RefInfo& info = inserted.first->second;
        if(!info.object){
            info.object = object;
        } else if(info.object != object){
            putWarning(format(_("UUID \"{0}\" is a duplicate of the existing ID.\n"), uuid.toString()));
            return false;
        }
    }
    return true;
}


void ArchiveSession::dereferenceLater_
(const Uuid& uuid,
 std::function<bool(Referenced* object)> onObjectFound,
 std::function<bool()> onObjectNotFound)
{
    impl->objectMap[uuid].callbacks.emplace_back(onObjectFound, onObjectNotFound);
}


void ArchiveSession::putWarning(const std::string& message)
{
    std::cerr << message << std::flush;
}


bool ArchiveSession::finalize()
{
    bool isComplete = true;
    for(auto& kv : impl->objectMap){
        RefInfo& info = kv.second;
        for(auto& callback : info.callbacks){
            if(info.object){
                if(!callback.onObjectFound(info.object)){
                    isComplete = false;
                }
            } else {
                if(!callback.onObjectNotFound()){
                    isComplete = false;
                }
            }
        }
    }
    impl->sigSessionFinalized();

    return isComplete;
}


SignalProxy<void()> ArchiveSession::sigSessionFinalized()
{
    return impl->sigSessionFinalized;
}
