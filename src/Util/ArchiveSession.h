#ifndef CNOID_UTIL_ARCHIVE_SESSION_H
#define CNOID_UTIL_ARCHIVE_SESSION_H

#include "Referenced.h"
#include "Signal.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Uuid;

class CNOID_EXPORT ArchiveSession
{
public:
    ArchiveSession();
    virtual ~ArchiveSession();

    void initialize();
    
    bool addReference(const Uuid& uuid, Referenced* object, bool doWarnUuidDuplication = true);

    template<class ObjectType>
    void dereferenceLater(
        const Uuid& uuid,
        std::function<bool(ObjectType* object)> onObjectFound,
        std::function<bool()> onObjectNotFound = nullptr)
    {
        dereferenceLater_(
            uuid,
            [onObjectFound](Referenced* object){
                if(auto derived = dynamic_cast<ObjectType*>(object)){
                    return onObjectFound(derived);
                }
                return false;
            },
            onObjectNotFound);
    }

    virtual void putWarning(const std::string& message);
        
    bool finalize();
    SignalProxy<void()> sigSessionFinalized();

private:
    void dereferenceLater_(
        const Uuid& uuid,
        std::function<bool(Referenced* object)> onObjectFound,
        std::function<bool()> onObjectNotFound);

    class Impl;
    Impl* impl;
};

}

#endif
