#ifndef CNOID_UTIL_ARCHIVE_SESSION_H
#define CNOID_UTIL_ARCHIVE_SESSION_H

#include "Referenced.h"
#include "Signal.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Uuid;

/**
   \deprecated
*/
class CNOID_EXPORT ArchiveSession
{
public:
    ArchiveSession();
    virtual ~ArchiveSession();

    void initialize();
    
    bool addReference(const Uuid& uuid, Referenced* object, bool doUnreferenceImmediately = false);

    template<class ObjectType>
    void resolveReference(
        const Uuid& uuid,
        std::function<bool(ObjectType* object, bool isImmediate)> onResolved,
        std::function<bool()> onNotResolved = nullptr)
    {
        resolveReference_(
            uuid,
            [onResolved](Referenced* object, bool isImmediate){
                if(auto derived = dynamic_cast<ObjectType*>(object)){
                    return onResolved(derived, isImmediate);
                }
                return false;
            },
            onNotResolved,
            false);
    }

    template<class ObjectType>
    void resolveReferenceLater(
        const Uuid& uuid,
        std::function<bool(ObjectType* object)> onResolved,
        std::function<bool()> onNotResolved = nullptr)
    {
        resolveReference_(
            uuid,
            [onResolved](Referenced* object, bool isImmediate){
                if(auto derived = dynamic_cast<ObjectType*>(object)){
                    return onResolved(derived);
                }
                return false;
            },
            onNotResolved,
            false);
    }

    virtual void putWarning(const std::string& message);
    virtual void putError(const std::string& message);

    void resolvePendingReferences();
    bool finalize();
    SignalProxy<void()> sigSessionFinalized();

private:
    void resolveReference_(
        const Uuid& uuid,
        std::function<bool(Referenced* object, bool isImmediate)> onResolved,
        std::function<bool()> onNotResolved,
        bool doResolveLater
        );

    class Impl;
    Impl* impl;
};

}

#endif
