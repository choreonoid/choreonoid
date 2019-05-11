#ifndef CNOID_BODY_BODY_HANDLER_H
#define CNOID_BODY_BODY_HANDLER_H

#include <cnoid/Referenced>
#include <cnoid/Config>
#include <iosfwd>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class BodyHandler;
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CNOID_BODY_HANDLER_EXPORT __declspec(dllexport)
#elif __GNUC__ >= 4
#define CNOID_BODY_HANDLER_EXPORT __attribute__ ((visibility("default")))
#else 
#define CNOID_BODY_HANDLER_EXPORT
#endif

extern "C" CNOID_BODY_HANDLER_EXPORT cnoid::BodyHandler* createCnoidBodyHandler(std::ostream& os);

namespace cnoid {

class Body;

/**
   This class is inherited by any body handler class.
   The virtual inheritance must be used for this class.
*/
class CNOID_EXPORT BodyHandler : public Referenced
{
public:
    static BodyHandler* load(const std::string& filename);

    Body* body() { return body_; }

    virtual bool initialize(Body* body, std::ostream& os) = 0;

private:
    Body* body_;

    static bool checkVersion(int version, int internalVersion, std::ostream& os);
    friend BodyHandler* ::createCnoidBodyHandler(std::ostream& os);
};

typedef ref_ptr<BodyHandler> BodyHandlerPtr;

}

#define CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(HandlerClassName)  \
    extern "C" CNOID_BODY_HANDLER_EXPORT cnoid::BodyHandler* createCnoidBodyHandler(std::ostream& os) \
    {                                                                   \
        if(BodyHandler::checkVersion(CNOID_VERSION, CNOID_INTERNAL_VERSION, os)){ \
            return new HandlerClassName(); \
        } \
        return nullptr; \
    }

#endif
