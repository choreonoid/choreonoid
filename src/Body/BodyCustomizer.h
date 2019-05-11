#ifndef CNOID_BODY_BODY_CUSTOMIZER_H
#define CNOID_BODY_BODY_CUSTOMIZER_H

#include <cnoid/Referenced>
#include <cnoid/Config>
#include <iosfwd>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class InverseKinematics;

class CNOID_EXPORT BodyCustomizer : public Referenced
{
public:
    Body* body() { return body_; }

    virtual bool initialize(Body* body, std::ostream& os);

    virtual std::shared_ptr<InverseKinematics> getCustomIk(Link* baseLink, Link* endLink);

private:
    Body* body_;

    static bool checkVersion(int version, int internalVersion, std::ostream& os);
    friend BodyCustomizer* createBodyCustomizer(std::ostream& os);
};

}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CNOID_BODY_CUSTOMIZER_EXPORT __declspec(dllexport)
#elif __GNUC__ >= 4
#define CNOID_BODY_CUSTOMIZER_EXPORT __attribute__ ((visibility("default")))
#else 
#define CNOID_BODY_CUSTOMIZER_EXPORT
#endif

extern "C" CNOID_BODY_CUSTOMIZER_EXPORT cnoid::BodyCustomizer* createBodyCustomizer();

#define CNOID_IMPLEMENT_BODY_CUSTOMIZER_FACTORY(CustomizerClassName)  \
    extern "C" CNOID_BODY_CUSTOMIZER_EXPORT cnoid::BodyCustomizer* createBodyCustomizer(std::ostream& os) \
    {                                                                   \
        if(BodyCustomizer::checkVersion(CNOID_VERSION, CNOID_INTERNAL_VERSION, os)){ \
            return new CustomizerClassName(); \
        } \
        return nullptr; \
    }

#endif
