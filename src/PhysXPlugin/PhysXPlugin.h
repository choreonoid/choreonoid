#ifndef CNOID_PHYSX_PLUGIN_PHYSX_PLUGIN_H
#define CNOID_PHYSX_PLUGIN_PHYSX_PLUGIN_H

#include <cnoid/Plugin>
#include <foundation/PxFoundation.h>
#include <memory>

namespace cnoid {

class PhysXPlugin : public Plugin
{
public:
    PhysXPlugin();
    virtual ~PhysXPlugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;

    static physx::PxFoundation* foundation();
    static void setErrorOutputEnabled(bool on);

private:
    physx::PxFoundation* foundation_;
    std::unique_ptr<physx::PxAllocatorCallback> allocatorCallback;
    std::unique_ptr<physx::PxErrorCallback> errorCallback;
    //physx::PxProfileZoneManager* profileZoneManager;
};

}

#endif
