#include <cnoid/SimpleController>
#include <cnoid/Config>

namespace cnoid {

class CNOID_GENERAL_EXPORT RockDrillController : public SimpleController
{
    SimpleControllerIO* io;
    Link* pusher;
    double time;
    double dt;
    bool isPowerOn;
    bool requestedPowerState;
    
public:
    static RockDrillController* instance();

    RockDrillController();
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
    void power(bool on);
};

}
