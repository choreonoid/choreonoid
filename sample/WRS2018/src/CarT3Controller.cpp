/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Device>
#include <cnoid/ForceSensor>
#include "SpreaderController.h"

using namespace std;
using namespace cnoid;

class CarT3Controller : public SimpleController
{
    struct SpreaderTarget {
        ForceSensorPtr forceSensor;
        DevicePtr breakableJoint;
        double time;
        string sensorName;
        string jointName;
        SpreaderTarget(const char* sensorName, const char* jointName)
            : time(0.0), sensorName(sensorName), jointName(jointName) { }
    };
    vector<SpreaderTarget> targets;
    SimpleControllerIO* io;
    double dt;
    SpreaderController* spreaderController;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        dt = io->timeStep();
        
        targets = {
            { "FR_DOOR_HINGE_FORCE_SENSOR", "FR_DOOR_HINGE_CONSTRAINT" },
            { "FR_DOOR_LOCK_FORCE_SENSOR",  "FR_DOOR_LOCK_CONSTRAINT"  }
        };

        auto body = io->body();
        auto iter = targets.begin();
        while(iter != targets.end()){
            auto& target = *iter;
            target.forceSensor = body->findDevice<ForceSensor>(target.sensorName);
            target.breakableJoint = body->findDevice(target.jointName);
            if(target.forceSensor && target.breakableJoint){
                io->enableInput(target.forceSensor);
                ++iter;
            } else {
                iter = targets.erase(iter);
            }
        }

        io->os() << io->controllerName() << ": ";
        if(targets.empty()){
            io->os() << "No spreader target was found." << endl;
        } else {
            io->os() << targets.size() << " spreader targets has been found." << endl;
        }

        return !targets.empty();
    }

    virtual bool start() override
    {
        spreaderController = SpreaderController::instance();
        return true;
    }

    virtual bool control() override
    {
        //io->os() << "control" << endl;
        bool doRequestToSpread = false;
        
        auto iter = targets.begin();
        while(iter != targets.end()){
            auto& target = *iter;
            double fy = target.forceSensor->f().y();
            if(fy > 100.0){
                doRequestToSpread = true;
                if(target.breakableJoint->on()){
                    target.time += dt;
                    //io->os() << target.forceSensor->name() << "'s time counter is " << target.time << endl;
                    if(target.time > 2.0){
                        target.breakableJoint->on(false);
                        target.breakableJoint->notifyStateChange();
                    }
                }
            }
            ++iter;
        }

        if(spreaderController){
            spreaderController->requestToSpread(doRequestToSpread);
        }
        
        return targets.empty();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CarT3Controller)
