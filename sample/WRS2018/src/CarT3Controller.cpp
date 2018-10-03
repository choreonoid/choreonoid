/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Device>
#include <cnoid/ForceSensor>

#include <iostream>

using namespace std;
using namespace cnoid;

class CarT3Controller : public SimpleController
{
    struct SpreaderTarget {
        ForceSensorPtr forceSensor;
        DevicePtr breakableJoint;
        double impulse;
        string sensorName;
        string jointName;
        SpreaderTarget(const char* sensorName, const char* jointName)
            : impulse(0.0), sensorName(sensorName), jointName(jointName) { }
    };
    vector<SpreaderTarget> targets;
    SimpleControllerIO* io;
    double dt;
    
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
            cout << "target.breakableJoint: " << target.breakableJoint << endl;
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

    virtual bool control() override
    {
        auto iter = targets.begin();
        while(iter != targets.end()){
            auto& target = *iter;
            if(!target.breakableJoint->on()){
                iter = targets.erase(iter);
            } else {
                double fy = target.forceSensor->f().y();
                if(fy > 100.0){
                    target.impulse += fy * dt;
                    io->os() << target.forceSensor->name() << "'s impulse is " << target.impulse << endl;
                    if(target.impulse > 300.0){
                        target.breakableJoint->on(false);
                        target.breakableJoint->notifyStateChange();
                    }
                }
                ++iter;
            }
        }
        return targets.empty();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CarT3Controller)
