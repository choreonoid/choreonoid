/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Device>
#include <cnoid/ForceSensor>
#include <cnoid/MarkerDevice>
#include "SpreaderController.h"

using namespace std;
using namespace cnoid;

class CarT3Controller : public SimpleController
{
    struct SpreaderTarget {
        Link* link;
        ForceSensorPtr forceSensor;
        DevicePtr breakableJoint;
        double timeToBreak;
        string sensorName;
        string jointName;
        SpreaderTarget(const char* sensorName, const char* jointName)
            : sensorName(sensorName), jointName(jointName) {
            timeToBreak = 0.0;
        }
    };
    vector<SpreaderTarget> targets;
    SimpleControllerIO* io;
    double timeStep;
    SpreaderController* spreaderController;
    MarkerDevice* marker;
    int markerBlinkIteration;
    double markerBlinkTimer;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        timeStep = io->timeStep();
        
        targets = {
            { "FR_DOOR_UPPER_HINGE_FORCE_SENSOR", "FR_DOOR_UPPER_HINGE_CONSTRAINT" },
            { "FR_DOOR_LOWER_HINGE_FORCE_SENSOR", "FR_DOOR_LOWER_HINGE_CONSTRAINT" },
            { "FR_DOOR_LOCK_FORCE_SENSOR",  "FR_DOOR_LOCK_CONSTRAINT"  }
        };

        auto body = io->body();
        auto iter = targets.begin();
        while(iter != targets.end()){
            auto& target = *iter;
            target.forceSensor = body->findDevice<ForceSensor>(target.sensorName);
            target.breakableJoint = body->findDevice(target.jointName);
            if(target.forceSensor && target.breakableJoint){
                target.link = target.forceSensor->link();
                io->enableInput(target.forceSensor);
                target.breakableJoint->on(true);
                target.breakableJoint->notifyStateChange();
                ++iter;
            } else {
                iter = targets.erase(iter);
            }
        }

        io->os() << io->controllerName() << ": ";
        if(targets.empty()){
            io->os() << "No spreader target was found." << endl;
            return false;
        }
        io->os() << targets.size() << " spreader targets have been found." << endl;

        marker = body->findDevice<MarkerDevice>("DestructionMarker");
        if(!marker){
            io->os() << "DestructionMarker was not found." << endl;
            return false;
        }
        io->enableInput(marker);
        marker->on(false);
        marker->setMarkerSize(0.24);
        marker->notifyStateChange();
        markerBlinkIteration = 0;

        return !targets.empty();
    }

    virtual bool start() override
    {
        spreaderController = SpreaderController::instance();
        return true;
    }

    virtual bool control() override
    {
        bool doRequestToSpread = false;
        
        for(auto& target : targets){
            double fy = target.forceSensor->f().y();
            if(fy > 100.0){
                doRequestToSpread = true;
                if(target.breakableJoint->on()){
                    target.timeToBreak += timeStep;
                    if(target.timeToBreak > 2.0){
                        target.breakableJoint->on(false);
                        target.breakableJoint->notifyStateChange();
                        markerBlinkIteration = 7;
                        markerBlinkTimer = 0.0;
                        marker->setOffsetTranslation(target.link->offsetTranslation());
                        marker->on(true);
                        marker->notifyStateChange();
                    }
                }
            }
        }

        if(spreaderController){
            spreaderController->requestToSpread(doRequestToSpread);
        }
        
        if(markerBlinkIteration > 0){
            markerBlinkTimer += timeStep;
            if(markerBlinkTimer >= 0.12){
                markerBlinkTimer = 0.0;
                --markerBlinkIteration;
                if(markerBlinkIteration == 0){
                    marker->on(false);
                } else {
                    marker->on(!marker->on());
                }
                marker->notifyStateChange();
            }
        }
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CarT3Controller)
