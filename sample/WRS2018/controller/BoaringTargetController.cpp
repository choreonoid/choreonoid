/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/ForceSensor>
#include "RockDrillController.h"

using namespace std;
using namespace cnoid;

class BoaringTargetController : public SimpleController
{
    SimpleControllerIO* io;
    double timeStep;
    Link* boaringBase;
    ForceSensor* forceSensor;
    Device* breakableJoint;
    RockDrillController* drillController;
    bool drillPower;
    double timeToPowerOff;
    double timeToSlideDown;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;

        auto body = io->body();
        boaringBase = body->link("BOARING_BASE");
        forceSensor = body->findDevice<ForceSensor>("FORCE_SENSOR");
        breakableJoint = body->findDevice("BOARING_CONSTRAINT");

        if(!boaringBase || !forceSensor || !breakableJoint){
            io->os() << "Objects required for boaring are not found." << endl;
            return false;
        }

        boaringBase->setActuationMode(Link::JOINT_DISPLACEMENT);
        io->enableIO(boaringBase);
        io->enableInput(forceSensor);
        breakableJoint->on(true);

        timeStep = io->timeStep();

        return true;
    }

    virtual bool start() override
    {
        drillController = RockDrillController::instance();
        if(!drillController){
            io->os() << "The controller of the rock drill is not found." << endl;
            return false;
        }
        drillPower = false;

        timeToSlideDown = 0.0;
        
        return true;
    }

    virtual bool control() override
    {
        double fz = forceSensor->f().z();
        //io->os() << "fz: " << fz << endl;

        if(fz > -300.0){
            if(drillPower){
                if(timeToPowerOff < 5.0){
                    timeToPowerOff += timeStep;
                } else {
                    drillController->power(false);
                    drillPower = false;
                }
            }
        } else if(breakableJoint->on()){

            timeToPowerOff = 0.0;
            if(!drillPower){
                drillController->power(true);
                drillPower = true;
            }

            if(fz < -500.0){
                timeToSlideDown += timeStep;
                //io->os() << "timeToSlideDown: " << timeToSlideDown << endl;
                if(timeToSlideDown > 0.3){
                    boaringBase->q_target() -= 0.001;
                    timeToSlideDown = 0.0;
                    if(boaringBase->q_target() <= -0.05){
                        breakableJoint->on(false);
                        breakableJoint->notifyStateChange();
                    }
                }
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(BoaringTargetController)
