#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace cnoid;

class TrackController : public SimpleController
{
    Link* trackL;
    Link* trackR;
    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        trackL = io->body()->link("TRACK_L");
        trackR = io->body()->link("TRACK_R");

        trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        
        return true;
    }

    virtual bool control() override
    {
        static const int axisID[] = { 0, 1 };
        
        joystick.readCurrentState();

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(axisID[i]);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }

        trackL->dq_target() = -2.0 * pos[1] + pos[0];
        trackR->dq_target() = -2.0 * pos[1] - pos[0];

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TrackController)
