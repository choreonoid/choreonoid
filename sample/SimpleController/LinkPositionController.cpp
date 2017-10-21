/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/SimpleController>
#include <cnoid/JoystickGamepad>

using namespace std;
using namespace cnoid;

class LinkPositionController : public SimpleController
{
    Link* link;
    double dt;
    Gamepad gamepad;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        const string& linkName = io->optionString();
        Body* body = io->body();
        link = body->link(linkName);
        if(!link){
            os << linkName << " not found." << endl;
            return false;
        }
        link->setActuationMode(Link::ActuationMode::LINK_POSITION);
        io->enableOutput(link);

        dt = io->timeStep();

        if(!gamepad.isReady()){
            os << "Joystick is not ready: " << gamepad.errorMessage() << endl;
        }

        return true;
    }

    virtual bool control() override
    {
        gamepad.readCurrentState();
        link->p().x() += gamepad.getStickLX() * dt;
        link->p().y() += gamepad.getStickLY() * dt;
        link->p().z() -= gamepad.getTriggerL2() * dt;
        link->p().z() += gamepad.getTriggerR2() * dt;

        Matrix3 mat;
        mat = AngleAxis(-1 * gamepad.getStickRY() * dt, Vector3::UnitX())
            * AngleAxis(     gamepad.getStickRX() * dt, Vector3::UnitZ());
        link->T().rotate(mat);

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LinkPositionController)
