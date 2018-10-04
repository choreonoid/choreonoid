/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class LinkPositionController : public SimpleController
{
    Link* link;
    double dt;
    Joystick joystick;

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

        if(!joystick.isReady()){
            os << "Joystick is not ready: " << joystick.errorMessage() << endl;
        }

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();

        if(joystick.getButtonState(Joystick::L_BUTTON) || joystick.getButtonState(Joystick::R_BUTTON)){
            Matrix3 R;
            R = AngleAxis(-joystick.getPosition(Joystick::R_STICK_H_AXIS) * dt, Vector3::UnitZ())
                * AngleAxis(-joystick.getPosition(Joystick::L_STICK_V_AXIS) * dt, Vector3::UnitY())
                * AngleAxis(joystick.getPosition(Joystick::L_STICK_H_AXIS) * dt, Vector3::UnitX());
            link->R() = link->R() * R;
        } else {
            link->p() += link->R() * Vector3::UnitX() * (-joystick.getPosition(Joystick::L_STICK_V_AXIS) * dt);
            link->p() += link->R() * Vector3::UnitY() * (-joystick.getPosition(Joystick::L_STICK_H_AXIS) * dt);
            link->p() += link->R() * Vector3::UnitZ() * (-joystick.getPosition(Joystick::R_STICK_V_AXIS) * dt);
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LinkPositionController)
