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
    double k;
    Joystick joystick;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        auto linkName = io->optionString();
        if(linkName.empty()){
            link = io->body()->rootLink();
        } else {
            link = io->body()->link(linkName);
            if(!link){
                os << linkName << " not found." << endl;
                return false;
            }
        }
        
        link->setActuationMode(Link::LinkPosition);
        io->enableOutput(link);

        k = 0.8 * io->timeStep();

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
            R = AngleAxis(-joystick.getPosition(Joystick::R_STICK_H_AXIS) * k, Vector3::UnitZ())
                * AngleAxis(-joystick.getPosition(Joystick::L_STICK_V_AXIS) * k, Vector3::UnitY())
                * AngleAxis(joystick.getPosition(Joystick::L_STICK_H_AXIS) * k, Vector3::UnitX());
            link->R() = link->R() * R;
        } else {
            link->p() += link->R() * Vector3::UnitX() * (-joystick.getPosition(Joystick::L_STICK_V_AXIS) * k);
            link->p() += link->R() * Vector3::UnitY() * (-joystick.getPosition(Joystick::L_STICK_H_AXIS) * k);
            link->p() += link->R() * Vector3::UnitZ() * (-joystick.getPosition(Joystick::R_STICK_V_AXIS) * k);
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LinkPositionController)
