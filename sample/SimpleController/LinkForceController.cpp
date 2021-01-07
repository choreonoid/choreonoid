#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class LinkForceController : public SimpleController
{
    Link* link;
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

        link->setActuationMode(LinkExtWrench);
        io->enableIO(link);

        if(!joystick.isReady()){
            os << "Joystick is not ready: " << joystick.errorMessage() << endl;
        }

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();

        const double k = 100.0;
        link->F_ext().setZero();
        Vector3 f;
        f  = link->R() * Vector3::UnitX() * (-joystick.getPosition(Joystick::L_STICK_V_AXIS) * k);
        f += link->R() * Vector3::UnitY() * (-joystick.getPosition(Joystick::L_STICK_H_AXIS) * k);
        f += link->R() * Vector3::UnitZ() * (-joystick.getPosition(Joystick::R_STICK_V_AXIS) * k);
        link->addExternalForceAtLocalPosition(f, Vector3(0.0, 0.0, 0.0));

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LinkForceController)
