/**
   Four wheel car controller
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class FourWheelCarJoystickController : public SimpleController
{ 
    Link* steering;
    Link* drive;
    Joystick joystick;
    double timeStep;
    double eold;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        
        Body* body = io->body();

        steering = body->link("STEERING_RIGHT");
        if(!steering){
            os << " The steering_right link is not found." << endl;
            return false;
        }
        steering->setActuationMode(Link::JointTorque);
        io->enableIO(steering);

        drive = body->link("REAR_WHEEL");
        if(!drive){
            os << "The rear_wheel link is not found." << endl;
            return false;
        }
        drive->setActuationMode(Link::JointTorque);
        io->enableInput(drive, JointVelocity);
        io->enableOutput(drive);
        
        if(!joystick.isReady()){
            os << "Joystick is not ready: " << joystick.errorMessage() << endl;
        }

        timeStep = io->timeStep();
        eold = 0;

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        
        static const double DRIVE_GAIN = 1.0;
        static const double STEERING_P_GAIN = 3.0;
        static const double STEERING_D_GAIN = 1.0;
        static const double VEL_MAX = 10;

        double pos[2];
        pos[0] = joystick.getPosition(2);
        pos[1] = joystick.getPosition(1);

        double steer_ref = - pos[0] * 0.8;
        double q= steering->q();
        double err = steer_ref - q;
        double derr = (err - eold) / timeStep;

        steering->u() = err * STEERING_P_GAIN + derr * STEERING_D_GAIN;

        double dq = drive->dq();
        double drive_ref = -pos[1] * VEL_MAX;
        drive->u() = (drive_ref - dq) * DRIVE_GAIN;

        eold = err;
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FourWheelCarJoystickController)
