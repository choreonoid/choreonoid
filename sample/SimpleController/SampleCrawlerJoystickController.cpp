/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace cnoid;

class SampleCrawlerJoystickController : public SimpleController
{ 
    Link* crawlerL;
    Link* crawlerR;
    double qRef[2];
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        std::ostream& os = io->os();

        crawlerL = io->body()->link("CRAWLER_TRACK_L");
        crawlerR = io->body()->link("CRAWLER_TRACK_R");

        if(!crawlerL || !crawlerR){
            os << "Crawlers are not found" << std::endl;
            return false;
        }

        io->enableOutput(crawlerL, JointVelocity);
        io->enableOutput(crawlerR, JointVelocity);

        for(int i=0; i < 2; i++){
            qRef[i] = 0;
        }

        if(!joystick.isReady()){
            os << "Joystick is not ready: " << joystick.errorMessage() << std::endl;
        }
        if(joystick.numAxes() < 5){
            os << "The number of the joystick axes is not sufficient for controlling the robot." << std::endl;
        }
        
        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(i);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each crawlers
        crawlerL->dq_target() = -2.0 * pos[1] + pos[0];
        crawlerR->dq_target() = -2.0 * pos[1] - pos[0];

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCrawlerJoystickController)
