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
    bool useSprocketMode;
    Joystick joystick;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        std::ostream& os = io->os();

        useSprocketMode = false;
        for(auto& opt : io->options()){
            if(opt == "wheels"){
                useSprocketMode = true;
            }
        }

        if(useSprocketMode){
            crawlerL = io->body()->link("SPROCKET_L");
            crawlerR = io->body()->link("SPROCKET_R");
        } else {
            crawlerL = io->body()->link("CRAWLER_TRACK_L");
            crawlerR = io->body()->link("CRAWLER_TRACK_R");
        }

        if(!crawlerL || !crawlerR){
            os << "Crawlers are not found" << std::endl;
            return false;
        }

        io->enableOutput(crawlerL, JointVelocity);
        io->enableOutput(crawlerR, JointVelocity);

        if(!joystick.makeReady()){
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
        crawlerL->dq_target() = -4.0 * (pos[1] - pos[0]);
        crawlerR->dq_target() = -4.0 * (pos[1] + pos[0]);

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCrawlerJoystickController)
