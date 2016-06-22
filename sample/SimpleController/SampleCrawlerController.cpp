/**
   Sample Crawler Controller
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class SampleCrawlerController : public cnoid::SimpleController
{ 
    Body* ioBody;
    int cnt;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        io->setJointOutput(JOINT_TORQUE);
        ioBody = io->body();
        cnt = 0;
        return true;
    }

    virtual bool control()
    {
        if(cnt < 1000){
            ioBody->joint(0)->u() = 0.0;
            ioBody->joint(1)->u() = 0.0;

        } else if(cnt < 3000){
            ioBody->joint(0)->u() = 1.5;
            ioBody->joint(1)->u() = 1.5;

        } else if(cnt < 4000){
            ioBody->joint(0)->u() =  1.0;
            ioBody->joint(1)->u() = -1.0;

        } else {
            ioBody->joint(0)->u() = 1.0;
            ioBody->joint(1)->u() = 1.0;
        }
        cnt++;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCrawlerController)
