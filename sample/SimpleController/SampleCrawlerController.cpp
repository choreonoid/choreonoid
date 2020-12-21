/**
   Sample Crawler Controller
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class SampleCrawlerController : public SimpleController
{
    Link* crawlerL;
    Link* crawlerR;
    double time;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        crawlerL = io->body()->link("CRAWLER_TRACK_L");
        crawlerR = io->body()->link("CRAWLER_TRACK_R");

        if(!crawlerL || !crawlerR){
            io->os() << "Crawlers are not found" << std::endl;
            return false;
        }

        io->enableOutput(crawlerL, JointVelocity);
        io->enableOutput(crawlerR, JointVelocity);

        time = 0.0;
        dt = io->timeStep();
        
        return true;
    }

    virtual bool control() override
    {
        if(time < 2.0){
            crawlerL->dq_target() = 1.5;
            crawlerR->dq_target() = 1.5;

        } else if(time < 3.0){
            crawlerL->dq_target() =  1.0;
            crawlerR->dq_target() = -1.0;

        } else if(time < 5.0){
            crawlerL->dq_target() = 1.5;
            crawlerR->dq_target() = 1.5;

        } else if(time < 6.0){
            crawlerL->dq_target() = -1.0;
            crawlerR->dq_target() =  1.0;
        }

        time = fmod(time + dt, 6.0);

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCrawlerController)
