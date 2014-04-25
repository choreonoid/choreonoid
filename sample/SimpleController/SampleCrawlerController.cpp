/**
   Sample Crawler Controller
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class SampleCrawlerController : public cnoid::SimpleController
{ 
    int cnt;

public:
    virtual bool initialize()
        {
            cnt = 0;
            return true;
        }

    virtual bool control()
        {
            BodyPtr io = ioBody();

            if(cnt < 1000){
                io->joint(0)->u() = 0.0;
                io->joint(1)->u() = 0.0;

            } else if(cnt < 3000){
                io->joint(0)->u() = 1.5;
                io->joint(1)->u() = 1.5;	

            } else if(cnt < 4000){
                io->joint(0)->u() =  1.0;
                io->joint(1)->u() = -1.0;	
            } else {
                io->joint(0)->u() = 1.0;	
                io->joint(1)->u() = 1.0;	
            }
            cnt++;
        
            return true;
        }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCrawlerController)
