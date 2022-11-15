#include <cnoid/LinkedJointHandler>
#include <cnoid/Body>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

class ClosedLinkSampleHandler : public LinkedJointHandler
{
public:
    virtual BodyHandler* clone(Body* body) override;
    virtual bool initialize(Body* body, std::ostream& os) override;
    virtual bool updateLinkedJointDisplacements(Link* masterJoint = nullptr) override;

private:
    Link* joints[3];
};

CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(ClosedLinkSampleHandler)


BodyHandler* ClosedLinkSampleHandler::clone(Body*)
{
    return new ClosedLinkSampleHandler(*this);
}


bool ClosedLinkSampleHandler::initialize(Body* body, std::ostream& os)
{
    const char* ids[3] = { "0", "1", "3" };
    for(int i=0; i < 3; ++i){
        joints[i] = body->link(format("J{0}", ids[i]));
        if(!joints[i]){
            return false;
        }
    }
    return true;
}


bool ClosedLinkSampleHandler::updateLinkedJointDisplacements(Link* masterJoint)
{
    if(!masterJoint || masterJoint == joints[0]){
        joints[1]->q() = -joints[0]->q();
        joints[2]->q() = joints[0]->q();
    } else if(masterJoint == joints[1]){
        joints[0]->q() = -joints[1]->q();
        joints[2]->q() = joints[0]->q();
    } else if(masterJoint == joints[2]){
        joints[0]->q() = joints[2]->q();
        joints[1]->q() = -joints[2]->q();
    }
    return true;
}
