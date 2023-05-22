#include <cnoid/LinkedJointHandler>
#include <cnoid/Body>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

class ClosedLinkSampleHandler : public LinkedJointHandler
{
public:
    virtual BodyHandler* clone() override;
    virtual bool initialize(Body* body, std::ostream& os) override;
    virtual bool updateLinkedJointDisplacements(Link* masterJoint, double masterJointDisplacement) override;

private:
    Link* joints[3];
};

CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(ClosedLinkSampleHandler)


BodyHandler* ClosedLinkSampleHandler::clone()
{
    return new ClosedLinkSampleHandler(*this);
}


bool ClosedLinkSampleHandler::initialize(Body* body, std::ostream& os)
{
    const char* ids[3] = { "0", "1", "3" };
    for(int i=0; i < 3; ++i){
        string name(format("J{0}", ids[i]));
        joints[i] = body->link(name);
        if(!joints[i]){
            os << name << "is not found." << endl;
            return false;
        }
    }
    return true;
}


bool ClosedLinkSampleHandler::updateLinkedJointDisplacements(Link* masterJoint, double masterJointDisplacement)
{
    if(masterJoint){
        masterJoint->q() = masterJointDisplacement;
    }
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
