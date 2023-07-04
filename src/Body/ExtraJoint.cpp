#include "ExtraJoint.h"
#include "Body.h"

using namespace std;
using namespace cnoid;

std::string ExtraJoint::bodyName(int which) const
{
    if(auto link = links[which]){
        if(auto body = link->body()){
            return body->name();
        }
    }
    return string();
}

