#include "BodyCloneMap.h"
#include "Body.h"
#include "Link.h"
#include "Device.h"

using namespace std;
using namespace cnoid;

BodyCloneMap::BodyCloneMap()
    : CloneMap(
        [this](const Referenced* org) -> Referenced* {
            if(auto link = dynamic_cast<const Link*>(org)){
                return link->clone();
            } else if(auto device = dynamic_cast<const Device*>(org)){
                return device->clone(*this);
            } else if(auto body = dynamic_cast<const Body*>(org)){
                return body->clone(*this);
            }
            return nullptr;
        })
{

}
