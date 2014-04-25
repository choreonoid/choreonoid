/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "DyBody.h"
#include <cnoid/Exception>

using namespace std;
using namespace boost;
using namespace cnoid;


DyLink::DyLink()
{

}


DyLink::DyLink(const Link& link)
    : Link(link)
{

}


void DyLink::prependChild(Link* link)
{
    if(DyLink* dyLink = dynamic_cast<DyLink*>(link)){
        Link::prependChild(dyLink);
    } else {
        throw type_mismatch_error();
    }
}


void DyLink::appendChild(Link* link)
{
    if(DyLink* dyLink = dynamic_cast<DyLink*>(link)){
        Link::appendChild(dyLink);
    } else {
        throw type_mismatch_error();
    }
}


DyBody::DyBody()
{

}


DyBody::DyBody(const Body& org)
{
    copy(org);
}

    
Body* DyBody::clone() const
{
    return new DyBody(*this);
}


Link* DyBody::createLink(const Link* org) const
{
    return org ? new DyLink(*org) : new DyLink();
}
