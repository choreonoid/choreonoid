#include "AbstractPose.h"

using namespace std;
using namespace cnoid;


AbstractPose::AbstractPose()
{

}


AbstractPose::AbstractPose(const AbstractPose& org)
{

}


AbstractPose::~AbstractPose()
{

}


bool AbstractPose::hasSameParts(AbstractPose* pose) const
{
    return false;
}


