#include "AbstractPose.h"

using namespace std;
using namespace cnoid;


AbstractPose::AbstractPose()
{
    seqLocalReferenceCounter = 0;
}


AbstractPose::AbstractPose(const AbstractPose& org)
    : name_(org.name_)
{
    seqLocalReferenceCounter = 0;
}


AbstractPose::~AbstractPose()
{

}


bool AbstractPose::hasSameParts(AbstractPose* pose) const
{
    return false;
}


