#include "BodyItemKinematicsKit.h"

using namespace std;
using namespace cnoid;


BodyItemKinematicsKit::BodyItemKinematicsKit(BodyItem* bodyItem)
    : bodyItem_(bodyItem)
{

}


BodyItemKinematicsKit::BodyItemKinematicsKit(const BodyItemKinematicsKit& org, CloneMap* cloneMap)
    : BodyKinematicsKit(org, cloneMap),
      bodyItem_(org.bodyItem_)
{

}


Referenced* BodyItemKinematicsKit::doClone(CloneMap* cloneMap) const
{
    return new BodyItemKinematicsKit(*this, cloneMap);
}
