#include "ReferencedObjectSeq.h"
#include "CloneMap.h"

using namespace cnoid;


ReferencedObjectSeq::ReferencedObjectSeq(int nFrames)
    : BaseSeqType("ReferencedObjectSeq", nFrames)
{

}


ReferencedObjectSeq::ReferencedObjectSeq(const ReferencedObjectSeq& org, CloneMap* cloneMap)
    : BaseSeqType(org, cloneMap ? false : true)
{
    if(cloneMap){
        for(auto& object : org.container){
            container.push_back(cloneMap->getClone(object));
        }
    }
}


std::shared_ptr<AbstractSeq> ReferencedObjectSeq::cloneSeq() const
{
    return std::make_shared<ReferencedObjectSeq>(*this);
}


ReferencedObjectSeq::~ReferencedObjectSeq()
{

}
