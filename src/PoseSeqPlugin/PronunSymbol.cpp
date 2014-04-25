/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "PronunSymbol.h"
#include <cnoid/ValueTree>
#include <cnoid/YAMLWriter>

using namespace std;
using namespace cnoid;


PronunSymbol::PronunSymbol()
{

}


PronunSymbol::PronunSymbol(const PronunSymbol& org)
    : PoseUnit(org),
      actualPoseUnit_(org.actualPoseUnit_)
{

}


PronunSymbol::~PronunSymbol()
{

}


PoseUnit* PronunSymbol::duplicate()
{
    return new PronunSymbol(*this);
}


bool PronunSymbol::restore(const Mapping& archive, const BodyPtr body)
{
    return true;
}


void PronunSymbol::store(Mapping& archive, const BodyPtr body) const
{
    archive.write("type", "PronunSymbol");
    archive.write("name", name(), DOUBLE_QUOTED);
}
