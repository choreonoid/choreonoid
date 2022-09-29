#include "PronunSymbol.h"
#include <cnoid/ValueTree>
#include <cnoid/YAMLWriter>

using namespace std;
using namespace cnoid;


PronunSymbol::PronunSymbol()
{

}


PronunSymbol::PronunSymbol(const PronunSymbol& org)
    : AbstractPose(org)
{

}


PronunSymbol::~PronunSymbol()
{

}


Referenced* PronunSymbol::doClone(CloneMap*) const
{
    return new PronunSymbol(*this);
}


bool PronunSymbol::restore(const Mapping& archive, const Body* body)
{
    return true;
}


void PronunSymbol::store(Mapping& archive, const Body* body) const
{
    archive.write("type", "PronunSymbol");
    archive.write("name", name(), DOUBLE_QUOTED);
}
