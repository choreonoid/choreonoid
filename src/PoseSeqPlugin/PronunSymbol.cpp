#include "PronunSymbol.h"
#include <cnoid/ValueTree>
#include <cnoid/YAMLWriter>

using namespace std;
using namespace cnoid;


PronunSymbol::PronunSymbol()
{

}


PronunSymbol::PronunSymbol(const PronunSymbol& org)
    : AbstractPose(org),
      symbol_(org.symbol_)
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
    return archive.read("name", symbol_);
}


void PronunSymbol::store(Mapping& archive, const Body* body) const
{
    archive.write("type", "PronunSymbol");
    archive.write("name", symbol_, DOUBLE_QUOTED);
}
