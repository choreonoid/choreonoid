#include "MprPositionStatement.h"
#include "MprProgram.h"
#include "MprStatementRegistration.h"
#include "MprPositionList.h"
#include <cnoid/ValueTree>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;


MprPositionStatement::MprPositionStatement()
{

}


MprPositionStatement::MprPositionStatement(const MprPositionStatement& org)
    : MprStatement(org),
      positionId_(org.positionId_)
{

}


Referenced* MprPositionStatement::doClone(CloneMap*) const
{
    return new MprPositionStatement(*this);
}
    

std::string MprPositionStatement::label(int index) const
{
    if(index == 0){
        return "Position";

    } else if(index == 1){
        return positionLabel();
    }
    return string();
}


std::string MprPositionStatement::positionLabel() const
{
    if(positionId_.isInt()){
        if(auto holder = holderProgram()){
            if(holder->hasLocalPositionList()){
                return format("({0})", positionId_.toInt());
            }
        }
        return format("P{0}", positionId_.toInt());

    } else {
        return positionId_.toString();
    }
}


MprPosition* MprPositionStatement::position()
{
    if(auto program = holderProgram()){
        return program->positionList()->findPosition(positionId_);
    }
    return nullptr;
}


const MprPosition* MprPositionStatement::position() const
{
    return const_cast<MprPositionStatement*>(this)->position();
}


bool MprPositionStatement::read(MprProgram* program, const Mapping& archive)
{
    string symbol;
    positionId_.read(archive, "position");
    return true;
}


bool MprPositionStatement::write(Mapping& archive) const
{
    archive.write("position", positionId_.label());
    return true;
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerAbstractType<MprPositionStatement, MprStatement>();
    }
} registration;

}
