#include "MprStatement.h"
#include "MprProgram.h"
#include "BasicMprStatements.h"

using namespace std;
using namespace cnoid;

MprStatementClassRegistry& MprStatementClassRegistry::instance()
{
    static MprStatementClassRegistry registry;
    return registry;
}


MprStatementClassRegistry::MprStatementClassRegistry()
{

}


PolymorphicMprStatementFunctionSet::PolymorphicMprStatementFunctionSet()
    : PolymorphicFunctionSet<MprStatement>(MprStatementClassRegistry::instance())
{

}


MprStatement::MprStatement()
{
    classId_ = -1;
}


MprStatement::MprStatement(const MprStatement& org)
    : MprStatement()
{

}


void MprStatement::validateClassId() const
{
    classId_ = MprStatementClassRegistry::instance().classId(this);
}


MprProgram* MprStatement::holderProgram() const
{
    return holderProgram_.lock();
}


MprProgram* MprStatement::topLevelProgram() const
{
    auto program = holderProgram();
    if(!program){
        return nullptr;
    }
    return program->topLevelProgram();
}


MprProgram* MprStatement::getLowerLevelProgram()
{
    return nullptr;
}


void MprStatement::notifyUpdate()
{
    if(auto holder = holderProgram()){
        holder->notifyStatementUpdate(this);
    }
}


