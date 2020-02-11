#include "ManipulatorStatement.h"
#include "ManipulatorProgram.h"
#include "BasicManipulatorStatements.h"

using namespace std;
using namespace cnoid;

ManipulatorStatementClassRegistry& ManipulatorStatementClassRegistry::instance()
{
    static ManipulatorStatementClassRegistry registry;
    return registry;
}


ManipulatorStatementClassRegistry::ManipulatorStatementClassRegistry()
{

}


PolymorphicManipulatorStatementFunctionSet::PolymorphicManipulatorStatementFunctionSet()
    : PolymorphicFunctionSet<ManipulatorStatement>(ManipulatorStatementClassRegistry::instance())
{

}


ManipulatorStatement::ManipulatorStatement()
{
    classId_ = -1;
}


ManipulatorStatement::ManipulatorStatement(const ManipulatorStatement& org)
    : ManipulatorStatement()
{

}


void ManipulatorStatement::validateClassId() const
{
    classId_ = ManipulatorStatementClassRegistry::instance().classId(this);
}


ManipulatorProgram* ManipulatorStatement::holderProgram() const
{
    return holderProgram_.lock();
}


ManipulatorProgram* ManipulatorStatement::topLevelProgram() const
{
    auto program = holderProgram();
    if(!program){
        return nullptr;
    }
    return program->topLevelProgram();
}


ManipulatorProgram* ManipulatorStatement::getLowerLevelProgram()
{
    return nullptr;
}


void ManipulatorStatement::notifyUpdate()
{
    if(auto holder = holderProgram()){
        holder->notifyStatementUpdate(this);
    }
}


