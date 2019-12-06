#include "ManipulatorStatement.h"
#include "ManipulatorProgram.h"
#include "BasicManipulatorStatements.h"

using namespace std;
using namespace cnoid;


ManipulatorStatement::ManipulatorStatement()
{

}


ManipulatorStatement::ManipulatorStatement(const ManipulatorStatement& org)
{

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


