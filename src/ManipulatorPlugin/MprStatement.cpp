#include "MprStatement.h"
#include "MprProgram.h"
#include "MprBasicStatements.h"
#include <cnoid/ValueTree>

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
    isEnabled_ = true;
}


MprStatement::MprStatement(const MprStatement& org)
    : MprStatement()
{
    isEnabled_ = org.isEnabled_;
}


MprStatement::~MprStatement()
{

}


void MprStatement::validateClassId() const
{
    classId_ = MprStatementClassRegistry::instance().getClassId(this);
}


std::string MprStatement::label() const
{
    string text;
    for(int i=0; i < 3; ++i){
        auto element = label(i);
        if(!element.empty()){
            if(i >= 1){
                text += " ";
            }
            text += element;
        }
    }
    return text;
}


MprProgram* MprStatement::holderProgram() const
{
    return holderProgram_.lock();
}


MprStructuredStatement* MprStatement::holderStatement() const
{
    if(auto program = holderProgram_.lock()){
        return program->holderStatement();
    }
    return nullptr;
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


bool MprStatement::read(MprProgram* /* program */, const Mapping& archive)
{
    isEnabled_ = true;
    if(!archive.read("enabled", isEnabled_)){
        // For backward compatibility
        bool isCommentedOut;
        if(archive.read("isCommentedOut", isCommentedOut)){
            isEnabled_ = !isCommentedOut;
        }
    }
    return true;
}


bool MprStatement::write(Mapping& archive) const
{
    if(!isEnabled_){
        archive.write("enabled", false);
    }
    return true;
}

