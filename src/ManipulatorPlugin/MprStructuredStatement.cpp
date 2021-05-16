#include "MprStructuredStatement.h"
#include "MprProgram.h"
#include "MprStatementRegistration.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


MprStructuredStatement::MprStructuredStatement()
    : attributes_(0)
{
    program_ = new MprProgram;
    program_->setHolderStatement(this);
}


MprStructuredStatement::MprStructuredStatement(const MprStructuredStatement& org, CloneMap* cloneMap)
    : MprStatement(org),
      attributes_(org.attributes_)
{
    if(cloneMap){
        program_ = cloneMap->getClone(org.program_);
    } else {
        if(hasStructuredStatementAttribute(ArbitraryLowerLevelProgram)){
            program_ = org.program_->clone();
        } else {
            program_ = new MprProgram;
        }
    }
    program_->setHolderStatement(this);
}


MprStructuredStatement::~MprStructuredStatement()
{

}


MprProgram* MprStructuredStatement::getLowerLevelProgram()
{
    return lowerLevelProgram();
}


bool MprStructuredStatement::isExpandedByDefault() const
{
    return true;
}


bool MprStructuredStatement::read(MprProgram* program, const Mapping& archive)
{
    if(hasStructuredStatementAttribute(ArbitraryLowerLevelProgram)){
        return program_->read(archive);
    }
    return true;
}


bool MprStructuredStatement::write(Mapping& archive) const
{
    if(hasStructuredStatementAttribute(ArbitraryLowerLevelProgram)){
        return program_->write(archive);
    }
    return true;
}


struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerAbstractType<MprStructuredStatement, MprStatement>();
    }
} registration;
