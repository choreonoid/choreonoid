#include "MprBasicStatements.h"
#include "MprStatementRegistration.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MprEmptyStatement::MprEmptyStatement()
{

}


MprEmptyStatement::MprEmptyStatement(const MprEmptyStatement& org)
    : MprStatement(org)
{

}


Referenced* MprEmptyStatement::doClone(CloneMap*) const
{
    return new MprEmptyStatement(*this);
}


std::string MprEmptyStatement::label(int index) const
{
    return string();
}


bool MprEmptyStatement::read(MprProgram* program, const Mapping& archive)
{
    return true;
}


bool MprEmptyStatement::write(Mapping& archive) const
{
    return true;
}


MprDummyStatement::MprDummyStatement()
{

}


MprDummyStatement::MprDummyStatement(const MprDummyStatement& org)
    : MprEmptyStatement(org)
{

}


Referenced* MprDummyStatement::doClone(CloneMap*) const
{
    return new MprDummyStatement(*this);
}


std::string MprDummyStatement::label(int index) const
{
    if(index == 0){
        return "---";
    }
    return string();
}


bool MprDummyStatement::write(Mapping& archive) const
{
    return true;
}


MprCommentStatement::MprCommentStatement()
{

}


MprCommentStatement::MprCommentStatement(const MprCommentStatement& org)
    : MprStatement(org),
      comment_(org.comment_)
{

}


Referenced* MprCommentStatement::doClone(CloneMap*) const
{
    return new MprCommentStatement(*this);
}


std::string MprCommentStatement::label(int index) const
{
    if(index == 0){
        return format("# {}", comment_);
    }
    return string();
}


bool MprCommentStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("comment", comment_);
    return true;
}
    

bool MprCommentStatement::write(Mapping& archive) const
{
    archive.write("comment", comment_);
    return true;
}


MprGroupStatement::MprGroupStatement()
{
    setStructuredStatementAttribute(ArbitraryLowerLevelProgram);
}


MprGroupStatement::MprGroupStatement(const MprGroupStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      groupName_(org.groupName_)
{

}


Referenced* MprGroupStatement::doClone(CloneMap* cloneMap) const
{
    return new MprGroupStatement(*this);
}


std::string MprGroupStatement::label(int index) const
{
    if(index == 0){
        return "Group";
    } else if(index == 1){
        if(groupName_.empty()){
            return "---";
        } else {
            return groupName_;
        }
    }
    return string();
}


bool MprGroupStatement::read(MprProgram* program, const Mapping& archive)
{
    if(MprStructuredStatement::read(program, archive)){
        archive.read("group_name", groupName_);
        return true;
    }
    return false;
}


bool MprGroupStatement::write(Mapping& archive) const
{
    archive.write("group_name", groupName_, DOUBLE_QUOTED);
    return MprStructuredStatement::write(archive);    
}


MprConditionStatement::MprConditionStatement()
{
    setStructuredStatementAttribute(ArbitraryLowerLevelProgram);
}


MprConditionStatement::MprConditionStatement(const MprConditionStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      condition_(org.condition_)
{

}


bool MprConditionStatement::read(MprProgram* program, const Mapping& archive)
{
    if(MprStructuredStatement::read(program, archive)){
        archive.read("condition", condition_);
        return true;
    }
    return false;
}


bool MprConditionStatement::write(Mapping& archive) const
{
    archive.write("condition", condition_, DOUBLE_QUOTED);
    return MprStructuredStatement::write(archive);
}


MprIfStatement::MprIfStatement()
{

}


MprIfStatement::MprIfStatement(const MprIfStatement& org, CloneMap* cloneMap)
    : MprConditionStatement(org, cloneMap)
{
    
}


Referenced* MprIfStatement::doClone(CloneMap* cloneMap) const
{
    return new MprIfStatement(*this, cloneMap);
}


std::string MprIfStatement::label(int index) const
{
    if(index == 0){
        return "IF";
    } else if(index == 1){
        if(condition().empty()){
            return "---";
        }
        return condition();
    }
    return string();
}


bool MprIfStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprConditionStatement::read(program, archive);
}


bool MprIfStatement::write(Mapping& archive) const
{
    return MprConditionStatement::write(archive);
}


MprElseStatement::MprElseStatement()
{
    setStructuredStatementAttribute(ArbitraryLowerLevelProgram);
}


MprElseStatement::MprElseStatement(const MprElseStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap)
{

}


Referenced* MprElseStatement::doClone(CloneMap* cloneMap) const
{
    return new MprElseStatement(*this, cloneMap);
}


std::string MprElseStatement::label(int index) const
{
    if(index == 0){
        return "ELSE";
    }
    return string();
}


bool MprElseStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprStructuredStatement::read(program, archive);
}


bool MprElseStatement::write(Mapping& archive) const
{
    return MprStructuredStatement::write(archive);
}


MprWhileStatement::MprWhileStatement()
{

}


MprWhileStatement::MprWhileStatement(const MprWhileStatement& org, CloneMap* cloneMap)
    : MprConditionStatement(org, cloneMap)
{

}


Referenced* MprWhileStatement::doClone(CloneMap* cloneMap) const
{
    return new MprWhileStatement(*this, cloneMap);
}


std::string MprWhileStatement::label(int index) const
{
    if(index == 0){
        return "WHILE";
    } else if(index == 1){
        if(condition().empty()){
            return "---";
        }
        return condition();
    }
    return string();
}


bool MprWhileStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprConditionStatement::read(program, archive);
}


bool MprWhileStatement::write(Mapping& archive) const
{
    return MprConditionStatement::write(archive);
}


MprCallStatement::MprCallStatement()
{

}


MprCallStatement::MprCallStatement(const MprCallStatement& org)
    : MprStatement(org),
      programName_(org.programName_)
{

}


Referenced* MprCallStatement::doClone(CloneMap*) const
{
    return new MprCallStatement(*this);
}


std::string MprCallStatement::label(int index) const
{
    if(index == 0){
        return "Call";
    } else if(index == 1){
        if(programName_.empty()){
            return "-----";
        }
        return programName_;
    }
    return string();
}


bool MprCallStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("program", programName_);
    return true;
}


bool MprCallStatement::write(Mapping& archive) const
{
    archive.write("program", programName_, DOUBLE_QUOTED);
    return true;
}


MprAssignStatement::MprAssignStatement()
{

}


MprAssignStatement::MprAssignStatement(const MprAssignStatement& org)
    : MprStatement(org),
      variableExpression_(org.variableExpression_),
      valueExpression_(org.valueExpression_)
{

}
      

Referenced* MprAssignStatement::doClone(CloneMap*) const
{
    return new MprAssignStatement(*this);
}


std::string MprAssignStatement::label(int index) const
{
    if(index == 0){
        return "Assign";

    } else if(index == 1){
        if(variableExpression_.empty()){
            return "..... = ";
        } else {
            return variableExpression_ + " = ";
        }
    } else if(index == 2){
        if(valueExpression_.empty()){
            return ".....";
        } else {
            return valueExpression_;
        }
    }
    return string();
}


bool MprAssignStatement::read(MprProgram* program, const Mapping& archive)
{
    if(archive.read("variable", variableExpression_)){
        std::smatch match;
        if(regex_match(variableExpression_, match, regex("^[+-]?\\d+$"))){
            variableExpression_ = format("var[{}]", variableExpression_);
        }
    }
    if(!archive.read("value", valueExpression_)){
        archive.read("expression", valueExpression_); // for backward compatibility
    }
    return true;
}


bool MprAssignStatement::write(Mapping& archive) const
{
    archive.write("variable", variableExpression_);
    archive.write("value", valueExpression_, SINGLE_QUOTED);
    return true;
}


MprSignalStatement::MprSignalStatement()
{
    signalIndex_ = 0;
    on_ = false;
}


MprSignalStatement::MprSignalStatement(const MprSignalStatement& org)
    : MprStatement(org)
{
    signalIndex_ = org.signalIndex_;
    on_ = org.on_;
}


Referenced* MprSignalStatement::doClone(CloneMap*) const
{
    return new MprSignalStatement(*this);
}


std::string MprSignalStatement::label(int index) const
{
    if(index == 0){
        return "Set";
    } else if(index == 1){
        return format("Out[{0}]", signalIndex_);
    } else if(index == 2){
        return on_ ? "on" : "off";
    }
    return string();
}
        

bool MprSignalStatement::read(MprProgram* program, const Mapping& archive)
{
    if(!archive.read("signal_index", signalIndex_)){
        archive.read("signalIndex", signalIndex_); // Old
    }
    archive.read("on", on_);
    return true;
}


bool MprSignalStatement::write(Mapping& archive) const
{
    archive.write("signal_index", signalIndex_);
    archive.write("on", on_);
    return true;
}


MprWaitStatement::MprWaitStatement()
{
    conditionType_ = SignalInput;
    signalIndex_ = 0;
    signalStateCondition_ = true;
}


MprWaitStatement::MprWaitStatement(const MprWaitStatement& org)
    : MprStatement(org)
{
    conditionType_ = org.conditionType_;
    signalIndex_ = org.signalIndex_;
    signalStateCondition_ = org.signalStateCondition_;
}


Referenced* MprWaitStatement::doClone(CloneMap*) const
{
    return new MprWaitStatement(*this);
}


std::string MprWaitStatement::label(int index) const
{
    if(index == 0){
        return "Wait";
    } else if(index == 1){
        if(conditionType_ == SignalInput){
            return "Signal";
        } else {
            return "None";
        }
    } else if(index == 2){
        if(conditionType_ == SignalInput){
            return std::to_string(signalIndex_);
        }
    } else if(index == 3){
        if(conditionType_ == SignalInput){
            return signalStateCondition_ ? "On" : "Off";
        }
    }
    return string();
}
    

bool MprWaitStatement::read(MprProgram* program, const Mapping& archive)
{
    auto& typeNode = archive["condition_type"];
    string type = typeNode.toString();
    if(type == "signal_input"){
        signalIndex_ = archive["signal_index"].toInt();
        signalStateCondition_ = archive["condition"].toBool();
    } else {
        typeNode.throwException(_("Invalid condition type"));
    }
    return true;
}
    
    
bool MprWaitStatement::write(Mapping& archive) const
{
    if(conditionType_ == SignalInput){
        archive.write("condition_type", "signal_input");
        archive.write("signal_index", signalIndex_);
        archive.write("condition", signalStateCondition_);
        return true;
    }
    return false;
}
    

MprDelayStatement::MprDelayStatement()
{
    time_ = 1.0;
}


MprDelayStatement::MprDelayStatement(const MprDelayStatement& org)
    : MprStatement(org)
{
    time_ = org.time_;
}


Referenced* MprDelayStatement::doClone(CloneMap*) const
{
    return new MprDelayStatement(*this);
}
    

std::string MprDelayStatement::label(int index) const
{
    if(index == 0){
        return "Delay";
    } else if(index == 1){
        return format("{0:.3}", time_);
    }
    return string();
}


bool MprDelayStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("time", time_);
    return true;
}


bool MprDelayStatement::write(Mapping& archive) const
{
    archive.write("time", time_);
    return true;
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerType<MprEmptyStatement, MprStatement>("Empty")
            .registerType<MprDummyStatement, MprEmptyStatement>("Dummy")
            .registerType<MprCommentStatement, MprStatement>("Comment")
            .registerType<MprGroupStatement, MprStructuredStatement>("Group")
            .registerAbstractType<MprConditionStatement, MprStructuredStatement>()
            .registerType<MprIfStatement, MprConditionStatement>("If")
            .registerType<MprElseStatement, MprStructuredStatement>("Else")
            .registerType<MprWhileStatement, MprConditionStatement>("While")
            .registerType<MprCallStatement, MprStatement>("Call")
            .registerType<MprAssignStatement, MprStatement>("Assign")
            .registerType<MprSignalStatement, MprStatement>("SetSignal")
            .registerType<MprWaitStatement, MprStatement>("Wait")
            .registerType<MprDelayStatement, MprStatement>("Delay")
            ;
    }
} registration;

}
