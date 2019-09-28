/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ScriptItem.h"
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

ScriptItem::ScriptItem()
{

}


ScriptItem::ScriptItem(const ScriptItem& org)
    : AbstractTextItem(org)
{

}


ScriptItem::~ScriptItem()
{

}


const std::string& ScriptItem::textFilename() const
{
    return scriptFilename();
}


std::string ScriptItem::identityName() const
{
    const string& name_ = name();
    const string fname = getFilename(stdx::filesystem::path(scriptFilename()));
    if(name_.empty()){
        return fname;
    }
    if(fname == name_){
        return name_;
    } else {
        return name_ + " (" + fname + ")";
    }
}


bool ScriptItem::isBackgroundMode() const
{
    return false;
}


bool ScriptItem::isRunning() const
{
    return false;
}


bool ScriptItem::executeCode(const char* code)
{
    return false;
}


bool ScriptItem::waitToFinish(double timeout)
{
    return false;
}


std::string ScriptItem::resultString() const
{
    return string();
}
