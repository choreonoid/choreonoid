/**
   @author Shin'ichiro Nakaoka
*/

#include "ControllerIO.h"
#include <cnoid/Tokenizer>
#include <cnoid/YAMLReader>
#include <iostream>

using namespace std;
using namespace cnoid;


ControllerIO::~ControllerIO()
{

}


std::string ControllerIO::optionString() const
{
    return string();
}


std::string ControllerIO::getIntegratedOptionString(const std::string& opt1, const std::string& opt2) const
{
    if(!opt1.empty()){
        if(opt2.empty()){
            return opt1;
        } else {
            return opt1 + " " + opt2;
        }
    }
    return opt2;
}


std::vector<std::string> ControllerIO::options() const
{
    std::vector<std::string> options;
    auto s = optionString();
    for(auto& token : Tokenizer<EscapedListSeparator<char>>(s, EscapedListSeparator<char>('\\', ' '))){
        if(!token.empty()){
            options.push_back(token);
        }
    }
    return options;
}


MappingPtr ControllerIO::structuredOptions() const
{
    YAMLReader reader;
    MappingPtr node;
    bool isError = false;
    auto option = optionString();
    if(!option.empty()){
        if(reader.parse(optionString())){
            node = reader.document()->toMapping();
        } else {
            os() << (controllerName() + ": " + reader.errorMessage()) << endl;
            isError = true;
        }
    }
    if(!node && !isError){
        node = new Mapping;
    }
    return node;
}


std::ostream& ControllerIO::os() const
{
    return std::cout;
}


double ControllerIO::timeStep() const
{
    return 0.0;
}


double ControllerIO::currentTime() const
{
    return 0.0;
}


bool ControllerIO::enableLog()
{
    return false;
}


void ControllerIO::outputLog(Referenced* frameLog)
{

}


bool ControllerIO::isNoDelayMode() const
{
    return false;
}


bool ControllerIO::setNoDelayMode(bool on)
{
    return false;
}

