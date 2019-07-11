/**
   @author Shin'ichiro Nakaoka
*/

#include "ControllerIO.h"
#include <regex>
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
    std::string s = optionString();
    regex ws("\\s+");
    sregex_token_iterator iter(s.begin(), s.end(), ws, -1);
    sregex_token_iterator end;
    return std::vector<std::string>(iter, end);
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


bool ControllerIO::isNoDelayMode() const
{
    return false;
}


bool ControllerIO::setNoDelayMode(bool on)
{
    return false;
}

