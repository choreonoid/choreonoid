/**
   @author Shin'ichiro Nakaoka
*/

#include "ControllerIO.h"
#include <boost/tokenizer.hpp>
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
    typedef boost::escaped_list_separator<char> separator;
    separator sep('\\', ' ');

    std::string s = optionString();
    boost::tokenizer<separator> tok(s, sep);

    std::vector<std::string> options;
    for(boost::tokenizer<separator>::iterator p = tok.begin(); p != tok.end(); ++p){
        const string& token = *p;
        if(!token.empty()){
            options.push_back(token);
        }
    }

    return options;
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

