/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleController.h"
#include <cnoid/NullOut>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class SimpleControllerImpl
{
public:
    mutable BodyPtr ioBody;
    double timeStep;
    boost::dynamic_bitset<> jointOutputFlags;
    bool isImmediateMode;
    mutable std::ostream* os;
    std::vector<std::string> options;
};

}
    

SimpleController::SimpleController()
{
    impl = new SimpleControllerImpl;
    impl->timeStep = 1.0;
    impl->isImmediateMode = false;
    impl->os = &nullout();
}


SimpleController::SimpleController(const SimpleController& org)
{
    impl = new SimpleControllerImpl;
    impl->timeStep = org.impl->timeStep;
    impl->isImmediateMode = org.impl->isImmediateMode;
    impl->os = org.impl->os;
    impl->options = org.impl->options;
}


SimpleController::~SimpleController()
{

}


void SimpleController::setOptions(const std::string& optionString)
{
    impl->options.clear();
    typedef boost::escaped_list_separator<char> separator;
    separator sep('\\', ' ');
    boost::tokenizer<separator> tok(optionString, sep);
    for(boost::tokenizer<separator>::iterator p = tok.begin(); p != tok.end(); ++p){
        const string& token = *p;
        if(!token.empty()){
            impl->options.push_back(token);
        }
    }
}
    

void SimpleController::setIoBody(Body* body)
{
    impl->ioBody = body;
    impl->jointOutputFlags.resize(body->numJoints(), true);
}


void SimpleController::setTimeStep(double timeStep)
{
    impl->timeStep = timeStep;
}


void SimpleController::setImmediateMode(bool on)
{
    impl->isImmediateMode = on;
}


void SimpleController::setOutputStream(std::ostream& os)
{
    impl->os = &os;
}

void SimpleController::setJointOutput(bool on)
{
    if(on){
        impl->jointOutputFlags.set();
    } else {
        impl->jointOutputFlags.reset();
    }
}
    

void SimpleController::setJointOutput(int jointId, bool on)
{
    impl->jointOutputFlags.set(jointId, on);
}

        
const boost::dynamic_bitset<>& SimpleController::jointOutputFlags() const
{
    return impl->jointOutputFlags;
}


const std::vector<std::string>& SimpleController::options() const
{
    return impl->options;
}


Body* SimpleController::ioBody()
{
    return impl->ioBody;
}


double SimpleController::timeStep() const
{
    return impl->timeStep;
}


bool SimpleController::isImmediateMode() const
{
    return impl->isImmediateMode;
}


std::ostream& SimpleController::os() const
{
    return *impl->os;
}
