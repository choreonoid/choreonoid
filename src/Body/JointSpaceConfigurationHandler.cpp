#include "JointSpaceConfigurationHandler.h"

using namespace std;
using namespace cnoid;


int JointSpaceConfigurationHandler::getCurrentNearSingularPointState() const
{
    return 0;
}


std::vector<std::string> JointSpaceConfigurationHandler::getNearSingularPointFactorNames(int state) const
{
    return std::vector<std::string>();
}


std::string JointSpaceConfigurationHandler::getNearSingularPointFactorString(int state) const
{
    string factors;
    for(auto& factor : getNearSingularPointFactorNames(state)){
        if(!factors.empty()){
            factors += ", ";
        }
        factors += factor;
    }
    return factors;
}
