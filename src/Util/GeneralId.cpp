#include "GeneralId.h"
#include "ValueTree.h"
#include <cstdlib>

using namespace std;
using namespace cnoid;

std::string GeneralId::label() const
{
    if(valueType == Int){
        return std::to_string(intId);
    } else {
        return stringId;
    }
}


bool GeneralId::read(const Mapping& archive, const char* key)
{
    auto idNode = archive.find(key);
    if(idNode->isValid() && idNode->isScalar()){
        auto scalar = static_cast<ScalarNode*>(idNode);
        if(scalar->stringStyle() != PLAIN_STRING){
            (*this) = idNode->toString();
        } else {
            auto s = idNode->toString();
            char* endptr;
            int id = strtol(s.c_str(), &endptr, 10);
            if(endptr == s.c_str()){
                (*this) = s;
            } else {
                (*this) = id;
            }
        }
        return true;
    }
    return false;
}


bool GeneralId::write(Mapping& archive, const char* key) const
{
    if(isValid()){
        if(isInt()){
            archive.write(key, toInt());
        } else {
            archive.write(key, toString(), DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}
