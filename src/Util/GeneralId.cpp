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


static void readGeneralId(GeneralId& id, ScalarNode* node)
{
    if(node->stringStyle() != PLAIN_STRING){
        id = node->toString();
    } else {
        auto str = node->toString();
        char* endptr;
        int number = strtol(str.c_str(), &endptr, 10);
        if(endptr == str.c_str()){
            id = str;
        } else {
            id = number;
        }
    }
}


bool GeneralId::read(const Mapping* archive, const char* key)
{
    auto idNode = archive->find(key);
    if(idNode->isValid() && idNode->isScalar()){
        readGeneralId(*this, idNode->toScalar());
        return true;
    }
    (*this)  = GeneralId(); // Set invalid ID
    return false;
}


bool GeneralId::read(const Mapping& archive, const char* key)
{
    return read(&archive, key);
}


void GeneralId::readEx(const Mapping* archive, const char* key)
{
    auto idNode = archive->get(key).toScalar();
    readGeneralId(*this, idNode);
}    


void GeneralId::readEx(const Mapping& archive, const char* key)
{
    readEx(&archive, key);
}    


bool GeneralId::write(Mapping* archive, const char* key) const
{
    if(isValid()){
        if(isInt()){
            archive->write(key, toInt());
        } else {
            archive->write(key, toString(), DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
}


bool GeneralId::write(Mapping& archive, const char* key) const
{
    return write(&archive, key);
}
