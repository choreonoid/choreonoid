/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "DataMap.h"

using namespace std;
using namespace cnoid;


DataMapBase::~DataMapBase()
{

}


std::map<std::string, int>& DataMapBase::nameToIdMap()
{
    static std::map<std::string, int> nameToIdMap_;
    return nameToIdMap_;
}


std::map<int, std::string>& DataMapBase::idToNameMap()
{
    static std::map<int, std::string> idToNameMap_;
    return idToNameMap_;
}


int DataMapBase::nextDynamicId()
{
    static int dynamicIdCounter = MIN_DYNAMIC_ID;
    return dynamicIdCounter++;
}


int DataMapBase::getDynamicID(const std::string& name)
{
    std::map<string, int> nameToIdMap_ = nameToIdMap();
    std::map<string, int>::iterator p = nameToIdMap_.find(name);
    if(p != nameToIdMap_.end()){
        return p->second;
    }
    int id = nextDynamicId();
    nameToIdMap_[name] = id;
    return id;
}


const std::string& DataMapBase::getDynamicIDname(int id)
{
    static std::string emptyString;
    
    std::map<int, string> idToNameMap_ = idToNameMap();
    std::map<int, string>::iterator p = idToNameMap_.find(id);
    if(p != idToNameMap_.end()){
        return p->second;
    }
    return emptyString;
}
