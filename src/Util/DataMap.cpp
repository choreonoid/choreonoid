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
    auto& nameToIdMap_ = nameToIdMap();
    auto it = nameToIdMap_.find(name);
    if(it != nameToIdMap_.end()){
        return it->second;
    }
    int id = nextDynamicId();
    nameToIdMap_[name] = id;
    return id;
}


const std::string& DataMapBase::getDynamicIDname(int id)
{
    static std::string emptyString;
    
    const auto& idToNameMap_ = idToNameMap();
    auto it = idToNameMap_.find(id);
    if(it != idToNameMap_.end()){
        return it->second;
    }
    return emptyString;
}
