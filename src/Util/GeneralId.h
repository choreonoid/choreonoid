#ifndef CNOID_UTIL_GENERAL_ID_H
#define CNOID_UTIL_GENERAL_ID_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT GeneralId
{
public:
    static GeneralId defaultId() { return GeneralId(0); }
    
    GeneralId()
        : valueType(Int), intId(-1) { }
    GeneralId(int id)
        : valueType(Int), intId(id) { }
    GeneralId(const std::string& id)
        : valueType(String), intId(0), stringId(id) { }
    GeneralId(const GeneralId& org)
        : valueType(org.valueType), intId(org.intId), stringId(org.stringId) { }

    GeneralId& operator=(const GeneralId& rhs){
        valueType = rhs.valueType;
        intId = rhs.intId;
        stringId = rhs.stringId;
        return *this;
    }
    GeneralId& operator=(int rhs){
        valueType = Int;
        intId = rhs;
        stringId.clear();
        return *this;
    }
    GeneralId& operator=(const std::string& rhs){
        valueType = String;
        intId = 0;
        stringId = rhs;
        return *this;
    }
    bool operator==(const GeneralId& rhs) const {
        if(valueType == Int){
            return rhs.valueType == Int && intId == rhs.intId;
        } else {
            return rhs.valueType == String && stringId == rhs.stringId;
        }
    }
    bool operator!=(const GeneralId& rhs) const {
        return !(this->operator==(rhs));
    }
    bool operator==(int rhs) const {
        return (valueType == Int && intId == rhs);
    }
    bool operator!=(int rhs) const {
        return !(this->operator==(rhs));
    }
    bool operator==(const std::string& rhs) const {
        return (valueType == String && stringId == rhs);
    }
    bool operator!=(const std::string& rhs) const {
        return !(this->operator==(rhs));
    }

    bool isValid() const { return intId >= 0; }
    bool isInt() const { return valueType == Int; }
    bool isString() const { return valueType == String; }
    int toInt() const { return intId; }
    const std::string& toString() const { return stringId; }
    std::string label() const;

    bool read(const Mapping& archive, const char* key);
    bool write(Mapping& archive, const char* key) const;

    struct Hash {
        typedef size_t result_type;
        result_type operator()(const GeneralId& key) const{
            if(key.isInt()){
                return std::hash<int>()(key.toInt());
            } else {
                return std::hash<std::string>()(key.toString());
            }
        }
    };
        
private:
    enum IdValueType { Int, String } valueType;
    int intId;
    std::string stringId;
};

}

#endif
