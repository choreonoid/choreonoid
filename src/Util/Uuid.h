#ifndef CNOID_UTIL_UUID_H
#define CNOID_UTIL_UUID_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT Uuid
{
public:
    Uuid();
    Uuid(const Uuid& org);
    Uuid(const std::string& str);

    bool isNull() const;

    bool operator==(const Uuid& rhs) const;
    bool operator!=(const Uuid& rhs) const;
    bool operator<(const Uuid& rhs) const;
    bool operator>(const Uuid& rhs) const;
    bool operator<=(const Uuid& rhs) const;
    bool operator>=(const Uuid& rhs) const;
    
    std::string toString() const;

    bool read(const Mapping* archive, const char* key);
    bool read(const Mapping* archive);
    bool read(const Mapping& archive, const char* key) { return read(&archive, key); }
    bool read(const Mapping& archive) { return read(&archive); }

    size_t hash() const;

private:
    unsigned char data[16];
};

}

namespace std {

template<> struct hash<cnoid::Uuid>
{
    std::size_t operator()(const cnoid::Uuid& u) const {
        return u.hash();
    }
};
    
}

#endif
