#include "Uuid.h"
#include "ValueTree.h"

#ifdef _WIN32
#include <windows.h>
#include <cstring>
#else
#include <uuid.h>
#endif

using namespace std;
using namespace cnoid;

#ifdef _WIN32

Uuid::Uuid()
{
    UuidCreate((UUID*)data);
}

Uuid::Uuid(const Uuid& org)
{
    memcpy(data, org.data, 16);
}

Uuid::Uuid(const std::string& str)
{
    UuidFromString((RPC_CSTR)str.c_str(), (UUID*)data);
}

bool Uuid::isNull() const
{
    unsigned char nulldata[] = {
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
    };

    return memcmp(data, nulldata, 16);
}

bool Uuid::operator==(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) == 0;
}

bool Uuid::operator!=(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) != 0;
}

bool Uuid::operator<(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) < 0;
}

bool Uuid::operator>(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) > 0;
}

bool Uuid::operator<=(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) <= 0;
}

bool Uuid::operator>=(const Uuid& rhs) const
{
    return memcmp(data, rhs.data, 16) >= 0;
}

std::string Uuid::toString() const
{
    RPC_CSTR p;
    if(UuidToString((UUID*)data, &p) == RPC_S_OK){
        std::string s(reinterpret_cast<char*>(p));
        RpcStringFree(&p);
        return s;
    }
    return string();
}

bool Uuid::read(const Mapping* archive, const char* key)
{
    string s;
    if(archive->read(key, s)){
        if(UuidFromString((RPC_CSTR)s.c_str(), (UUID*)data) == RPC_S_OK){
            return true;
        }
    }
    return false;
}

#else

Uuid::Uuid()
{
    uuid_generate(data);
}


Uuid::Uuid(const Uuid& org)
{
    uuid_copy(data, org.data);
}


Uuid::Uuid(const std::string& str)
{
    if(uuid_parse(str.c_str(), data) < 0){
        uuid_clear(data);
    }
}

bool Uuid::isNull() const
{
    return uuid_is_null(data);
}

bool Uuid::operator==(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) == 0;
}


bool Uuid::operator!=(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) != 0;
}


bool Uuid::operator<(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) < 0;
}


bool Uuid::operator>(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) > 0;
}


bool Uuid::operator<=(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) <= 0;
}


bool Uuid::operator>=(const Uuid& rhs) const
{
    return uuid_compare(data, rhs.data) >= 0;
}

std::string Uuid::toString() const
{
    char s[37];
    uuid_unparse(data, s);
    return std::string(s);
}

bool Uuid::read(const Mapping* archive, const char* key)
{
    string s;
    if(archive->read(key, s)){
        return (uuid_parse(s.c_str(), data) == 0);
    }
    return false;
}

#endif


bool Uuid::read(const Mapping* archive)
{
    return read(archive, "uuid");
}


size_t Uuid::hash() const
{
    size_t seed = 0;
    for(int i=0; i < 16; ++i){
        seed ^= static_cast<size_t>(data[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
}
