/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_DATA_MAP_H
#define CNOID_UTIL_DATA_MAP_H

#include <map>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DataMapBase
{
public:
    static const int MIN_DYNAMIC_ID = 10000;
    virtual ~DataMapBase();

    int getDynamicID(const std::string& name);
    const std::string& getDynamicIDname(int id);
        
protected:
    virtual std::map<std::string, int>& nameToIdMap();
    virtual std::map<int, std::string>& idToNameMap();
    virtual int nextDynamicId();
};


template <class ElementType = double, class Allocator = std::allocator<ElementType>>
class DataMap : public DataMapBase,
                protected std::map<int, std::vector<ElementType, Allocator>>
{
    typedef std::map<int, std::vector<ElementType, Allocator>> MapType;

    static const std::vector<ElementType, Allocator> emptyData;
        
public:

    typedef std::vector<ElementType, Allocator> Data;

    DataMap() { }
    
    DataMap(const DataMap& org) : MapType(org) { }

    DataMap& operator=(const DataMap& rhs) {
        MapType::operator=(rhs);
        return *this;
    }

    bool operator==(const DataMap& rhs) const {
        return std::operator==(static_cast<MapType>(*this), static_cast<MapType>(rhs));
    }
    bool operator!=(const DataMap& rhs) const {
        return std::operator!=(static_cast<MapType>(*this), static_cast<MapType>(rhs));
    }

#ifndef _MSC_VER
    using typename MapType::iterator;
    using MapType::size;
    using MapType::empty;
    using MapType::clear;
    using MapType::begin;
    using MapType::end;
    using MapType::find;
#else
    // In VC++, the above declarations produce C2487 error in compiling a class
    // which inherits this class. The following code is used instead of the above one.
    typedef typename MapType::iterator iterator;
    size_t size() const { return MapType::size(); }
    bool empty() const { return MapType::empty(); }
    void clear() { MapType::clear(); }
    typename MapType::iterator begin() { return MapType::begin(); }
    typename MapType::const_iterator begin() const { return MapType::begin(); }
    typename MapType::iterator end() { return MapType::end(); }
    typename MapType::const_iterator end() const { return MapType::end(); }
    typename MapType::iterator find(const typename MapType::key_type& x) { return MapType::find(x); }
    typename MapType::const_iterator find(const typename MapType::key_type& x) const { return MapType::find(x); }
#endif
    Data& data(int id) { return (*this)[id]; }

    const Data& data(int id) const {
        typename MapType::const_iterator p = find(id);
        if(p != end()){
            return p->second;
        }
        return emptyData;
    }
};

template <class ElementType, class Allocator>
const std::vector<ElementType, Allocator> DataMap<ElementType, Allocator>::emptyData;

}

#endif
